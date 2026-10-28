/*
 * tc_can.c - MCMCAN CAN0 node 0 for DroneCAN (see tc_can.h). Board only.
 */
#include "tc_can.h"

#include "Ifx_Types.h"
#include "IfxCan_Can.h"
#include "IfxCan.h"
#include "IfxPort.h"

#include "hal_pins.h"
#include "memplace.h"

SHARED_BEGIN()          /* core 0 writes them at the start, CPU1 reads them (can_send_reserved): not cached */
static volatile boolean s_res_up;
static Ifx_CAN_TXMSG * volatile s_res_elem[CAN_TX_RESERVED];
SHARED_END()

static IfxCan_Can      s_mod;
static IfxCan_Can_Node s_node;
static bool            s_up;
static const IfxCan_Can_Pins s_pins = {
    &CAN_TXD, IfxPort_OutputMode_pushPull,
    &CAN_RXD, IfxPort_InputMode_noPullDevice,
    IfxPort_PadDriver_cmosAutomotiveSpeed2
};

/* Called once from hal_init (before the control loop): the iLLD start-up has
 * short bounded register handshakes (module enable, INIT / CCE), nothing waits
 * on the bus. */
bool can_start(void)
{
    /* transceiver out of stand-by (TO CONFIRM: pin + polarity) */
    if (CAN_STB_NORMAL) IfxPort_setPinHigh(PIN_CAN_STB); else IfxPort_setPinLow(PIN_CAN_STB);
    IfxPort_setPinModeOutput(PIN_CAN_STB, IfxPort_OutputMode_pushPull, IfxPort_OutputIdx_general);

    IfxCan_Can_Config mc;
    IfxCan_Can_initModuleConfig(&mc, &MODULE_CAN0);
    IfxCan_Can_initModule(&s_mod, &mc);

    IfxCan_Can_NodeConfig nc;
    IfxCan_Can_initNodeConfig(&nc, &s_mod);
    nc.nodeId = IfxCan_NodeId_0;
    nc.clockSource = IfxCan_ClockSource_both;
    nc.frame.type = IfxCan_FrameType_transmitAndReceive;
    nc.frame.mode = IfxCan_FrameMode_standard;              /* classic CAN 2.0B (no FD) */
    nc.baudRate.baudrate = CAN_BITRATE;
    nc.baudRate.samplePoint = CAN_SAMPLE_POINT;
    nc.calculateBitTimingValues = TRUE;
    nc.txConfig.txMode = IfxCan_TxMode_fifo;
    nc.txConfig.dedicatedTxBuffersNumber = CAN_TX_RESERVED;   /* buffer 0: CPU1 only (11e) */
    nc.txConfig.txFifoQueueSize = CAN_TX_FIFO;
    nc.txConfig.txBufferDataFieldSize = IfxCan_DataFieldSize_8;
    nc.txConfig.txEventFifoSize = 0u;
    nc.rxConfig.rxMode = IfxCan_RxMode_fifo0;
    nc.rxConfig.rxFifo0Size = CAN_RX_FIFO;
    nc.rxConfig.rxFifo0DataFieldSize = IfxCan_DataFieldSize_8;
    nc.rxConfig.rxFifo0OperatingMode = IfxCan_RxFifoMode_overwrite;   /* full: the oldest goes (never stuck) */
    nc.filterConfig.messageIdLength = IfxCan_MessageIdLength_extended;
    nc.filterConfig.standardListSize = 0u;
    nc.filterConfig.extendedListSize = 0u;
    nc.filterConfig.standardFilterForNonMatchingFrames = IfxCan_NonMatchingFrame_reject;
    nc.filterConfig.extendedFilterForNonMatchingFrames = IfxCan_NonMatchingFrame_acceptToRxFifo0;
    nc.filterConfig.rejectRemoteFramesWithExtendedId = TRUE;
    nc.filterConfig.rejectRemoteFramesWithStandardId = TRUE;
    nc.pins = &s_pins;
    nc.busLoopbackEnabled = FALSE;
    s_up = IfxCan_Can_initNode(&s_node, &nc) == TRUE;
    if (s_up) {
        /* CPU1's reserved buffers: their message RAM elements, once (CPU1 never touches core 0's node struct) */
        for (uint32_t b = 0; b < CAN_TX_RESERVED; b++)
            s_res_elem[b] = IfxCan_Node_getTxBufferElementAddress(s_node.node, s_node.messageRAM.baseAddress,
                                                                  s_node.messageRAM.txBuffersStartAddress,
                                                                  (IfxCan_TxBufferId)b);
        __dsync();
        s_res_up = TRUE;
    }
    return s_up;
}

/* every pending frame of the TX FIFO (buffers CAN_TX_RESERVED .. + CAN_TX_FIFO - 1): one write to TXBCR
 * (writing 0 has no effect, so CPU1's reserved buffers stay) - from any core */
void can_cancel_fifo(void)
{
    if (s_res_up == FALSE) return;
    MODULE_CAN0.N[0].TX.BCR.U = ((1u << CAN_TX_FIFO) - 1u) << CAN_TX_RESERVED;
}

/* CPU1: one frame into its reserved buffer buf (0..CAN_TX_RESERVED - 1); still pending = false */
bool can_send_reserved(uint8_t buf, const dc_frame_t *f)
{
    if (s_res_up == FALSE || buf >= CAN_TX_RESERVED || f->len > 8u) return false;
    Ifx_CAN_N *n = &MODULE_CAN0.N[0];
    if (IfxCan_Node_isTxBufferRequestPending(n, (IfxCan_TxBufferId)buf)) return false;
    Ifx_CAN_TXMSG *e = s_res_elem[buf];
    IfxCan_Node_setMsgId(e, f->id & 0x1FFFFFFFu, IfxCan_MessageIdLength_extended);
    e->T0.B.RTR = 0u;
    e->T1.B.EFC = 0u;                           /* no TX event (the element RAM is not cleared at start) */
    e->T1.B.MM = 0u;
    e->T1.B.DLC = f->len;
    uint32 d[2] = { 0u, 0u };
    for (uint32_t i = 0; i < f->len; i++) d[i / 4u] |= (uint32)f->data[i] << (8u * (i % 4u));
    IfxCan_Node_writeTxBufData(e, (IfxCan_DataLengthCode)f->len, d);
    IfxCan_Node_setFrameModeReq(e, IfxCan_FrameMode_standard);
    __dsync();
    n->TX.BAR.U = 1u << buf;                    /* only our bit (zeros have no effect) */
    return true;
}

bool can_send(const dc_frame_t *f)
{
    if (!s_up || f->len > 8u) return false;
    if (IfxCan_Can_isTxFifoQueueFull(&s_node)) return false;
    IfxCan_Message m;
    IfxCan_Can_initMessage(&m);
    m.messageId = f->id & 0x1FFFFFFFu;
    m.messageIdLength = IfxCan_MessageIdLength_extended;
    m.dataLengthCode = (IfxCan_DataLengthCode)f->len;
    m.frameMode = IfxCan_FrameMode_standard;
    m.storeInTxFifoQueue = TRUE;
    uint32 d[2] = { 0u, 0u };
    for (uint32_t i = 0; i < f->len; i++) d[i / 4u] |= (uint32)f->data[i] << (8u * (i % 4u));
    return IfxCan_Can_sendMessage(&s_node, &m, d) == IfxCan_Status_ok;
}

bool can_recv(dc_frame_t *f)
{
    if (!s_up) return false;
    while (IfxCan_Can_getRxFifo0FillLevel(&s_node) > 0u) {      /* at most the FIFO size */
        IfxCan_Message m;
        uint32 d[2] = { 0u, 0u };
        IfxCan_Can_initMessage(&m);
        m.readFromRxFifo0 = TRUE;
        IfxCan_Can_readMessage(&s_node, &m, d);
        if (m.messageIdLength != IfxCan_MessageIdLength_extended || (uint32_t)m.dataLengthCode > 8u) continue;
        f->id = m.messageId & 0x1FFFFFFFu;
        f->len = (uint8_t)m.dataLengthCode;
        for (uint32_t i = 0; i < 8u; i++) f->data[i] = (uint8_t)(d[i / 4u] >> (8u * (i % 4u)));
        return true;
    }
    return false;
}

uint8_t can_state(uint8_t *tec, uint8_t *rec)
{
    if (!s_up) {
        *tec = *rec = 0u;
        return DC_CTRL_BUSOFF;
    }
    Ifx_CAN_N *n = s_node.node;
    *tec = (uint8_t)n->ECR.B.TEC;
    *rec = (uint8_t)n->ECR.B.REC;
    if (IfxCan_Node_getBusOffStatus(n) == IfxCan_CanNodeBusOffErrorStatus_BusOffErr) return DC_CTRL_BUSOFF;
    if (IfxCan_Node_isErrorPassive(n)) return DC_CTRL_PASSIVE;
    return DC_CTRL_ACTIVE;
}

void can_recover(void)
{
    if (!s_up) return;
    IfxCan_Node_setInitialisation(s_node.node, FALSE);     /* one register write: the bus-off recovery starts */
}
