/*
 * tc_adc.c - Analog inputs (EVADC). See tc_adc.h. Board only (ADS build).
 */
#include "tc_adc.h"

#include "Ifx_Types.h"
#include "Evadc/Adc/IfxEvadc_Adc.h"

#include "hal_pins.h"

static IfxEvadc_Adc s_evadc;
static IfxEvadc_Adc_Group s_grp;
static IfxEvadc_Adc_Channel s_ch[ADC_COUNT];
static boolean s_up = FALSE;

void adc_init(void)
{
    IfxEvadc_Adc_Config mc;
    IfxEvadc_Adc_initModuleConfig(&mc, &MODULE_EVADC);
    IfxEvadc_Adc_initModule(&s_evadc, &mc);

    IfxEvadc_Adc_GroupConfig gc;
    IfxEvadc_Adc_initGroupConfig(&gc, &s_evadc);
    gc.groupId = ADC_MOTOR_GROUP;
    gc.master = gc.groupId;
    gc.arbiter.requestSlotQueue0Enabled = TRUE;
    gc.queueRequest[0].triggerConfig.gatingMode = IfxEvadc_GatingMode_always;
    IfxEvadc_Adc_initGroup(&s_grp, &gc);

    /* one group (8) for both channels: the motor current sense (ch 4) and the
     * supply voltage divider (ch 5), measured in turns by the refilling queue */
    static const IfxEvadc_ChannelId ids[ADC_COUNT] = { ADC_MOTOR_CHANNEL, ADC_SUPPLY_CHANNEL };
    for (int i = 0; i < (int)ADC_COUNT; i++) {
        IfxEvadc_Adc_ChannelConfig cc;
        IfxEvadc_Adc_initChannelConfig(&cc, &s_grp);
        cc.channelId = ids[i];
        cc.resultRegister = (IfxEvadc_ChannelResult)ids[i];           /* result register = channel number */
        IfxEvadc_Adc_initChannel(&s_ch[i], &cc);
        IfxEvadc_Adc_addToQueue(&s_ch[i], IfxEvadc_RequestSource_queue0, IFXEVADC_QUEUE_REFILL);
    }
    IfxEvadc_Adc_startQueue(&s_grp, IfxEvadc_RequestSource_queue0);
    s_up = TRUE;
}

bool adc_read(adc_in_t in, uint16_t *raw)
{
    if (!s_up || in >= ADC_COUNT) return false;
    Ifx_EVADC_G_RES r = IfxEvadc_Adc_getResult(&s_ch[in]);
    if (!r.B.VF) return false;                      /* no new conversion since the last read */
    *raw = (uint16_t)r.B.RESULT;
    return true;
}

float adc_volts(uint16_t raw)
{
    return (float)raw * (3.3f / 4095.0f);
}
