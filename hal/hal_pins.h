/*
 * hal_pins.h - Every TC375 pin, peripheral and interrupt priority the board
 * HAL uses, in one place (WIRING.md has the same table for the wiring).
 *
 * Pins marked CHECK are not on a documented Arduino / Shield2Go position:
 * they are on the X1 / X2 expansion headers. Check on the board's silk
 * screen / the Lite Kit user manual that the header pin really carries this
 * port pin BEFORE you wire it, and change it here if needed (the bench test,
 * FIRST_TEST.md, shows each one live).
 */
#ifndef HAL_PINS_H
#define HAL_PINS_H

/* --- outputs (WIRING.md section 3) ------------------------------------------ */
#define PIN_THROTTLE_PWM    &MODULE_P02, 7u     /* D9, GTM TOM0 ch 15 */
#define PIN_GEAR_PWM        &MODULE_P02, 6u     /* D8, GTM TOM0 ch 14 */
#define PIN_RELAY           &MODULE_P10, 5u     /* D10, high = safety relay closed */
#define PIN_ALARM_LED       &MODULE_P00, 5u     /* LED1 on the board, low = on */
/* ENGINE KILL relay (outboard kill-switch circuit, UNCREWED lost link): TBD - pin, relay
 * type and fail-safe direction not decided yet (WIRING.md 3A). Until then hal_set_engine_kill
 * drives no pin. When chosen: #define PIN_ENGINE_KILL &MODULE_Pxx, n and ENGINE_KILL_ACTIVE_HIGH. */

/* DP thrusters (WIRING.md 3C, CLAUDE.md P): TBD - the bow tunnel thruster, the transom motor, their
 * controllers and the pins are not chosen yet. Assumed: bidirectional controllers with an RC-style input
 * (50 Hz, THR_STOP_US = stop, +-THR_SPAN_US = full one way / the other), driven like the servos from GTM
 * TOM0 channels 12 / 13 (free on TGC1 next to the servo channels 14 / 15; shadow registers, taken at the
 * period end). Pins CHECK X1/X2: TOM0_12 -> P22.3 (TOUT50), TOM0_13 -> P11.10 (TOUT99). HAL_THRUSTERS_FITTED
 * 0 until they are wired: the channels run at the stop pulse but the pins are never touched / routed (no
 * pulses = the controllers stop). A controller with PWM + DIR inputs instead needs a change here and in
 * hal_tc375.c (TBD). The e-stop contactor cuts the thruster power itself (WIRING.md 3C). */
#define PIN_THR_BOW_PWM     &MODULE_P22, 3u     /* CHECK X1/X2, GTM TOM0 ch 12 */
#define PIN_THR_STERN_PWM   &MODULE_P11, 10u    /* CHECK X1/X2, GTM TOM0 ch 13 */
#define THR_BOW_TOUT        IfxGtm_TOM0_12_TOUT50_P22_3_OUT
#define THR_STERN_TOUT      IfxGtm_TOM0_13_TOUT99_P11_10_OUT
#define HAL_THRUSTERS_FITTED 0                  /* TBD: 1 when the thruster controllers are wired */
#define THR_STOP_US         1500u               /* TBD: per the controllers */
#define THR_SPAN_US         400u                /* 1100 us = full astern / to port, 1900 us = full ahead / to starboard */

/* Pololu G2 High-Power Motor Driver 18v17 (steering motor) */
#define PIN_MOTOR_PWM       &MODULE_P02, 5u     /* D6 -> G2 PWM (on / off: full speed or stopped) */
#define PIN_MOTOR_DIR       &MODULE_P02, 4u     /* D7 -> G2 DIR */
#define PIN_MOTOR_SLP       &MODULE_P33, 7u     /* CHECK X1/X2 -> G2 SLP (low = driver asleep = off) */
/* G2 CS (current sense, ~20 mV/A) -> Arduino A3 = P40.6 / AN36 CHECK. iLLD: group 8 channel 4
 * (IfxEvadc_G8CH4_P40_6_IN; the board manual calls group 8 "G4" = secondary group 0) */
#define ADC_MOTOR_GROUP     IfxEvadc_GroupId_8
#define ADC_MOTOR_CHANNEL   IfxEvadc_ChannelId_4
#define MOTOR_CS_V_PER_A    0.020f              /* G2 18v17 current sense */
#define MOTOR_CS_OFFSET_V   0.050f              /* its zero-current offset (measure on the bench) */
/* Supply voltage of the 12 V bank (WIRING.md 3B): a resistor divider on Arduino A2 = P40.7 / AN37 CHECK
 * (iLLD: group 8 channel 5, the same EVADC group as the motor current). TBD: the pin and the
 * divider are not wired yet - HAL_SUPPLY_FITTED 0 makes hal_read_supply_voltage return "not
 * measured" (the boat shows NO VOLTAGE DATA, no alarm). Set 1 once the divider is on the board
 * and SUPPLY_DIV_RATIO was measured (bank volts / volts at the pin; 47 k over 10 k = 5.7:
 * 15 V -> 2.63 V, below the 3.3 V reference). NEVER a pin above 3.3 V: the divider must be fitted. */
#define ADC_SUPPLY_GROUP    IfxEvadc_GroupId_8
#define ADC_SUPPLY_CHANNEL  IfxEvadc_ChannelId_5
#define HAL_SUPPLY_FITTED   0                   /* TBD: 1 when the divider is wired and the ratio measured */
#define SUPPLY_DIV_RATIO    5.7f                /* (47 k + 10 k) / 10 k, measure the real one */

#define PIN_BUZZER          &MODULE_P33, 3u     /* CHECK X1/X2 -> transistor -> 12 V piezo buzzer, high = on */

/* --- inputs (internal pull-up) ------------------------------------------------ */
/* panel buttons (WIRING.md section 2): normally open to GND, pressed = low */
#define PIN_BTN_STANDBY     &MODULE_P00, 4u
#define PIN_BTN_HDG_LOCK    &MODULE_P10, 8u
#define PIN_BTN_MINUS_1     &MODULE_P23, 4u
#define PIN_BTN_PLUS_1      &MODULE_P23, 5u
#define PIN_BTN_MINUS_10    &MODULE_P32, 2u
#define PIN_BTN_PLUS_10     &MODULE_P32, 3u
/* steering end stops: normally CLOSED to GND - open (high) = at the end stop or wire broken */
#define PIN_LIMIT_LEFT      &MODULE_P33, 1u     /* CHECK X1/X2 */
#define PIN_LIMIT_RIGHT     &MODULE_P33, 2u     /* CHECK X1/X2 */
#define HAL_LIMITS_FITTED   1
/* emergency stop: its second contact, normally CLOSED to GND - open (high) = pressed or wire broken */
#define PIN_ESTOP           &MODULE_P33, 0u     /* CHECK X1/X2 */
#define HAL_ESTOP_FITTED    1                   /* 0 = no e-stop contact wired (bench without it) */
#define PIN_NEUTRAL_SW      &MODULE_P02, 3u     /* D5, closed to GND in neutral */
#define HAL_NEUTRAL_SW_FITTED 0                 /* 1 when the gearbox switch is wired */

/* --- BNO085 IMU (SPI on QSPI1 = the Arduino SPI pins) -------------------------- */
/* breakout in SPI mode: PS0 and PS1 to 3.3 V (PS0 is WAKE in SPI mode: here a GPIO) */
#define BNO_QSPI            MODULE_QSPI1
#define BNO_SCLK            IfxQspi1_SCLK_P10_2_OUT     /* D13 CHECK */
#define BNO_MTSR            IfxQspi1_MTSR_P10_3_OUT     /* D11 CHECK (to the BNO085 DI) */
#define BNO_MRST            IfxQspi1_MRSTA_P10_1_IN     /* D12 CHECK (from the BNO085 SDA/MISO) */
#define PIN_BNO_CS          &MODULE_P10, 0u     /* CHECK X1/X2, software chip select */
#define PIN_BNO_INT         &MODULE_P33, 5u     /* CHECK X1/X2, H_INTN: low = data ready */
#define PIN_BNO_RST         &MODULE_P33, 4u     /* CHECK X1/X2, NRST: low = reset */
#define PIN_BNO_WAKE        &MODULE_P33, 6u     /* CHECK X1/X2, PS0/WAKE: low = wake up to receive */
#define BNO_SPI_HZ          2000000.0f          /* <= 3 MHz; 2 MHz: a full packet in ~1.5 ms */
/* direction conventions, check on the bench (FIRST_TEST.md): turn the boat clockwise ->
 * heading goes up and the yaw rate is positive; bow up -> pitch positive; starboard down ->
 * roll positive. Flip a sign here if one is the wrong way; the offset = mounting error. */
#define BNO_HEADING_SIGN    1.0f
#define BNO_HEADING_OFFSET  0.0f
#define BNO_YAWRATE_SIGN    (-1.0f)             /* the sensor's Z points up: counter-clockwise positive */
#define BNO_PITCH_SIGN      (-1.0f)
#define BNO_ROLL_SIGN       1.0f

/* --- microSD click (mikroBUS, QSPI2), WIRING.md section 4 ----------------------- */
#define SD_QSPI             MODULE_QSPI2
#define SD_SCLK             IfxQspi2_SCLK_P15_8_OUT
#define SD_MTSR             IfxQspi2_MTSR_P15_6_OUT
#define SD_MRST             IfxQspi2_MRSTB_P15_7_IN
#define PIN_SD_CS           &MODULE_P14, 7u     /* software chip select */
#define PIN_SD_CD           &MODULE_P10, 7u     /* card detect: low = card inserted */

/* --- UARTs ------------------------------------------------------------------------ */
#define LINK_ASC            MODULE_ASCLIN0      /* USB virtual COM (on-board debugger), 921600 */
#define LINK_TX             IfxAsclin0_TX_P14_0_OUT
#define LINK_RX             IfxAsclin0_RXA_P14_1_IN
#define GW_ASC              MODULE_ASCLIN1      /* Raspberry Pi gateway, 921600 */
#define GW_TX               IfxAsclin1_TX_P15_0_OUT
#define GW_RX               IfxAsclin1_RXA_P15_1_IN
#define GPS_ASC             MODULE_ASCLIN2      /* u-blox MAX-M10S on Shield2Go 1 */
#define GPS_TX              IfxAsclin2_TX_P33_9_OUT     /* to the GPS RXD */
#define GPS_RX              IfxAsclin2_RXE_P33_8_IN     /* from the GPS TXD */
#define GPS_BAUD_DEFAULT    9600.0f             /* the module's factory setting */
#define GPS_BAUD            38400.0f            /* what we switch it to */
#define BRIDGE_ASC          MODULE_ASCLIN3      /* Teensy USB joystick bridge, receive only, 230400 */
#define BRIDGE_TX           IfxAsclin3_TX_P20_0_OUT
#define BRIDGE_RX           IfxAsclin3_RXC_P20_3_IN
/* RS485 Modbus RTU relay board (WIRING.md 3D, CLAUDE.md M): TBD - the board, its baud rate and the
 * pins are not chosen yet. ASCLIN4 is free (ASCLIN0-3 = USB, gateway, GPS, bridge); its only LQFP176
 * pins are TX P00.9 / RX P00.12 (CHECK on the X1/X2 headers). The transceiver (MAX3485 class, 3.3 V)
 * needs a driver-enable pin: DE and /RE tied together on PIN_RELAY_DE (high = transmit). HAL_RELAY_FITTED
 * 0 until the board is wired: the driver is then never started (the boat shows RELAY BOARD SILENT as
 * soon as a boat file with relays is loaded - an advisory, nothing else changes). */
#define RELAY_ASC           MODULE_ASCLIN4
#define RELAY_TX            IfxAsclin4_TX_P00_9_OUT     /* CHECK X1/X2 */
#define RELAY_RX            IfxAsclin4_RXA_P00_12_IN    /* CHECK X1/X2 */
#define PIN_RELAY_DE        &MODULE_P00, 11u            /* CHECK X1/X2: RS485 DE + /RE, high = drive the bus */
#ifndef HAL_RELAY_FITTED                                /* (-D only for the gate's compile of the board path) */
#define HAL_RELAY_FITTED    0                           /* TBD: 1 when the board + transceiver are wired */
#endif
#define RELAY_BAUD          9600.0f                     /* TBD: per the board (9600-115200 8N1) */
#define RELAY_SLAVE_ADDR    1u                          /* TBD: the board's Modbus address */
#define RELAY_CHANNELS      8u                          /* TBD: coils the board has (8-16); read back per poll */
/* Which relay board (owner decision OD-9, step 11f): the RS485 Modbus board above, or the DroneCAN relay node
 * (hal/can_relay.c: node 30 = DC_NODE_RELAY, hardpoints 0-5 = relays 1-6 at 50 Hz, WIRING.md 3E). Both behind
 * the same hal_relay_* API (app/relays.c keeps every rule). The default stays MODBUS until the owner has chosen
 * the board (DroneCAN only if it meets R-1..R-3, docs/dronecan_design.md 9) - TO CONFIRM C17. DRONECAN needs
 * HAL_CAN_FITTED 1 (else: no relay board). HAL_RELAY_FITTED 0 = no relay board on either bus. */
#define RELAY_BUS_MODBUS    1
#define RELAY_BUS_DRONECAN  2
#ifndef RELAY_BUS
#define RELAY_BUS           RELAY_BUS_MODBUS            /* TO CONFIRM C17 (the owner decides at the 11f review) */
#endif
#define CAN_RELAY_CHANNELS  6u                          /* TO CONFIRM C17: hardpoints 0-5 (ArduPilot RELAY1..6) */

/* DroneCAN bus (WIRING.md 3E, CLAUDE.md T, docs/dronecan_design.md). Driver: hal/tc_can.c (step 11d, MCMCAN
 * CAN0 node 0) + hal/dronecan.c; with HAL_CAN_FITTED 0 hal_init never starts it (the code is compiled and its
 * iLLD files are built: tests/acceptance/check_ads_ifx.py). CONFIRMED by the iLLD pin map: P20.8 can be CAN0 node 0 TXD,
 * P20.7 can be CAN0 node 0 RXD (input B). NOT CONFIRMED (the kit user manual is not in the repo): that
 * the KIT_A2G_TC375_LITE V2 wires them to its TLE9251VSJ transceiver, that P20.6 drives the transceiver's
 * STB pin and its polarity (TLE9251V: high = stand-by, low = normal - check datasheet + kit schematic),
 * the CAN connector pinout and whether a 120 Ohm termination is on the kit. HAL_CAN_FITTED stays 0 until
 * the owner confirms all of it. Bus: 1 Mbit/s (docs/ardupilot.param CAN_P1_BITRATE), 120 Ohm at both ends.
 * Static node IDs TO CONFIRM (proposal: AURIX 10 = ArduPilot's CAN_D1_UC_NODE, throttle 20, steering 21,
 * gear 22, relay board 30; GPS + compass 124 not used): set on the nodes with the DroneCAN GUI Tool. */
#ifndef HAL_CAN_FITTED                                  /* (-D only for the gate's compile of the board path) */
#define HAL_CAN_FITTED      0                           /* TO CONFIRM: 1 only after the pins / termination are checked */
#endif
#define CAN_TXD             IfxCan_TXD00_P20_8_OUT      /* TO CONFIRM on the kit schematic */
#define CAN_RXD             IfxCan_RXD00B_P20_7_IN      /* TO CONFIRM on the kit schematic */
#define PIN_CAN_STB         &MODULE_P20, 6u             /* TO CONFIRM: TLE9251V STB (GPIO), polarity TO CONFIRM */
#define CAN_STB_NORMAL      0u                          /* TO CONFIRM: STB level for normal operation (TLE9251V: low) */
#define CAN_BITRATE         1000000u
#define CAN_SAMPLE_POINT    8750u                       /* 87.5 % (DroneCAN / CiA recommendation at 1 Mbit/s) */
/* Step 11e: throttle, steering and gear as DroneCAN servo nodes (hal/can_act.c, CPU1 safe frames in the reserved
 * TX buffers). Only with HAL_CAN_FITTED; 0 = the PWM servos + the steering motor of the old hardware (FIRST TEST,
 * the board today). TO CONFIRM by the owner (bench: FIRST_TEST.md 3j) before it becomes 1. */
#ifndef HAL_CAN_SERVOS                                  /* (-D only for the gate's compile of the board path) */
#define HAL_CAN_SERVOS      0
#endif
/* The UNCREWED dead-man pulse (OA-3, docs/dronecan_design.md 14.1 option 1 = the recommended default, WAITING FOR
 * OWNER W5): with the DroneCAN servos P10.5 no longer drives the safety relay (OD-2: the servo power is never
 * cut) - CPU1 toggles it for the retriggerable timer relay in the kill circuit. Pin / relay / time TO CONFIRM. */
#define PIN_DEADMAN         PIN_RELAY                   /* P10.5 (D10) */

/* --- interrupt priorities (unique per CPU; 1 = lowest) ----------------------------- */
#define PRIO_LINK_TX   10
#define PRIO_LINK_RX   11
#define PRIO_LINK_ER   12
#define PRIO_GW_TX     13
#define PRIO_GW_RX     14
#define PRIO_GW_ER     15
#define PRIO_GPS_TX    16
#define PRIO_GPS_RX    17
#define PRIO_GPS_ER    18
#define PRIO_BR_TX     19
#define PRIO_BR_RX     20
#define PRIO_BR_ER     21
#define PRIO_RL_TX     22                       /* RS485 relay board (ASCLIN4) */
#define PRIO_RL_RX     23
#define PRIO_RL_ER     24
#define PRIO_BNO_TX    30                       /* QSPI1, CPU0 (the control loop reads the IMU) */
#define PRIO_BNO_RX    31
#define PRIO_BNO_ER    32
#define PRIO_SD_TX     40                       /* QSPI2, CPU2 (the logger writes the card) */
#define PRIO_SD_RX     41
#define PRIO_SD_ER     42

#endif /* HAL_PINS_H */
