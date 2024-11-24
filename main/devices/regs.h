#ifndef REGS_H
#define REGS_H

enum control_port {
    GOTO_PG,
    RESET_CTRL,
    DEVICE_CTRL_1,
    DEVICE_CTRL_2,
    SIG_CH_CTRL = 0x28,
    FS_MON = 0x37,
    CLKDET_STATUS = 0x39,
    DIG_VOL_CTRL = 0x4C,
    DIG_VOL_CTRL_RIGHT = 0x4D,
    AUTO_MUTE_CTRL = 0x50,
    AUTO_MUTE_TIME = 0x51,
    DSP_MISC = 0x66,
    POWER_STATE = 0x68,
    CHAN_FAULT = 0x70,
    GLOBAL_FAULT_1 = 0x71,
    GLOBAL_FAULT_2 = 0x72,
    FAULT_CLEAR = 0x78,
    GOTO_BK = 0x7f,
    PROG_DELAY = 0xFF
};

#endif //REGS_H
