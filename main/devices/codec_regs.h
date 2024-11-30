#ifndef CODEC_REGS_H
#define CODEC_REGS_H

enum codec_control_port {
    CONTROL_4 = 0x04,
    SERIAL_AUDIO_FORMAT = 0x05,
    RECEIVER_ERROR_MASK = 0x06,
    AUDIO_FORMAT_DETECT = 0x0B,
    RECEIVER_ERROR = 0x0C,
};

enum codec_source {
    SOURCE_RXP0 = 0x81,
    SOURCE_RXP1 = 0x89,
};

#endif //CODEC_REGS_H
