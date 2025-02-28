#ifndef BAUDRATE_H
#define BAUDRATE_H

enum BaudRate {
    BAUD_300 = 0,
    BAUD_600,
    BAUD_1200,
    BAUD_2400,
    BAUD_4800,
    BAUD_9600,
    BAUD_14400,
    BAUD_19200,
    BAUD_28800,
    BAUD_38400,
    BAUD_57600,
    BAUD_115200
};

const int baudRates[] = {
    300, 600, 1200, 2400, 4800, 9600, 14400, 19200, 28800, 38400, 57600, 115200
};

#endif // BAUDRATE_H