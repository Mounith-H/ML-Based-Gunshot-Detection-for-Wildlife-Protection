#ifndef GPS_HANDLER_H
#define GPS_HANDLER_H

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "minmea.h"

#define GPS_BAUD_RATE 9600

class GPSHandler {
public:
    GPSHandler(uart_inst_t* uart, uint tx_pin, uint rx_pin);
    bool init(void);
    bool update(void);
    bool has_fix(void);
    double get_latitude(void);
    double get_longitude(void);
    char* get_time(void);
    char* get_date(void);

private:
    uart_inst_t* _uart;
    uint _tx_pin;
    uint _rx_pin;
    char _buffer[256];
    uint _buffer_pos;
    bool _has_fix;
    struct minmea_float _latitude;
    struct minmea_float _longitude;
    char _time[9];    // HH:MM:SS\0
    char _date[11];   // DD/MM/YYYY\0
    struct minmea_sentence_rmc _rmc;
};

#endif // GPS_HANDLER_H
