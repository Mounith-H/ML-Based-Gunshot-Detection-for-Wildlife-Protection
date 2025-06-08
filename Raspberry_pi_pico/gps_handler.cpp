#include "gps_handler.h"
#include <string.h>

GPSHandler::GPSHandler(uart_inst_t* uart, uint tx_pin, uint rx_pin)
    : _uart(uart), _tx_pin(tx_pin), _rx_pin(rx_pin), _buffer_pos(0), _has_fix(false) {
    _time[0] = '\0';
    _date[0] = '\0';
    _latitude.value = 0;
    _latitude.scale = 1;
    _longitude.value = 0;
    _longitude.scale = 1;
}

bool GPSHandler::init(void) {
    uart_init(_uart, GPS_BAUD_RATE);
    gpio_set_function(_tx_pin, GPIO_FUNC_UART);
    gpio_set_function(_rx_pin, GPIO_FUNC_UART);
    uart_set_hw_flow(_uart, false, false);
    uart_set_format(_uart, 8, 1, UART_PARITY_NONE);
    return true;
}

bool GPSHandler::update(void) {
    while (uart_is_readable(_uart)) {
        char c = uart_getc(_uart);
        
        if (_buffer_pos < sizeof(_buffer) - 1) {
            _buffer[_buffer_pos++] = c;
        }
        
        if (c == '\n') {
            _buffer[_buffer_pos] = '\0';
            
            // Parse with minmea for more robust handling
            switch (minmea_sentence_id(_buffer, false)) {
                case MINMEA_SENTENCE_RMC: {
                    if (minmea_parse_rmc(&_rmc, _buffer)) {
                        if (_rmc.valid) {
                            _has_fix = true;
                            _latitude = _rmc.latitude;
                            _longitude = _rmc.longitude;
                            
                            // Format time
                            snprintf(_time, sizeof(_time), "%02d:%02d:%02d",
                                   _rmc.time.hours, _rmc.time.minutes, _rmc.time.seconds);
                            
                            // Format date
                            snprintf(_date, sizeof(_date), "%02d/%02d/%04d",
                                   _rmc.date.day, _rmc.date.month, _rmc.date.year + 2000);
                        }
                    }
                    break;
                }
                
                case MINMEA_SENTENCE_GGA: {
                    struct minmea_sentence_gga frame;
                    if (minmea_parse_gga(&frame, _buffer)) {
                        if (frame.fix_quality > 0) {
                            _has_fix = true;
                            _latitude = frame.latitude;
                            _longitude = frame.longitude;
                        }
                    }
                    break;
                }
            }
            
            _buffer_pos = 0;
            return true;
        }
    }
    
    return false;
}

bool GPSHandler::has_fix(void) {
    return _has_fix;
}

double GPSHandler::get_latitude(void) {
    return minmea_tocoord(&_latitude);
}

double GPSHandler::get_longitude(void) {
    return minmea_tocoord(&_longitude);
}

char* GPSHandler::get_time(void) {
    return _time;
}

char* GPSHandler::get_date(void) {
    return _date;
}
