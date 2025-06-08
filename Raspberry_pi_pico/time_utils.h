#ifndef TIME_UTILS_H
#define TIME_UTILS_H

#include <time.h>

// Simple implementation of timegm() for systems that lack it
static inline time_t timegm(struct tm *tm) {
    static const int days_before_month[] = {
        0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334
    };

    int year = tm->tm_year + 1900;
    int month = tm->tm_mon;
    if (month < 0 || month > 11) return (time_t)-1;

    // Calculate days since 1970-01-01
    int days = (year - 1970) * 365 + days_before_month[month] + (tm->tm_mday - 1);
    
    // Add leap year days
    days += (year - 1969) / 4;  // Leap years since 1970
    days -= (year - 1901) / 100;  // Subtract century years
    days += (year - 1601) / 400;  // Add back every 4th century
    
    // Add leap day for current year if needed
    if (month > 1 && ((year % 4 == 0 && year % 100 != 0) || year % 400 == 0)) {
        days++;
    }

    return ((time_t)days * 86400 + 
            (time_t)tm->tm_hour * 3600 +
            (time_t)tm->tm_min * 60 + 
            (time_t)tm->tm_sec);
}

#endif // TIME_UTILS_H
