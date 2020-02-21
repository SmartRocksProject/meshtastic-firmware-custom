
#include "GPS.h"
#include "time.h"
#include <sys/time.h>

// stuff that really should be in in the instance instead...
HardwareSerial _serial_gps(GPS_SERIAL_NUM);
uint32_t timeStartMsec;  // Once we have a GPS lock, this is where we hold the initial msec clock that corresponds to that time
uint64_t zeroOffsetSecs; // GPS based time in secs since 1970 - only updated once on initial lock

RTC_DATA_ATTR bool timeSetFromGPS;     // We only reset our time once per _boot_ after that point just run from the internal clock (even across sleeps)

GPS gps;

GPS::GPS() : PeriodicTask(30 * 1000)
{
}

void GPS::setup()
{
    readFromRTC();

#ifdef GPS_RX_PIN
    _serial_gps.begin(GPS_BAUDRATE, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
#endif
}

void GPS::readFromRTC()
{
    struct timeval tv; /* btw settimeofday() is helpfull here too*/

    if (!gettimeofday(&tv, NULL))
    {
        uint32_t now = millis();

        DEBUG_MSG("Read RTC time as %ld (cur millis %u) valid=%d\n", tv.tv_sec, now, timeSetFromGPS);
        timeStartMsec = now;
        zeroOffsetSecs = tv.tv_sec;
    }
}

/// If we haven't yet set our RTC this boot, set it from a GPS derived time
void GPS::perhapsSetRTC(const struct timeval *tv)
{
    if (!timeSetFromGPS)
    {
        timeSetFromGPS = true;
        DEBUG_MSG("Setting RTC %ld secs\n", tv->tv_sec);
        settimeofday(tv, NULL);
        readFromRTC();
    }
}

#include <time.h>

// for the time being we need to rapidly read from the serial port to prevent overruns
void GPS::loop()
{
    PeriodicTask::loop();

#ifdef GPS_RX_PIN
    while (_serial_gps.available())
    {
        encode(_serial_gps.read());
    }

    if (!timeSetFromGPS && time.isValid() && date.isValid())
    {
        struct timeval tv;

        /* Convert to unix time 
        The Unix epoch (or Unix time or POSIX time or Unix timestamp) is the number of seconds that have elapsed since January 1, 1970 (midnight UTC/GMT), not counting leap seconds (in ISO 8601: 1970-01-01T00:00:00Z). 
        */
        struct tm t;
        t.tm_sec = time.second();
        t.tm_min = time.minute();
        t.tm_hour = time.hour();
        t.tm_mday = date.day();
        t.tm_mon = date.month() - 1;
        t.tm_year = date.year() - 1900;
        t.tm_isdst = false;
        time_t res = mktime(&t);
        tv.tv_sec = res;
        tv.tv_usec = 0; // time.centisecond() * (10 / 1000);

        perhapsSetRTC(&tv);
    }
#endif
}

uint32_t GPS::getTime()
{
    return ((millis() - timeStartMsec) / 1000) + zeroOffsetSecs;
}

uint32_t GPS::getValidTime()
{
    return timeSetFromGPS ? getTime() : 0;
}

void GPS::doTask()
{
    if (location.isValid() && location.isUpdated())
    { // we only notify if position has changed
        // DEBUG_MSG("new gps pos\n");
        notifyObservers();
    }
}

String GPS::getTimeStr()
{
    static char t[12]; // used to sprintf for Serial output

    snprintf(t, sizeof(t), "%02d:%02d:%02d", time.hour(), time.minute(), time.second());
    return t;
}