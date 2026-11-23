#pragma once

#include <cstdlib>
#include <ctime>

// Wall-clock time seen by the guest (CD-ROM RTC, memory card file dates).
// PS2X_DETERMINISTIC=1 freezes it to a fixed date, identical on every
// machine and time zone: netplay and replays need runs that do not depend on
// when or where they happen.
namespace ps2x
{
    inline bool guestClockDeterministic()
    {
        static const bool deterministic = []
        {
            const char *value = std::getenv("PS2X_DETERMINISTIC");
            return value && *value && *value != '0';
        }();
        return deterministic;
    }

    // Local calendar time for `hostTime`, or 2001-01-01 12:00:00 when deterministic.
    inline bool guestLocalTime(std::time_t hostTime, std::tm &out)
    {
        if (guestClockDeterministic())
        {
            out = std::tm{};
            out.tm_year = 101;
            out.tm_mon = 0;
            out.tm_mday = 1;
            out.tm_hour = 12;
            out.tm_wday = 1;
            return true;
        }
#ifdef _WIN32
        return localtime_s(&out, &hostTime) == 0;
#else
        return localtime_r(&hostTime, &out) != nullptr;
#endif
    }
}
