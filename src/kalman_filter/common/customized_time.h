#ifndef COMMON_CUSTOMIZED_TIME_H_
#define COMMON_CUSTOMIZED_TIME_H_

#include <chrono>
#include <ostream>
#include <ratio>

#include "port.h"

namespace common{

constexpr int64 kUtsEpochOffsetFromUnixEpochInSeconds = (719162ll * 24ll * 60ll * 60ll);

struct UniversalTimeScaleClock {
  using rep = int64;
  using period = std::ratio<1, 10000000>;
  using duration = std::chrono::duration<rep, period>;
  using time_point = std::chrono::time_point<UniversalTimeScaleClock>;
  static constexpr bool is_steady = true;
};

using Duration = UniversalTimeScaleClock::duration;
using Time = UniversalTimeScaleClock::time_point;


double ToSeconds(const Duration duration);

Time FromUniversal(int64 ticks);

int64 ToUniversal(const Time time);

std::ostream& operator<<(std::ostream& os, Time time);

}

#endif 
