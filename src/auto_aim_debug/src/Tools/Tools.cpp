#include"Tools.h"

namespace rm{

    double get_now_time()
	{
		auto now = std::chrono::system_clock::now();
		auto durationSinceEpoch = now.time_since_epoch();
		auto seconds = std::chrono::duration_cast<std::chrono::seconds>(durationSinceEpoch);
		auto microseconds = std::chrono::duration_cast<std::chrono::microseconds>(durationSinceEpoch - seconds);
		double fractional_seconds = microseconds.count() * 1e-6;
		return seconds.count() + fractional_seconds;
	};
};