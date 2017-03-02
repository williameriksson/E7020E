#include "timerUtils.h"

int getTimeDiff16b(int startTime, int endTime) {
	if (endTime < startTime) {
		return (0xFFFF) - startTime + endTime;
	}
	return endTime - startTime;
}
