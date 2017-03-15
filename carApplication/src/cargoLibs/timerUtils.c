#include "timerUtils.h"

int getTimeDiff16b(int startTime, int endTime) {
	if (endTime < startTime) {
		return (0xFFFF) - startTime + endTime;
	}
	return endTime - startTime;
}

uint32_t getTimeDiff32b(uint32_t startTime, uint32_t endTime) {
	if (endTime < startTime) {
		return ((0xFFFFFFFF) - startTime) + endTime;
	}
	return endTime - startTime;
}
