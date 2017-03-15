#include "filterUtils.h"

/*
 * a simple noise variance filter that smoothes out spikes by linear extrapolation.
 */
void varianceFilter32b(uint32_t *start, uint32_t *end, float tolerance) {
	uint32_t *currentPos = start;
	currentPos++;
	while(currentPos != end) {
		uint32_t *previousPos = currentPos-1;
		uint32_t previousValue = *previousPos;
		uint32_t currentValue = *currentPos;
		uint32_t *nextPos = currentPos+1;
		uint32_t nextValue = *nextPos;

		//checks if the current value is within tolerated deviation percentages
		if((previousValue*tolerance > currentValue && currentValue < previousValue*(1-tolerance)) ||
		(nextValue*tolerance > currentValue && currentValue < nextValue*(1-tolerance))) {
			//do nothing. continue to next value
		}
		else {
			//value deviates too much, replace value at currentPos by linear extrapolation
			*currentPos = (previousValue + nextValue) / 2;
		}
		currentPos++;
	}
}

float filterNoise(float previous, CircularBUFFER *buff, int tolerance) {
	int values[5];
	int count = 0;

	for(int i = 0; i < buff->size; i++) {
		values[i] = pullBuffer(buff, 0-i);
	}
	int bufferSize = buff->size;
	int sum = 0;
	for(int i = 0; i < bufferSize; i++) {
		if((previous+tolerance) > values[i] && (previous-tolerance) < values[i]) {
			sum += values[i];
		}
		else {
			count++;
			if(count > 2) {
				//more than 2 of 5 values out of scope ("spike" seems unlikely).
				sum += values[i];
			}
			else {
				sum += previous; //filtered away unrealistic spike
			}
		}
	}
	//below smoothes out changes (dampens change)
	int average = sum/bufferSize;
	int maxDev = 50; //TODO: parameterize this.
	if(previous+maxDev < average) {
		average = previous+maxDev;
	}
	else if(previous-maxDev > average) {
		average = previous-maxDev;
	}
	return average;
}
