#include "circularBuffer.h";

void circularBufferInit(CircularBUFFER *buffer) {
	buffer->current = 0;
}

void bufferAdd(CircularBUFFER *buffer, int input) {
	buffer->bufferList[buffer->current] = input;
	buffer->current++;
	if(buffer->current == BUFFERSIZE-1) {
		buffer->current = 0;
	}
}
//returns the average(mean) value of the buffer.
int getAverage(CircularBUFFER *buffer) {
	int sum = 0;
	for(int i = 0; i < sizeof(buffer->bufferList); i++) {
		sum += buffer->bufferList[i];
	}
	int average = sum/(sizeof(buffer->bufferList));
	return average;
}
