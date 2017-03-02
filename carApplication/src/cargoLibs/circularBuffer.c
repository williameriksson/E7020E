#include "circularBuffer.h";


void circularBufferInit(CircularBUFFER* buff, int i, int s) {
	buff->indexPointer = i;
	buff->size = s;
	buff->buffer = malloc(s * sizeof(int));
}

void pushBuffer (CircularBUFFER *buff, int value) {
	if (buff->indexPointer == buff->size - 1) {
		buff->indexPointer = 0;
	} else {
		buff->indexPointer += 1;
	}
	buff->buffer[buff->indexPointer] = value;
}

int getBufferAverage(CircularBUFFER *buff) {
	int tmp = 0;
	// What happens before the array has wrapped around?
	// Does the compiler guarantee the unassigned positions of the array
	// to be initialized to 0 by default, or can it be garbish?
	for (int i = 0; i < buff->size; i++) {
		tmp += buff->buffer[i];
	}
	return tmp / buff->size;
}


/*
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
*/
