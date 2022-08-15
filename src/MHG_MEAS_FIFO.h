/* MHG_MEAS_FIFO
 *
 * transmits measurement data across multicore fifo
 * code adapted from https://github.com/earlephilhower/arduino-pico/blob/master/cores/rp2040/RP2040Support.h
 *
 * */
#ifndef _MHG_MEAS_FIFO_
#define _MHG_MEAS_FIFO_


#include <stdint.h>
#include "pico/util/queue.h"

typedef struct
{
	uint64_t meas_time;
	uint8_t meas_type;
	float data[3];
} MeasurementDataT;

class MHG_MEAS_FIFO
{

private:
	queue_t _queue[2];
	bool initialized = false;

public:

	//code adapted from https://github.com/earlephilhower/arduino-pico/blob/master/cores/rp2040/RP2040Support.h

	static const int FIFOCNT = 8;

	void begin() {
		queue_init(&_queue[0], sizeof(MeasurementDataT), FIFOCNT);
		queue_init(&_queue[1], sizeof(MeasurementDataT), FIFOCNT);
		initialized = true;
	}

	void push(MeasurementDataT val) {
		while (initialized && !push_nb(val)) { /* noop */ } //initialzed check better than infinite loop of stuckness
	 }

	bool push_nb(MeasurementDataT val) {
		// Push to the other FIFO
		if (!initialized) {
			return false;
		}
		return queue_try_add(&_queue[get_core_num() ^ 1], &val);
	}

	MeasurementDataT pop() {
		MeasurementDataT ret;
		while(initialized  && !pop_nb(&ret)) { /* noop */ } //initialized check - bad return value better than infinite loop of stuckness
		return ret;
	}

	bool pop_nb(MeasurementDataT *val) {
		// Pop from my own FIFO
		if (!initialized) {
			return false;
		}
		return queue_try_remove(&_queue[get_core_num()], val);
	}

	int availableToRead() {
		if (!initialized) {
			return 0;
		}

		return queue_get_level(&_queue[get_core_num()]);
	}

	int availableToWrite() {
		if (!initialized) {
			return 0;
		}
		return FIFOCNT-queue_get_level(&_queue[get_core_num()^1]);
	}

};

#endif
