/* MHG_ADC_DMA
 *
 * uses adc fifo and dma to capture values at ~500 kHz and average them,
 * downsampling to 0.5 kHz
 *
 * usage:
 *  MHG_ACD_DMA *ad = MHG_ACD_DMA.getADC_DMA();
 *
 *  int chan = 0;
 *  float readingValue;
 *  absolute_time_t us;
 *  ad->startRecording(chan)
 *  loop:
 *   bool newReading = ad->getReading(value,time);
 *   if (newReading) {
 *   	//do something
 *   }
 *
 *   ad->stopRecording();
 *   //optional, if need to free up dma
 *   ad->releaseDMA();
 *
 *
 * */


#ifndef _MHG_ADC_DMA__
#define _MHG_ADC_DMA__


#include <stdint.h>
#include "pico/stdlib.h"
#include "pico/util/queue.h"
#include "hardware/adc.h"
#include "hardware/dma.h"
#include "hardware/irq.h"


class MHG_ADC_DMA
{
	static const int DMACNT = 1000;
	static constexpr float scaling = 3.3 / (4096.0 * (float) DMACNT);

private:
	MHG_ADC_DMA() = default;
	int dma_chan = -1;
	uint16_t bufferA[DMACNT];
	uint16_t bufferB[DMACNT];
	bool readingA = true;
	bool newReading = false;
	bool doneReading = false;
	uint64_t readingTime = 0;
	float readingAverage = 0;
	float zeroVoltage = 1.65;
public:

	static void dma_handler() {


		// Clear the interrupt request.
		dma_hw->ints0 = 1u << getInstance()->dma_chan;
		//
		getInstance()->readingA = !getInstance()->readingA;
		getInstance()->newReading = true;
		getInstance()->readingTime = time_us_64();

		// re-trigger
		if (!getInstance()->doneReading) {
			dma_channel_set_write_addr(getInstance()->dma_chan, getInstance()->readingA ? getInstance()->bufferA : getInstance()->bufferB, true);
		}
	}
	static MHG_ADC_DMA *getInstance (void) {
		static MHG_ADC_DMA instance;
		return &instance;
	}

	float getZeroVoltage() {
		return zeroVoltage;
	}
	void setZeroVoltage(float zv) {
		zeroVoltage = zv;
	}
	void adjustZeroVoltage(float increment) {
		zeroVoltage += increment;
	}

	bool getReading(float &value, uint64_t &time_stamp, bool subtractZero = true) {
		uint16_t *b = readingA ? bufferB : bufferA; //return average of buffer not being filled
		if (newReading){
			float value = 0.0;

			for (int j = 0; j < DMACNT; ++j) {
				value += (b[j] & 0xFFF); //lowest 12 bits
			}
			readingAverage = value*scaling;
		}
		time_stamp = readingTime;
		value = readingAverage - (subtractZero? zeroVoltage : 0);
		bool rv = newReading;
		newReading = false;
		return rv;
	}

	//adapted from pico adc and dma examples

	  void stopReading() {
		  doneReading = true;

		   if (dma_chan > 0) {
			   // disable the channel on IRQ0
			   dma_channel_set_irq0_enabled(dma_chan, false);
			   // do we need irq_set_enabled(DMA_IRQ_0, false); ??
			   // abort the channel
			   dma_channel_abort(dma_chan);
			   // clear the spurious IRQ (if there was one)
			   dma_channel_acknowledge_irq0(dma_chan);
		   }

		   adc_run(false);

		   newReading = false;

	  }

	   void startRecording(uint capture_channel) {
		   stopReading();
		// Init GPIO for analogue use: hi-Z, no pulls, disable digital input buffer.
		   if (capture_channel > 25) {
			   capture_channel -= 26; //if given pin number, subtract off 26
		   }

			adc_select_input(capture_channel);
			adc_fifo_setup(
				true,    // Write each completed conversion to the sample FIFO
				true,    // Enable DMA data request (DREQ)
				1,       // DREQ (and IRQ) asserted when at least 1 sample present
				false,   // don't use ERR bit
				false     // don't shift each sample to 8 bits when pushing to FIFO
			);

			// Divisor of 0 -> full speed.
			adc_set_clkdiv(0);

			// Set up the DMA to start transferring data as soon as it appears in FIFO
			while (dma_chan < 0) {
				dma_chan = dma_claim_unused_channel(true);
			}

			dma_channel_config cfg = dma_channel_get_default_config(dma_chan);

			// Reading from constant address, writing to incrementing short addresses
			channel_config_set_transfer_data_size(&cfg, DMA_SIZE_16);
			channel_config_set_read_increment(&cfg, false);
			channel_config_set_write_increment(&cfg, true);

			// Pace transfers based on availability of ADC samples
			channel_config_set_dreq(&cfg, DREQ_ADC);

			readingA = true;
			newReading = false;
			doneReading = false;

			dma_channel_configure(dma_chan, &cfg,
				bufferA,    // dst
				&adc_hw->fifo,  // src
				DMACNT,  // transfer count
				false            // don't start yet
			);

			// Tell the DMA to raise IRQ line 0 when the channel finishes a block
			dma_channel_set_irq0_enabled(dma_chan, true);

			// Configure the processor to run dma_handler() when DMA IRQ 0 is asserted
			irq_set_exclusive_handler(DMA_IRQ_0, dma_handler);
			irq_set_enabled(DMA_IRQ_0, true);


			dma_channel_start(dma_chan);
		    adc_run(true);
	   }

	   void releaseDMA() {
		   stopReading();
		   dma_channel_unclaim(dma_chan);
		   dma_chan = -1;

	   }


};

#endif
