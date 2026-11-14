#include <cstdio>
#include <cmath>
#include "hardware/adc.h"
#include "hardware/irq.h"
#include "pico/stdlib.h"
#include "config.h"
#include "globals.h"
#include "adc.h"

#include <FreeRTOS.h>
#include <queue.h>
#include <task.h>

void adcIRQHandler() { 
  BaseType_t wake = pdFALSE;

  if (!waveEmptyQueue || !waveFullQueue) {
    adc_fifo_drain();
    irq_clear(ADC_IRQ_FIFO);
    return;
  } 

  #if ADC_TEST_MODE == 0
    while (adc_fifo_get_level() >= 2) { 
      uint16_t v0 = adc_fifo_get(); 
      uint16_t i0 = adc_fifo_get(); 
      int16_t v = (int16_t)v0; 
      int16_t i = (int16_t)i0; 

      currentId++; 
      if (!buffer) { 
        if (xQueueReceiveFromISR(waveEmptyQueue, &buffer, &wake) != pdFALSE) { 
          currentBufferIndex = 0; 
          buffer->id = currentId; 
        } 
      } if (buffer) { 
        if (currentBufferIndex < SAMPLE_AMOUNT) { 
          buffer->voltage[currentBufferIndex] = v; 
          buffer->current[currentBufferIndex] = i; 
          currentBufferIndex++; 
        } 
        if (currentBufferIndex >= SAMPLE_AMOUNT) { 
          if (xQueueSendFromISR(waveFullQueue, &buffer, &wake)) buffer = 0; 
        } 
      } 
    } 
  #else 
    while (adc_fifo_get_level() >= 1) { 
      uint16_t s = adc_fifo_get();
      int16_t v = (ADC_TEST_MODE==1) ? (int16_t)s : 0;
      int16_t i = (ADC_TEST_MODE==2) ? (int16_t)s : 0;
      currentId++; 
      if (!buffer) { 
        if (xQueueReceiveFromISR(waveEmptyQueue, &buffer, &wake) != pdFALSE) { 
          currentBufferIndex = 0; 
          buffer->id = currentId; 
        } 
      } if (buffer) { 
        if (currentBufferIndex < SAMPLE_AMOUNT) { 
          buffer->voltage[currentBufferIndex] = v; 
          buffer->current[currentBufferIndex] = i; 
          currentBufferIndex++; 
        } 
        if (currentBufferIndex >= SAMPLE_AMOUNT) { 
          if (xQueueSendFromISR(waveFullQueue, &buffer, &wake)) buffer = 0; 
        } 
      } 
    }  
  #endif
  irq_clear(ADC_IRQ_FIFO); 
  portYIELD_FROM_ISR(wake); 
}

void setupADC() {
  printf("Setuping ADC... \n");
  adc_init();

  adc_gpio_init(27); 
  adc_gpio_init(28);

  #if ADC_TEST_MODE == 0
    adc_select_input(1);
    adc_set_round_robin((1u << 1) | (1u << 2));     
    adc_fifo_setup(true, false, 2, true, false);
  #else
    adc_set_round_robin(0);                   
    adc_select_input((ADC_TEST_MODE==1) ? 1 : 2); 
    adc_fifo_setup(true, false, 1, true, false);
  #endif
  
  float adcClock = 48000000.0;
  float clkdiv = adcClock / SAMPLE_RATE;
  adc_set_clkdiv(clkdiv);

  irq_set_exclusive_handler(ADC_IRQ_FIFO, adcIRQHandler);
  adc_fifo_drain();
  irq_clear(ADC_IRQ_FIFO);          
  
  adc_irq_set_enabled(true); 
  irq_set_enabled(ADC_IRQ_FIFO, true); 
                                    
  printf("Finished ADC setup...\n\n");
}