/*
  ADCDAC 
  (C) Copyright 2023-2024 ELD/LION, Leiden University

  Written by       : Harry Visser & Arno van Amersfoort
  Target compiler  : Arduino IDE
  Target platform  : Teensy 4.1
  Dependencies     : spi.h
  Initial date     : June 20, 2024
  Last modified    : June 28, 2024
*/

#include <Arduino.h>
#include <IntervalTimer.h>
#include "adc_dac_io.h"

EventResponder g_SPI0Event;
EventResponder g_SPI1Event;
ADCDAC adc_dac(g_SPI0Event, g_SPI1Event);


int32_t g_adc_results[ADC_COUNT];
int32_t g_dac_results[DAC_COUNT];
IntervalTimer sample_timer;

void trigger_adcs()
{
  adc_dac.start_conversion();
}



void SPI1EventResponder(EventResponderRef event_responder) 
{
  adc_dac.parse_adc_data(g_dac_results);
  adc_dac.check_for_adc_overload (g_dac_results);
}

void SPI0EventResponder(EventResponderRef event_responder) 
{
  digitalWriteFast(_DAC_SYNC, HIGH);
}


void setup()
{
  adc_dac.init();
  sample_timer.begin(trigger_adcs, SAMPLE_INTERVAL);
  g_SPI0Event.attachImmediate(&SPI0EventResponder);
  g_SPI1Event.attachImmediate(&SPI1EventResponder);

  delay(10);
}


void loop()
{

    while (!adc_dac.adc_busy());

    adc_dac.get_adcs();
    adc_dac.put_dacs(g_dac_results);


    //enter your code here
    for (uint8_t i = 0; i < ADC_COUNT; i++ )
    {
       g_dac_results[i] = g_adc_results[i];

    // end of code
    }


}
