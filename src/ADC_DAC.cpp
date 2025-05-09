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
int32_t g_dac_data[DAC_COUNT];
IntervalTimer sample_timer;

long unsigned int g_lastms = 0;

void trigger_adcs()
{
  adc_dac.start_conversion();
}

void SPI1EventResponder(EventResponderRef event_responder) 
{
  adc_dac.parse_adc_data(g_adc_results);
  adc_dac.check_for_adc_overload(g_adc_results);
}

void SPI0EventResponder(EventResponderRef event_responder) 
{
  digitalWriteFast(_DAC_SYNC, HIGH);
}


void setup()
{
  Serial.begin(140000) ;     // De serieele snelheid in Bit/sec
  delay(500) ;               // Er blijkt enige wachttijd noodzakelijk. Het waarom is onbekend bij mij.

  adc_dac.init();
  sample_timer.begin(trigger_adcs, SAMPLE_INTERVAL);
  g_SPI0Event.attachImmediate(&SPI0EventResponder);
  g_SPI1Event.attachImmediate(&SPI1EventResponder);

  delay(10);
}

float i2u(int32_t i) {
  return -float(i)/8388608*11.7/1.2 ;  // 1.2 schalingsfout door filter en ingangsversterker??
}

int32_t u2i(float u) {
  if (u>10.){u=10.;} else if(u<-10.){u=-10.;} // Overrun bescherming
  return int(u/11.7*8388608*1.2);   // Van float schalen en dan naar integer 
}

void loop()
{

    while (!adc_dac.adc_busy());

    adc_dac.get_adcs();

    float u[ADC_COUNT];

    for (uint8_t i = 0; i < ADC_COUNT; i++ )
    {
       u[i]=i2u(g_adc_results[i]);
       // do stuff with u[]
       g_dac_data[i]=u2i(u[i]);
    }
    adc_dac.put_dacs(g_dac_data);

    if (millis() > g_lastms+1000) { // send voltages every second to serial monitor
      char temp[20];
      for (uint8_t i = 0; i < ADC_COUNT; i++ ) {
        sprintf(temp,"u[%d]=%f\t",i, u[i]);
        Serial.write(temp);
      }
      Serial.write("\n");
      g_lastms = millis();
    }


}
