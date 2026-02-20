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

union packed_int32 {
  int32_t i32;
  byte b[4];
};

int32_t micros_buffer[USB_IO_BUFFER_SIZE];
int32_t adc_buffer[ADC_COUNT][USB_IO_BUFFER_SIZE];

// int32_t g_dac_data[DAC_COUNT];
IntervalTimer sample_timer;

int32_t cur_i = 0;
uint8_t buf_i = 0;

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
  Serial.begin(1000000) ;     // De serieele snelheid in Bit/sec
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

    while (!adc_dac.adc_busy())
    {
        // Serial.send_now();
    }

    
    adc_dac.get_adcs();

    micros_buffer[buf_i] = micros();
    for (uint8_t i = 0; i < ADC_COUNT; i++ )
    {
        adc_buffer[i][buf_i]=g_adc_results[i];
    }

    buf_i++;

    int packets_avialable = (int)floor(Serial.availableForWrite()/USB_PACKET_BYTE_SIZE);

    
    if (packets_avialable)
    {
        packed_int32 temp;
        uint8_t iii = 0;
        for (uint8_t ii = 0; (ii < packets_avialable) && (ii<buf_i); ii++)
        {
            temp.i32 = micros_buffer[ii];
            Serial.write(temp.b,4);
               
            for (uint8_t i = 0; i < ADC_COUNT; i++ )
            {
                temp.i32=adc_buffer[i][ii];
                Serial.write(temp.b,4);
            }
            Serial.write("\xff\xff\xff\n");
            iii++;
        }
        memmove(micros_buffer, micros_buffer + iii, (USB_IO_BUFFER_SIZE - iii) * sizeof(int32_t));
        for (uint8_t i = 0; i < ADC_COUNT; i++ )
            {
                memmove(adc_buffer[i], adc_buffer[i] + iii, (USB_IO_BUFFER_SIZE - iii) * sizeof(int32_t));
            }
        
        buf_i-=iii; 
    }
    cur_i++;
    if (buf_i==USB_IO_BUFFER_SIZE)
    {
        buf_i=0;
        Serial.write("ERROR\xff\xff\xff\n");
    }


}
