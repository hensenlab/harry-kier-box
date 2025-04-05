#include "adc_dac_io.h"

// Constructor:
ADCDAC::ADCDAC(EventResponder& SPI0_Event, EventResponder& SPI1_Event)
{
  m_pSPI1Event = &SPI1_Event;
  m_pSPI0Event = &SPI0_Event;
}

/*
// Destructor:
ADCDAC::~ADCDAC()
*/

void ADCDAC::init()
{
  // initialize digital pin LED as an output.
  pinMode(_ADC_CNV, OUTPUT);
  pinMode(_ADC_BUSY, INPUT);
  pinMode(_DAC_RESET, OUTPUT);
  pinMode(_DAC_SYNC, OUTPUT);
  pinMode(_DAC_LDAC, OUTPUT);
  pinMode(_OVERLOAD_LED1, OUTPUT);
  pinMode(_OVERLOAD_LED2, OUTPUT);
  pinMode(_OVERLOAD_LED3, OUTPUT);
  SPI1.begin();
  SPI1.beginTransaction(SPISettings(SPI_ADC_SPEED, MSBFIRST, SPI_MODE0));    // perform ADC transfer in SPI mode 0

  dac_init();
  
}


void ADCDAC::dac_init()
{
  SPI.begin();
  digitalWrite(_ADC_CNV, LOW);
  digitalWrite(_DAC_SYNC, HIGH);
  digitalWrite(_DAC_LDAC, HIGH);
  digitalWrite(_DAC_RESET, LOW);
  delay(100);
  digitalWrite(_DAC_RESET, HIGH);
  delay(1);

  uint32_t dacConfigWord = 0;

  dacConfigWord |= Read << 31;
  dacConfigWord |= Address << 24;
  dacConfigWord |= EN_TMP_CAL << 23;
  dacConfigWord |= TNH_MASK << 18;
  dacConfigWord |= LDACMODE << 14;
  dacConfigWord |= FSDO << 13;
  dacConfigWord |= ENALMP << 12;
  dacConfigWord |= DSDO << 11;
  dacConfigWord |= FSET << 10;
  dacConfigWord |= VREFVAL << 6;
  dacConfigWord |= PDN << 4;

  // Perform DAC transfer in SPI mode 1
  SPI.beginTransaction(SPISettings(SPI_INITIAL_DAC_SPEED, MSBFIRST, SPI_MODE1));

  digitalWrite(_DAC_SYNC, LOW);

  for (uint8_t i = 0; i < DAC_COUNT; i++)
  {
    SPI.transfer((uint8_t)  (dacConfigWord >> 24));
    SPI.transfer((uint8_t) ((dacConfigWord >> 16) & 0xFF));
    SPI.transfer((uint8_t) ((dacConfigWord >> 8) & 0xFF));
    SPI.transfer((uint8_t)  (dacConfigWord & 0xFF));
  }

  digitalWrite(_DAC_SYNC, HIGH);
  SPI.endTransaction();

  memset(m_adc_spi_data_dummy, 0, ADC_SPI_BLOCK_SIZE);
  memset(m_dac_spi_data_dummy, 0, DAC_SPI_BLOCK_SIZE);
  SPI.beginTransaction(SPISettings(SPI_DAC_SPEED, MSBFIRST, SPI_MODE1));      // perform DAC transfer in SPI mode 1

  delay(10);
}



//-----------------------------------------------
//
//    Collects data from ADCs
//    Writes SPI-data to DAC's
//
//-----------------------------------------------

void ADCDAC::get_adcs()
{
  spi_read_adc_data();
}

//-----------------------------------------------
//
//    Prepare DAC-SPI-buffer
//
//-----------------------------------------------

void ADCDAC::put_dacs(int32_t* i_dac_results)
{
  calculate_dac_data(m_dac_spi_data, i_dac_results);
  spi_write_dac_data(m_dac_spi_data);
}

//------------------------------------------------
//
//    Calculates SPI-buffer based on dac_results (32 bit, signed)
//
//-----------------------------------------------

void ADCDAC::calculate_dac_data(uint8_t* dac_spi_data, int32_t* i_dac_results)
{
  for (uint8_t dacidx = 0; dacidx < DAC_COUNT; dacidx++)
  {
    uint32_t unsignedResult = (UNSIGNED_OFFSET + i_dac_results[dacidx]); // Convert to unsigned using offset
    uint8_t dac_byte_idx = DAC_REMAP_TABLE[dacidx] * DAC_BYTES_PER_SAMPLE;

    dac_spi_data[dac_byte_idx + 0] = 0x01; // Controlbyte 0x01 (write dac)
    dac_spi_data[dac_byte_idx + 1] = (unsignedResult & 0x00FF0000) >> 16; // MSB
    dac_spi_data[dac_byte_idx + 2] = (unsignedResult & 0x0000FF00) >> 8;
    dac_spi_data[dac_byte_idx + 3] = (unsignedResult & 0x000000FF) >> 0; // LSB
  }
}


//------------------------------------------------
//
//    Parses SPI buffer to 32 bit values (signed)
//
//-----------------------------------------------

void ADCDAC::parse_adc_data(int32_t* i_adc_results)
{
  for (uint8_t adccnt = 0; adccnt < ADC_COUNT; adccnt++)
  {
    uint8_t adcidx = ADC_REMAP_TABLE[adccnt];
  
    // NOTE: endian ordering is big endian
    if (m_adc_spi_data[(adcidx * ADC_BYTES_PER_SAMPLE) + 0] & 0x80) //if sign bit
    {
      i_adc_results[adccnt] = 0xFF000000;
    }
    else
    {
      i_adc_results[adccnt] = 0x00000000;
    }
    i_adc_results[adccnt] |= (int32_t)m_adc_spi_data[(adcidx * ADC_BYTES_PER_SAMPLE) + 0] << 16;
    i_adc_results[adccnt] |= (int32_t)m_adc_spi_data[(adcidx * ADC_BYTES_PER_SAMPLE) + 1] << 8;
    i_adc_results[adccnt] |= (int32_t)m_adc_spi_data[(adcidx * ADC_BYTES_PER_SAMPLE) + 2];
  }
}

//------------------------------------------------
//
//    TTransfers dac-SPI-buffer to DACs
//
//-----------------------------------------------


void ADCDAC::spi_write_dac_data(uint8_t* dac_spi_data)
{
  // Perform DAC transfer in SPI mode 1
  //
  digitalWriteFast(_DAC_SYNC, LOW);
  SPI.transfer(dac_spi_data, m_dac_spi_data_dummy, DAC_SPI_BLOCK_SIZE, *m_pSPI0Event);  
}

//------------------------------------------------
//
//    Collects SPI-data from ADCs
//
//-----------------------------------------------

void ADCDAC::spi_read_adc_data()
{
  // Perform ADC transfer in SPI mode 0
  SPI1.transfer(m_adc_spi_data_dummy, m_adc_spi_data, ADC_SPI_BLOCK_SIZE, *m_pSPI1Event);

}




void ADCDAC::check_for_adc_overload(int32_t* i_adc_results)
{
  for (uint8_t i = 0; i < ADC_COUNT; i++)
  {
    if (abs(i_adc_results[i]) >= OVERLOAD_VALUE) 
    {
      ol_led_cnt[i] = OL_LED_COUNT;
    }
    if (ol_led_cnt[i])
    {
      ol_led_cnt[i]--;
      digitalWrite(ol_led_pin[i], HIGH);
    }
    else
    {
      digitalWrite(ol_led_pin[i], LOW);
    }
  }
}
