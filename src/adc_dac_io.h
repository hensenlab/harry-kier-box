#ifndef ADC_DAC_H
#define ADC_DAC_H
#include <Arduino.h>
#include <SPI.h>
#include <EventResponder.h>

#define _ADC_CNV    8
#define _ADC_BUSY   7

#define _DAC_SYNC   9
#define _DAC_LDAC   10
#define _DAC_RESET  24

#define _OVERLOAD_LED1   23
#define _OVERLOAD_LED2   6
#define _OVERLOAD_LED3   5

#define OVERLOAD_VALUE        8388608
#define UNSIGNED_OFFSET       8388608
#define DAC_COUNT             3 //4
#define ADC_COUNT             3
#define SPI_ADC_SPEED         30000000 // max. 100MHz
#define SPI_DAC_SPEED         15000000  //max. 50MHz, when FSDO enabled
#define SPI_INITIAL_DAC_SPEED 1000000
#define ADC_BYTES_PER_SAMPLE  5
#define DAC_BYTES_PER_SAMPLE  4
#define DAC_SPI_BLOCK_SIZE    (DAC_COUNT * DAC_BYTES_PER_SAMPLE)
#define ADC_SPI_BLOCK_SIZE    (ADC_COUNT * ADC_BYTES_PER_SAMPLE)
#define SAMPLE_INTERVAL       10 //us
#define OL_LED_INTERVAL       100 //ms
#define OL_LED_COUNT          1000 * OL_LED_INTERVAL / SAMPLE_INTERVAL

#define USB_PACKET_BYTE_SIZE  4+ADC_COUNT*4+4
#define USB_IO_BUFFER_SIZE    100

class ADCDAC
{
  public:
  
//    int32_t* adc_results;
//    int32_t* dac_results;
    
    void get_adcs();
    void put_dacs(int32_t* dac_results);

    bool adc_busy() { return digitalReadFast(_ADC_BUSY); };
    void init(void);
    void start_conversion() { digitalWriteFast(_ADC_CNV, HIGH); digitalWriteFast(_ADC_CNV, LOW); digitalWriteFast(_DAC_LDAC, LOW); };
    void parse_adc_data(int32_t* adc_results);
    void check_for_adc_overload(int32_t* i_adc_results);
    ADCDAC(EventResponder& SPI0_Event, EventResponder& SPI1_Event);

  private:
  
    //settings according to datasheet page 31
    uint32_t Read = 0;         //Write mode
    uint32_t Address = 0x02;   //Register Address
    uint32_t EN_TMP_CAL = 0;   //Temperature calibration mode disabled
    uint32_t TNH_MASK = 0;     //Track and hold masked for code jump > 2^14 (doesn't work since FSET = 1)
    uint32_t LDACMODE = 0;     //DAC output updated on SYNC rising edge
    uint32_t FSDO = 1;         //FAst SDO disabled (when enabled, SDO can handle 50MHz)
    uint32_t ENALMP = 0;       //No alarm on the ALARM pindac_buf
    uint32_t DSDO = 1;         //SDO enabled (for readback and daisy-chain)
    uint32_t FSET = 1;         //Enhanced THD
    uint32_t VREFVAL = 0b1000; //1000: Reference span = 20 V ± 1.25 V
    uint32_t PDN = 0;          //DAC power up

    uint32_t ol_led_cnt[ADC_COUNT] = {0, 0, 0};
    const uint8_t ol_led_pin[ADC_COUNT] = {_OVERLOAD_LED1, _OVERLOAD_LED2, _OVERLOAD_LED3};
    


    // ADC/DAC software->hardware mappings
    const size_t ADC_REMAP_TABLE[ADC_COUNT] = {0x02, 0x01, 0x00};
    //const size_t DAC_REMAP_TABLE[DAC_COUNT] = {0x02, 0x01, 0x00, 0x03};
    const size_t DAC_REMAP_TABLE[DAC_COUNT] = {0x01, 0x00, 0x02};


    uint8_t m_dac_spi_data[DAC_SPI_BLOCK_SIZE];
    uint8_t m_adc_spi_data[ADC_SPI_BLOCK_SIZE];
    uint8_t m_adc_spi_data_dummy[ADC_SPI_BLOCK_SIZE];
    uint8_t m_dac_spi_data_dummy[DAC_SPI_BLOCK_SIZE];

    EventResponder* m_pSPI0Event;
    EventResponder* m_pSPI1Event;
    


    void dac_init(void);
    void trigger_adcs(void);
    void calculate_dac_data(uint8_t* m_dac_spi_data, int32_t* i_dac_results);
    void spi_write_dac_data(uint8_t* m_dac_spi_data);
    void spi_read_adc_data();
    
  

};

#endif
