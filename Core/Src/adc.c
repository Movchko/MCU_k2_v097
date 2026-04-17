#include "main.h"
#include "app.h"

uint16_t MCU_K2_ADC_VAL[MCU_K2_NUM_ADC_CHANNEL];

static uint16_t adc_filter_val[MCU_K2_NUM_ADC_CHANNEL];
static uint16_t adc_sma_buf[MCU_K2_NUM_ADC_CHANNEL][MCU_K2_FILTERSIZE];
static uint32_t adc_sma_sum[MCU_K2_NUM_ADC_CHANNEL];
static uint8_t  adc_sma_fill_index[MCU_K2_NUM_ADC_CHANNEL];
static uint8_t  adc_sma_index[MCU_K2_NUM_ADC_CHANNEL];

static volatile uint16_t ign1_adc_filtered = 0;
static volatile uint16_t ign2_adc_filtered = 0;
static volatile uint16_t ign3_adc_filtered = 0;
static volatile uint16_t u24_adc_filtered   = 0;

static uint16_t SmaProcess(uint8_t num, uint16_t val)
{
    uint16_t old_val = 0;

    if (adc_sma_fill_index[num] == MCU_K2_FILTERSIZE) {
        old_val = adc_sma_buf[num][adc_sma_index[num]];
        adc_sma_sum[num] -= old_val;
    } else {
        adc_sma_fill_index[num]++;
    }

    adc_sma_buf[num][adc_sma_index[num]] = val;
    adc_sma_sum[num] += val;

    adc_sma_index[num]++;
    if (adc_sma_index[num] >= MCU_K2_FILTERSIZE) {
        adc_sma_index[num] = 0;
    }

    return (uint16_t)(adc_sma_sum[num] / adc_sma_fill_index[num]);
}

uint16_t ADC_GetIgniter1Filtered(void) { return ign1_adc_filtered; }
uint16_t ADC_GetIgniter2Filtered(void) { return ign2_adc_filtered; }
uint16_t ADC_GetIgniter3Filtered(void) { return ign3_adc_filtered; }
uint16_t ADC_GetU24Filtered(void)      { return u24_adc_filtered; }

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc != &hadc1) {
        return;
    }

    for (uint8_t i = 0; i < MCU_K2_NUM_ADC_CHANNEL; i++) {
        adc_filter_val[i] = SmaProcess(i, MCU_K2_ADC_VAL[i]);
    }

    /* Карта каналов АЦП (MCU_k2_v097):
     * [0] CH0  -> контроль спички 3
     * [1] CH9  -> контроль спички 1
     * [2] CH11 -> входное 24В
     * [3] CH14 -> контроль спички 2
     */
    ign3_adc_filtered = adc_filter_val[0];
    ign1_adc_filtered = adc_filter_val[1];
    u24_adc_filtered  = adc_filter_val[2];
    ign2_adc_filtered = adc_filter_val[3];
}

