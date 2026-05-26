/*
 * AnalogInput.cpp
 *
 *  Created 30/7/2020
 *      Author: Andy
 */

/*
Read ADC values. We configure things so that the selected channels are constantly being converted.
Only those channels attached to ADC1 and ADC3 plus a single special case of the MCU temperature device on
ADC1 are currently supported.

Sample timing
We sample continuously up to 16 channels per ADC. We set the ADC clock to APB2/8 and sample each channel for 480 cycles
the conversion time is 12 cycles. This gives a per channel time of...
(480+12)/(84/8) uS = 47uS
Total time for for 16 channels 47*16 = 750uS or approx 1333 full samples per second.
Each channel is oversampled 16 times to increase the resolution by 2 bits to 14 bits.
with oversampling we get approx 83 samples per second. We can read values more frequantly than
this, bt if we do then the readings will contain samples that were also present in previous
readings.

Note:
The above timing is for stm32f4 based mcus. On stm32H72x devices ADC3 only supports 12bit conversions
for this reason on H7 based devices we only convert to 12bits and use the same oversampling scheme as used
on the F4 based devices. We should probably consider using higher sample avlues and/or using the built in
oversampling.
*/
#ifdef RTOS
#include <CoreImp.h>
#include <AnalogIn.h>
#include <Cache.h>

extern "C" void debugPrintf(const char* fmt, ...) __attribute__ ((format (printf, 1, 2)));

constexpr uint32_t NumADCs = 3;                          // Max supported ADCs
constexpr uint32_t OversampleBits = 2;                   // Number of extra bit of resolution
constexpr uint32_t Oversample = 16;                      // 4^OversampleBits
constexpr uint32_t MaxActiveChannels = 16;               // Max active ADC channels
constexpr AnalogChannelNumber ADC_1 = 0x10000;
constexpr AnalogChannelNumber ADC_2 = 0x20000;
constexpr AnalogChannelNumber ADC_3 = 0x30000;
#if STM32H7
# define __nocache		__attribute__((section(".ram_nocache")))
constexpr uint32_t NumChannelsADC1 = 20;
constexpr uint32_t NumChannelsADC3 = 20;
constexpr uint32_t StmChanMap1[] = {ADC_CHANNEL_0, ADC_CHANNEL_1, ADC_CHANNEL_2, ADC_CHANNEL_3, ADC_CHANNEL_4, ADC_CHANNEL_5, ADC_CHANNEL_6,
                                    ADC_CHANNEL_7, ADC_CHANNEL_8, ADC_CHANNEL_9, ADC_CHANNEL_10, ADC_CHANNEL_11, ADC_CHANNEL_12, ADC_CHANNEL_13,
                                    ADC_CHANNEL_14, ADC_CHANNEL_15, ADC_CHANNEL_16, ADC_CHANNEL_17, ADC_CHANNEL_18, ADC_CHANNEL_19};
constexpr uint32_t StmChanMap3[] = {ADC_CHANNEL_0, ADC_CHANNEL_1, ADC_CHANNEL_2, ADC_CHANNEL_3, ADC_CHANNEL_4, ADC_CHANNEL_5, ADC_CHANNEL_6,
                                    ADC_CHANNEL_7, ADC_CHANNEL_8, ADC_CHANNEL_9, ADC_CHANNEL_10, ADC_CHANNEL_11, ADC_CHANNEL_12, ADC_CHANNEL_13,
                                    ADC_CHANNEL_14, ADC_CHANNEL_15, ADC_CHANNEL_16, ADC_CHANNEL_VBAT, LL_ADC_CHANNEL_TEMPSENSOR, ADC_CHANNEL_VREFINT};
constexpr uint32_t CHAN_VREFINT = (ADC_3 | 19);
constexpr uint32_t CHAN_TEMPSENSOR = (ADC_3 | 18);
constexpr uint32_t ADC1DMA = DMA_REQUEST_ADC1;
constexpr uint32_t ADC3DMA = DMA_REQUEST_ADC3;
constexpr uint32_t ADC_SAMPLETIME = ADC_SAMPLETIME_387CYCLES_5;
#else
# define __nocache
constexpr uint32_t NumChannelsADC1 = 19;
constexpr uint32_t NumChannelsADC3 = 16;
constexpr uint32_t StmChanMap1[] = {ADC_CHANNEL_0, ADC_CHANNEL_1, ADC_CHANNEL_2, ADC_CHANNEL_3, ADC_CHANNEL_4, ADC_CHANNEL_5, ADC_CHANNEL_6,
                                    ADC_CHANNEL_7, ADC_CHANNEL_8, ADC_CHANNEL_9, ADC_CHANNEL_10, ADC_CHANNEL_11, ADC_CHANNEL_12, ADC_CHANNEL_13,
                                    ADC_CHANNEL_14, ADC_CHANNEL_15, ADC_CHANNEL_TEMPSENSOR, ADC_CHANNEL_VREFINT, ADC_CHANNEL_VBAT};
constexpr uint32_t StmChanMap3[] = {ADC_CHANNEL_0, ADC_CHANNEL_1, ADC_CHANNEL_2, ADC_CHANNEL_3, ADC_CHANNEL_4, ADC_CHANNEL_5, ADC_CHANNEL_6,
                                    ADC_CHANNEL_7, ADC_CHANNEL_8, ADC_CHANNEL_9, ADC_CHANNEL_10, ADC_CHANNEL_11, ADC_CHANNEL_12, ADC_CHANNEL_13,
                                    ADC_CHANNEL_14, ADC_CHANNEL_15};
constexpr uint32_t CHAN_VREFINT = (ADC_1 | 17);
constexpr uint32_t CHAN_TEMPSENSOR = (ADC_1 | 16);
constexpr uint32_t ADC1DMA = DMA_CHANNEL_0;
constexpr uint32_t ADC3DMA = DMA_CHANNEL_2;
constexpr uint32_t ADC_SAMPLETIME = ADC_SAMPLETIME_480CYCLES;
#endif
static constexpr const uint32_t *StmChanMap[NumADCs+1] = {nullptr, StmChanMap1, nullptr, StmChanMap3};
constexpr uint32_t NumChannels[NumADCs+1] = {0, NumChannelsADC1, 0, NumChannelsADC3};
#if defined(ADC_REGULAR_RANK_1)
constexpr uint32_t Ranks[] = { ADC_REGULAR_RANK_1, ADC_REGULAR_RANK_2, ADC_REGULAR_RANK_3, ADC_REGULAR_RANK_4, ADC_REGULAR_RANK_5,
                               ADC_REGULAR_RANK_6, ADC_REGULAR_RANK_7, ADC_REGULAR_RANK_8, ADC_REGULAR_RANK_9, ADC_REGULAR_RANK_10,
                               ADC_REGULAR_RANK_11, ADC_REGULAR_RANK_12, ADC_REGULAR_RANK_13, ADC_REGULAR_RANK_14, ADC_REGULAR_RANK_15,
                               ADC_REGULAR_RANK_16 };
#else
constexpr uint32_t Ranks[] = { 1, 2, 3, 4, 5,
                               6, 7, 8, 9, 10,
                               11, 12, 13, 14, 15,
                               16 };
#endif

static int32_t ChanMap1[NumChannelsADC1];
static int32_t ChanMap3[NumChannelsADC3];
static __nocache uint32_t ChanValues[MaxActiveChannels*Oversample];
static constexpr int32_t *ChanMap[NumADCs+1] = {nullptr, ChanMap1, nullptr, ChanMap3};
static uint8_t NumActiveChannels[NumADCs+1] = {0, 0, 0, 0};
static bool Adc1Running = false;
static bool Adc3Running = false;

// Configuration structures for the ADC and Dma
static ADC_HandleTypeDef Adc1Handle = {};
static ADC_HandleTypeDef Adc3Handle = {};
static DMA_HandleTypeDef Dma1Handle = {};
static DMA_HandleTypeDef Dma3Handle = {};

extern "C" void DMA2_Stream4_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&Dma1Handle);
}
extern "C" void DMA2_Stream0_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&Dma3Handle);
}

/* Private Functions */
static AnalogChannelNumber GetAdcChannel(PinName pin)
{
    uint32_t function = pinmap_function(pin, PinMap_ADC);
    if (function == static_cast<uint32_t>(NC)) return NO_ADC;
    // we return the ADC number in the high 16 bits and the channel number in the low
    ADC_TypeDef* adc = (ADC_TypeDef *)pinmap_peripheral(pin, PinMap_ADC);

    // We only support ADC3 and ADC1 at the moment
    if (adc == ADC1)
        return (ADC_1 | STM_PIN_CHANNEL(function));
#if 0
    else if (adc == ADC2)
        return (ADC_2 | STM_PIN_CHANNEL(function));
#endif
    else if (adc == ADC3)
        return (ADC_3 | STM_PIN_CHANNEL(function));
    else
    {
        return NO_ADC;
    }
}


static void ConfigureDma(DMA_HandleTypeDef& DmaHandle, DMA_Stream_TypeDef *inst, uint32_t chan )
{
    DmaHandle.Instance = inst;

#if STM32H7
    DmaHandle.Init.Request  = chan;
#else
    DmaHandle.Init.Channel  = chan;
#endif
    DmaHandle.Init.Direction = DMA_PERIPH_TO_MEMORY;
    DmaHandle.Init.PeriphInc = DMA_PINC_DISABLE;
    DmaHandle.Init.MemInc = DMA_MINC_ENABLE;
    DmaHandle.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    DmaHandle.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    DmaHandle.Init.Mode = DMA_CIRCULAR;
    DmaHandle.Init.Priority = DMA_PRIORITY_LOW;
    DmaHandle.Init.FIFOMode = DMA_FIFOMODE_DISABLE;         
    DmaHandle.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_HALFFULL;
    DmaHandle.Init.MemBurst = DMA_MBURST_SINGLE;
    DmaHandle.Init.PeriphBurst = DMA_PBURST_SINGLE; 

    HAL_DMA_DeInit(&DmaHandle);
    HAL_DMA_Init(&DmaHandle);
}


static void ConfigureAdc(ADC_HandleTypeDef& AdcHandle, ADC_TypeDef *inst, uint32_t chanCount)
{
    // Adc converts channels, continuously and
    // captured to RAM via DMA
    AdcHandle.Instance = inst;
#if STM32H7xx
    AdcHandle.Init.ClockPrescaler           = ADC_CLOCK_SYNC_PCLK_DIV4;
    AdcHandle.Init.LowPowerAutoWait         = DISABLE;                       /* Auto-delayed conversion feature disabled */
    AdcHandle.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_CIRCULAR; /* ADC DMA circular requested */
    AdcHandle.Init.OversamplingMode         = DISABLE;                       /* No oversampling */
    AdcHandle.Init.Overrun                  = ADC_OVR_DATA_OVERWRITTEN;      /* DR register is overwritten with the last conversion result in case of overrun */
    AdcHandle.Init.EOCSelection             = ADC_EOC_SINGLE_CONV;           /* EOC flag picked-up to indicate conversion end */
#if STM32H723xx
    AdcHandle.Init.DMAContinuousRequests    = ENABLE;
#endif
#else
    AdcHandle.Init.ClockPrescaler           = ADC_CLOCK_SYNC_PCLK_DIV8;
    AdcHandle.Init.DataAlign                = ADC_DATAALIGN_RIGHT;           /* Right-alignment for converted data */
    AdcHandle.Init.EOCSelection             = DISABLE;                       /* EOC flag picked-up to indicate conversion end */
    AdcHandle.Init.DMAContinuousRequests = ENABLE;                        /* DMA continuous mode enabled */
#endif
    AdcHandle.Init.Resolution               = ADC_RESOLUTION_12B;            /* 12-bit resolution for converted data */
    AdcHandle.Init.ScanConvMode             = ENABLE;                        /* Sequencer disabled (ADC conversion on only 1 channel: channel set on rank 1) */
    AdcHandle.Init.ContinuousConvMode       = ENABLE;                        /* Continuous mode enabled (automatic conversion restart after each conversion) */
    AdcHandle.Init.NbrOfConversion          = chanCount;                     /* Parameter discarded because sequencer is disabled */
    AdcHandle.Init.DiscontinuousConvMode    = DISABLE;                       /* Parameter discarded because sequencer is disabled */
    AdcHandle.Init.NbrOfDiscConversion      = 0;                             /* Parameter discarded because sequencer is disabled */
    AdcHandle.Init.ExternalTrigConv         = ADC_SOFTWARE_START;            /* Software start to trig the 1st conversion manually, without external event */
    AdcHandle.Init.ExternalTrigConvEdge     = ADC_EXTERNALTRIGCONVEDGE_NONE; /* Parameter discarded because software trigger chosen */

    AdcHandle.State = HAL_ADC_STATE_RESET;
    AdcHandle.Lock = HAL_UNLOCKED;
    /* Some other ADC_HandleTypeDef fields exists but not required */
    if (HAL_ADC_Init(&AdcHandle) != HAL_OK) 
    {
        debugPrintf("ADC Init failed\n");
        return;
    }
}

static uint32_t GetActiveChannels(int32_t *cm, uint32_t sz)
{
    uint32_t cnt = 0;
    for(uint32_t i = 0; i < sz; i++)
        if (cm[i] != -1)
            cnt++;
    return cnt;
}

static uint32_t AddActiveChannels(ADC_HandleTypeDef& AdcHandle, uint32_t AdcNum, int32_t *cm, uint32_t sz, uint32_t dataOffset)
{
    ADC_ChannelConfTypeDef  AdcChannelConf = {};
    AdcChannelConf.SamplingTime = ADC_SAMPLETIME;
    AdcChannelConf.Offset = 0;                                 /* Parameter discarded because offset correction is disabled */
#if STM32H7
    AdcChannelConf.SingleDiff   = ADC_SINGLE_ENDED;            /* Single-ended input channel */
    AdcChannelConf.OffsetNumber = ADC_OFFSET_NONE;             /* No offset subtraction */
    AdcChannelConf.OffsetRightShift = DISABLE;                 /* No Right Offset Shift */
    AdcChannelConf.OffsetSignedSaturation = DISABLE;           /* Signed saturation feature is not used */
#endif

    int32_t sampleOffset = 0;
    for(uint32_t i = 0; i < sz; i++)
    {
        if (cm[i] != -1)
        {
            // include this channel
            AdcChannelConf.Channel = StmChanMap[AdcNum][i];
            AdcChannelConf.Rank = Ranks[sampleOffset++];
            HAL_ADC_ConfigChannel(&AdcHandle, &AdcChannelConf);
            // record the location of the sample
            cm[i] = dataOffset++;
        }
    }
    return dataOffset;
}


static void ConfigureChannels()
{
    // Reset everything
    if (Adc3Running)
    {
        HAL_ADC_Stop_DMA(&Adc3Handle);
        // Unfortunately we can't just restart the DMA as it seems to restart at the same channel
        // it stopped at which causes a skew in the memory contents. I'm not sure what will 
        // reset the ADC to match the DMA transfer, but calling Deinit does the job. Unfortuately
        // it also resets ADC1, so we need to restart that as well!
        HAL_ADC_DeInit(&Adc3Handle);
#if defined(__HAL_RCC_ADC3_FORCE_RESET)
        __HAL_RCC_ADC3_FORCE_RESET();
        __HAL_RCC_ADC3_RELEASE_RESET();
        __HAL_RCC_ADC3_CLK_DISABLE();
#endif
    }
    if (Adc1Running)
    {
        HAL_ADC_Stop_DMA(&Adc1Handle);
        HAL_ADC_DeInit(&Adc1Handle);
#if defined(__HAL_RCC_ADC12_FORCE_RESET)
        __HAL_RCC_ADC12_FORCE_RESET();
        __HAL_RCC_ADC12_RELEASE_RESET();
        __HAL_RCC_ADC12_CLK_DISABLE();
#endif
    }
    uint32_t Active3 = GetActiveChannels(ChanMap3, NumChannelsADC3);
    uint32_t Active1 = GetActiveChannels(ChanMap1, NumChannelsADC1);
    if (Active3 + Active1 > MaxActiveChannels)
    {
        debugPrintf("Too many active ADC channels\n");
        return;
    }
    if (Active3 > 0)
    {
        __HAL_RCC_ADC3_CLK_ENABLE();
        //__HAL_RCC_ADC_CONFIG(RCC_ADCCLKSOURCE_CLKP);
        // setup the ADC for that number of channels
        ConfigureAdc(Adc3Handle, ADC3, Active3);
#if STM32H7
        HAL_ADCEx_Calibration_Start(&Adc3Handle, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
#endif
        AddActiveChannels(Adc3Handle, 3, ChanMap3, NumChannelsADC3, Active1*Oversample);
        // All done restart sampling
        HAL_ADC_Start_DMA(&Adc3Handle, ChanValues + Active1*Oversample, Active3*Oversample);
        Adc3Running = true;
    }
    NumActiveChannels[3] = Active3;
    if (Active1 > 0)
    {
#if STM32H7
        __HAL_RCC_ADC12_CLK_ENABLE();
#else
        __HAL_RCC_ADC1_CLK_ENABLE();
#endif
        //__HAL_RCC_ADC_CONFIG(RCC_ADCCLKSOURCE_CLKP);
        // setup the ADC for that number of channels
        ConfigureAdc(Adc1Handle, ADC1, Active1);
#if STM32H7
        HAL_ADCEx_Calibration_Start(&Adc1Handle, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
#endif
        AddActiveChannels(Adc1Handle, 1, ChanMap1, NumChannelsADC1, 0);
        // All done restart sampling
        HAL_ADC_Start_DMA(&Adc1Handle, ChanValues, Active1*Oversample);
        Adc1Running = true;
    }
    NumActiveChannels[1] = Active1;
}

namespace LegacyAnalogIn
{

    // Module initialisation
    void AnalogInInit()
    {
        // Initially no channels are mapped
        for(uint32_t i = 0; i < NumChannelsADC1; i++)
            ChanMap1[i] = -1;
        for(uint32_t i = 0; i < NumChannelsADC3; i++)
            ChanMap3[i] = -1;

        __HAL_RCC_DMA2_CLK_ENABLE();
        __HAL_RCC_DMA1_CLK_ENABLE();
        ConfigureDma(Dma1Handle, DMA2_Stream4, ADC1DMA);
        ConfigureDma(Dma3Handle, DMA2_Stream0, ADC3DMA);
        __HAL_LINKDMA(&Adc1Handle, DMA_Handle, Dma1Handle);
        __HAL_LINKDMA(&Adc3Handle, DMA_Handle, Dma3Handle);
        // Note we deliberately do not setup the interrupt handler normnally used for DMA operations
        // this code runs the capture/conversion process with no intervention from the mcu
    }

    // Enable or disable a channel. Use AnalogCheckReady to make sure the ADC is ready before calling this.
    void AnalogInEnableChannel(AnalogChannelNumber channel, bool enable)
    {
        //#if 0
        if (channel == NO_ADC) 
        {
            //debugPrintf("Enable bad ADC channel %d\n", static_cast<int>(channel));
            return;
        }
        AnalogChannelNumber AdcNo = (channel >> 16);
        channel &= 0xffff;
        if (AdcNo > NumADCs || channel >= NumChannels[AdcNo])
        {
            debugPrintf("Enable bad ADC channel %d %d\n", static_cast<int>(AdcNo), static_cast<int>(channel));
            return;
        }
        ChanMap[AdcNo][channel] = (enable ? 0 : -1);
        ConfigureChannels();
        //#endif
    }


    // Read the most recent 12-bit result from a channel
    uint16_t AnalogInReadChannel(AnalogChannelNumber channel)
    {
        if (channel == NO_ADC)
            return 0;
        AnalogChannelNumber AdcNo = (channel >> 16);
        channel &= 0xffff;
        uint32_t *pSamples;
        uint32_t step;
        if (AdcNo > NumADCs || channel >= NumChannels[AdcNo])
        {
            debugPrintf("Read bad ADC channel %d %d\n", static_cast<int>(AdcNo), static_cast<int>(channel));
            return 0;
        }
      
        pSamples = &ChanValues[ChanMap[AdcNo][channel]];
        step = NumActiveChannels[AdcNo];
        Cache::InvalidateAfterDMAReceive(pSamples, Oversample*step*sizeof(uint32_t));
        uint32_t val = 0;
        for(uint32_t i = 0; i < Oversample; i++)
        {
            val += *pSamples;
            pSamples += step;
        }
        // decimate
        return val >> OversampleBits;
    }


    // Start converting the enabled channels
    void AnalogInStartConversion(uint32_t channels)
    {

    }

    // Check whether all conversions have been completed since the last call to AnalogStartConversion
    bool AnalogInCheckReady(uint32_t channels)
    {
        return true;
    }

    // Convert an Arduino Due analog pin number to the corresponding ADC channel number
    AnalogChannelNumber PinToAdcChannel(uint32_t pin)
    {
        return GetAdcChannel(static_cast<PinName>(pin));
    }

    // Get the temperature measurement channel
    AnalogChannelNumber GetTemperatureAdcChannel()
    {
        return CHAN_TEMPSENSOR;
    }

    // Get the temperature measurement channel
    AnalogChannelNumber GetVREFAdcChannel()
    {
        return CHAN_VREFINT;
    }
}
#endif
// End
