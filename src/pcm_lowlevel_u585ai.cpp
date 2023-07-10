#if defined(ARDUINO_STEVAL_MKBOXPRO)

#include <PCM.h>

#define SaturaLH(N, L, H) (((N)<(L))?(L):(((N)>(H))?(H):(N)))

/* DFSDM Configuration defines */
#define DMIC_ONBOARD_FILTER                     ADF1_Filter0
#define DMIC_ONBOARD_MDFx_CLK_ENABLE()          __HAL_RCC_ADF1_CLK_ENABLE()
#define DMIC_ONBOARD_DMAx_CLK_ENABLE()          __HAL_RCC_GPDMA1_CLK_ENABLE()

#define DMIC_ONBAORD_DMA_REQUEST                GPDMA1_REQUEST_ADF1_FLT0
#define DMIC_ONBAORD_DMA_IRQn                   GPDMA1_Channel1_IRQn

#define DMIC_ONBOARD_CKOUT_GPIO_CLK_ENABLE()    __HAL_RCC_GPIOE_CLK_ENABLE()
#define DMIC_ONBOARD_CKOUT_GPIO_PORT            GPIOE
#define DMIC_ONBOARD_CKOUT_AF                   GPIO_AF3_ADF1
#define DMIC_ONBOARD_CKOUT_PIN                  GPIO_PIN_9

#define DMIC_ONBOARD_DATAIN_GPIO_CLK_ENABLE()   __HAL_RCC_GPIOE_CLK_ENABLE()
#define DMIC_ONBOARD_DATIN_GPIO_PORT            GPIOE
#define DMIC_ONBOARD_DATAIN_AF                  GPIO_AF3_ADF1
#define DMIC_ONBOARD_DATIN_PIN                  GPIO_PIN_10

#ifdef __cplusplus
extern "C" {
#endif

MDF_HandleTypeDef       DMic_OnBoard_AdfHandle;
MDF_FilterConfigTypeDef DMic_OnBoard_AdfFilterConfig;
DMA_HandleTypeDef       DMic_OnBoard_DmaHandle;
DMA_QListTypeDef        DMic_OnBoard_MdfQueue;
DMA_NodeTypeDef         DMic_OnBoard_DmaNode;
MDF_DmaConfigTypeDef    DMic_OnBoard_DMAConfig;
int16_t                 DMic_OnBoard_Buffer[DEFAULT_AUDIO_IN_BUFFER_SIZE];
int                     DMic_OnBoard_CountSkip;

int pcm_lowlevel_init(uint32_t sampleRate)
{
  int ret = PCM_OK;

  switch (sampleRate) {
    case AUDIO_FREQUENCY_16K:
      /**
        * Input clock: 15.36MHz
        * Output clock divider = 10
        * MIC_CLK = 15.36MHz / 10 = 1.536MHz
        */
      DMic_OnBoard_AdfHandle.Init.CommonParam.OutputClock.Divider = 10;
      break;
    default:
      return PCM_ERROR;
      break;
  }

  /* Initialize MDF */
  DMic_OnBoard_AdfHandle.Instance                                        = DMIC_ONBOARD_FILTER;
  DMic_OnBoard_AdfHandle.Init.CommonParam.InterleavedFilters             = 0U;
  DMic_OnBoard_AdfHandle.Init.CommonParam.ProcClockDivider               = 1U;
  DMic_OnBoard_AdfHandle.Init.CommonParam.OutputClock.Activation         = ENABLE;
  DMic_OnBoard_AdfHandle.Init.CommonParam.OutputClock.Pins               = MDF_OUTPUT_CLOCK_0;
  DMic_OnBoard_AdfHandle.Init.CommonParam.OutputClock.Trigger.Activation = ENABLE;
  DMic_OnBoard_AdfHandle.Init.CommonParam.OutputClock.Trigger.Source     = MDF_CLOCK_TRIG_TRGO;
  DMic_OnBoard_AdfHandle.Init.CommonParam.OutputClock.Trigger.Edge       = MDF_CLOCK_TRIG_RISING_EDGE;
  DMic_OnBoard_AdfHandle.Init.SerialInterface.Activation                 = ENABLE;
  DMic_OnBoard_AdfHandle.Init.SerialInterface.Mode                       = MDF_SITF_NORMAL_SPI_MODE;
  DMic_OnBoard_AdfHandle.Init.SerialInterface.ClockSource                = MDF_SITF_CCK0_SOURCE;
  DMic_OnBoard_AdfHandle.Init.SerialInterface.Threshold                  = 31U;
  DMic_OnBoard_AdfHandle.Init.FilterBistream                             = MDF_BITSTREAM0_RISING;

  if (HAL_MDF_Init(&DMic_OnBoard_AdfHandle) != HAL_OK) {
    return PCM_ERROR;
  }

  switch (sampleRate) {
    case AUDIO_FREQUENCY_16K:
      /**
        * MIC_CLK = 1.536MHz
        * Filter = SINC5
        * Decimation Ratio = 24
        * ReshapeFilter Enabled - Decimation Ratio = 4
        * Audio signal = 1.536MHz / (24 * 4) = 16kHz
        *
        * Sinc5, d=24 --> 24bit --> Gain = -4
        * Gain it's expressed in 3dB steps --> half a bit
        */
      DMic_OnBoard_AdfFilterConfig.Gain = 6;
      DMic_OnBoard_AdfFilterConfig.DecimationRatio = 24;
      break;
    default:
      return PCM_ERROR;
      break;
  }

  /** Initialize filter configuration parameters */
  DMic_OnBoard_AdfFilterConfig.DataSource                         = MDF_DATA_SOURCE_BSMX;
  DMic_OnBoard_AdfFilterConfig.Delay                              = 0U;
  DMic_OnBoard_AdfFilterConfig.CicMode                            = MDF_ONE_FILTER_SINC5;
  DMic_OnBoard_AdfFilterConfig.Offset                             = 0;
  DMic_OnBoard_AdfFilterConfig.ReshapeFilter.Activation           = ENABLE;
  DMic_OnBoard_AdfFilterConfig.ReshapeFilter.DecimationRatio      = MDF_RSF_DECIMATION_RATIO_4;
  DMic_OnBoard_AdfFilterConfig.HighPassFilter.Activation          = ENABLE;
  DMic_OnBoard_AdfFilterConfig.HighPassFilter.CutOffFrequency     = MDF_HPF_CUTOFF_0_000625FPCM;
  DMic_OnBoard_AdfFilterConfig.Integrator.Activation              = DISABLE;
  DMic_OnBoard_AdfFilterConfig.SoundActivity.Activation           = DISABLE;
  DMic_OnBoard_AdfFilterConfig.FifoThreshold                      = MDF_FIFO_THRESHOLD_NOT_EMPTY;
  DMic_OnBoard_AdfFilterConfig.DiscardSamples                     = 0U;
  DMic_OnBoard_AdfFilterConfig.SnapshotFormat                     = MDF_SNAPSHOT_23BITS;
  DMic_OnBoard_AdfFilterConfig.Trigger.Source                     = MDF_FILTER_TRIG_TRGO;
  DMic_OnBoard_AdfFilterConfig.Trigger.Edge                       = MDF_FILTER_TRIG_RISING_EDGE;
  DMic_OnBoard_AdfFilterConfig.AcquisitionMode                    = MDF_MODE_SYNC_CONT;

  return ret;
}

int pcm_lowlevel_deinit()
{
  int ret = PCM_OK;

  /* De-initializes DFSDM Filter handle */
  if (DMic_OnBoard_AdfHandle.Instance != NULL) {
    if (HAL_MDF_DeInit(&DMic_OnBoard_AdfHandle) != HAL_OK) {
      ret =  PCM_ERROR;
    }
    DMic_OnBoard_AdfHandle.Instance = NULL;
  }

  return ret;
}

int pcm_lowlevel_start()
{
  int ret = PCM_OK;

  /* Initialize DMA configuration parameters */
  DMic_OnBoard_DMAConfig.Address    = (uint32_t)&DMic_OnBoard_Buffer[0];
  DMic_OnBoard_DMAConfig.DataLength = (DEFAULT_AUDIO_IN_BUFFER_SIZE * 2U);
  DMic_OnBoard_DMAConfig.MsbOnly    = ENABLE;

  if (HAL_MDF_AcqStart_DMA(
        &DMic_OnBoard_AdfHandle,
        &DMic_OnBoard_AdfFilterConfig,
        &DMic_OnBoard_DMAConfig) != HAL_OK) {
    ret = PCM_ERROR;
  }

  if (HAL_MDF_GenerateTrgo(&DMic_OnBoard_AdfHandle) != HAL_OK) {
    ret = PCM_ERROR;
  }

  return ret;
}

int pcm_lowlevel_stop()
{
  int ret = PCM_OK;

  if (HAL_MDF_AcqStop_DMA(&DMic_OnBoard_AdfHandle) != HAL_OK) {
    ret = PCM_ERROR;
  }

  return ret;
}

/**
* @brief  Regular conversion complete callback.
* @note   In interrupt mode, user has to read conversion value in this function
using HAL_DFSDM_FilterGetRegularValue.
* @param  hmdf   MDF filter handle.
* @retval None
*/
void HAL_MDF_AcqCpltCallback(MDF_HandleTypeDef *hmdf)
{
  UNUSED(hmdf);

  if (PCM.Buffer == nullptr) {
    return;
  }

  if (DMic_OnBoard_CountSkip < 64) {
    ++DMic_OnBoard_CountSkip;
    return;
  }

  for (uint32_t j = 0U; j < ((PCM.SampleRate / 1000U) * N_MS_PER_INTERRUPT); j++) {
    int32_t Z = ((DMic_OnBoard_Buffer[j + ((PCM.SampleRate / 1000U) * N_MS_PER_INTERRUPT)]) * (int32_t)(PCM.Volume)) / 100;
    PCM.Buffer[j] = (uint16_t) SaturaLH(Z, -32760, 32760);
  }

  if (PCM.OnRecv != nullptr) {
    PCM.OnRecv();
  }
}

/**
* @brief  Half regular conversion complete callback.
* @param  hmdf   MDF filter handle.
* @retval None
*/
void HAL_MDF_AcqHalfCpltCallback(MDF_HandleTypeDef *hmdf)
{
  UNUSED(hmdf);

  if (PCM.Buffer == nullptr) {
    return;
  }

  if (DMic_OnBoard_CountSkip < 64) {
    ++DMic_OnBoard_CountSkip;
    return;
  }

  for (uint32_t j = 0U; j < ((PCM.SampleRate / 1000U) * N_MS_PER_INTERRUPT); j++) {
    int32_t Z = (DMic_OnBoard_Buffer[j] * (int32_t)(PCM.Volume)) / 100;
    PCM.Buffer[j] = (uint16_t) SaturaLH(Z, -32760, 32760);
  }

  if (PCM.OnRecv != nullptr) {
    PCM.OnRecv();
  }
}

/**
 * Initialize HAL MDF.
 * @param  hmdf   MDF filter handle.
 * @retval None
 */
void HAL_MDF_MspInit(MDF_HandleTypeDef *hmdf)
{
  DMA_NodeConfTypeDef dmaLinkNodeConfig;
  GPIO_InitTypeDef  GPIO_Init;

  if (hmdf->Instance == DMIC_ONBOARD_FILTER) {
    /** Initializes the peripherals clock */
    RCC_PeriphCLKInitTypeDef PeriphClkInit;
    memset(&PeriphClkInit, 0x0, sizeof(RCC_PeriphCLKInitTypeDef));
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADF1;
    PeriphClkInit.Adf1ClockSelection = RCC_ADF1CLKSOURCE_PLL3;
    PeriphClkInit.PLL3.PLL3Source = RCC_PLLSOURCE_HSE;
    PeriphClkInit.PLL3.PLL3M = 2;
    PeriphClkInit.PLL3.PLL3N = 48;
    PeriphClkInit.PLL3.PLL3P = 2;
    PeriphClkInit.PLL3.PLL3Q = 25;
    PeriphClkInit.PLL3.PLL3R = 2;
    PeriphClkInit.PLL3.PLL3RGE = RCC_PLLVCIRANGE_0;
    PeriphClkInit.PLL3.PLL3FRACN = 0;
    PeriphClkInit.PLL3.PLL3ClockOut = RCC_PLL3_DIVQ;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
      Error_Handler();
    }

    __HAL_RCC_ADF1_CONFIG(RCC_ADF1CLKSOURCE_PLL3);

    /* Enable MDF1 clock */
    DMIC_ONBOARD_MDFx_CLK_ENABLE();

    /* Configure GPIOs used for ADF1 */
    DMIC_ONBOARD_CKOUT_GPIO_CLK_ENABLE();
    DMIC_ONBOARD_DATAIN_GPIO_CLK_ENABLE();

    GPIO_Init.Mode      = GPIO_MODE_AF_PP;
    GPIO_Init.Pull      = /*GPIO_NOPULL*/GPIO_PULLDOWN;
    GPIO_Init.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;

    GPIO_Init.Alternate = DMIC_ONBOARD_CKOUT_AF;
    GPIO_Init.Pin       = DMIC_ONBOARD_CKOUT_PIN;
    HAL_GPIO_Init(DMIC_ONBOARD_CKOUT_GPIO_PORT, &GPIO_Init);

    GPIO_Init.Alternate = DMIC_ONBOARD_DATAIN_AF;
    GPIO_Init.Pin       = DMIC_ONBOARD_DATIN_PIN;
    HAL_GPIO_Init(DMIC_ONBOARD_DATIN_GPIO_PORT, &GPIO_Init);

    /* Configure DMA used for ADF1 */
    DMIC_ONBOARD_DMAx_CLK_ENABLE();

    /* Set node type */
    dmaLinkNodeConfig.NodeType                            = DMA_GPDMA_LINEAR_NODE;
    /* Set common node parameters */
    dmaLinkNodeConfig.Init.Request                        = DMIC_ONBAORD_DMA_REQUEST;
    dmaLinkNodeConfig.Init.BlkHWRequest                   = DMA_BREQ_SINGLE_BURST;
    dmaLinkNodeConfig.Init.Direction                      = DMA_PERIPH_TO_MEMORY;
    dmaLinkNodeConfig.Init.SrcInc                         = DMA_SINC_FIXED;
    dmaLinkNodeConfig.Init.DestInc                        = DMA_DINC_INCREMENTED;
    dmaLinkNodeConfig.Init.SrcDataWidth                   = DMA_SRC_DATAWIDTH_HALFWORD;
    dmaLinkNodeConfig.Init.DestDataWidth                  = DMA_DEST_DATAWIDTH_HALFWORD;
    dmaLinkNodeConfig.Init.SrcBurstLength                 = 1;
    dmaLinkNodeConfig.Init.DestBurstLength                = 1;
    dmaLinkNodeConfig.Init.Priority                       = DMA_HIGH_PRIORITY;
    dmaLinkNodeConfig.Init.TransferEventMode              = DMA_TCEM_EACH_LL_ITEM_TRANSFER;
    dmaLinkNodeConfig.Init.TransferAllocatedPort          = DMA_SRC_ALLOCATED_PORT0 | DMA_DEST_ALLOCATED_PORT1;
    dmaLinkNodeConfig.Init.Mode                           = DMA_NORMAL;

    /* Set node data handling parameters */
    dmaLinkNodeConfig.DataHandlingConfig.DataExchange     = DMA_EXCHANGE_NONE;
    dmaLinkNodeConfig.DataHandlingConfig.DataAlignment    = DMA_DATA_UNPACK;

    /* Set node trigger parameters */
    dmaLinkNodeConfig.TriggerConfig.TriggerPolarity       = DMA_TRIG_POLARITY_MASKED;
    dmaLinkNodeConfig.RepeatBlockConfig.RepeatCount         = 1U;
    dmaLinkNodeConfig.RepeatBlockConfig.SrcAddrOffset       = 0;
    dmaLinkNodeConfig.RepeatBlockConfig.DestAddrOffset      = 0;
    dmaLinkNodeConfig.RepeatBlockConfig.BlkSrcAddrOffset    = 0;
    dmaLinkNodeConfig.RepeatBlockConfig.BlkDestAddrOffset   = 0;

    /* Build NodeTx */
    HAL_DMAEx_List_BuildNode(&dmaLinkNodeConfig, &DMic_OnBoard_DmaNode);

    /* Insert NodeTx to SAI queue */
    HAL_DMAEx_List_InsertNode_Tail(&DMic_OnBoard_MdfQueue, &DMic_OnBoard_DmaNode);

    /* Select the DMA instance to be used for the transfer : GPDMA_Channel1 */
    DMic_OnBoard_DmaHandle.Instance                         = GPDMA1_Channel1;

    /* Set queue circular mode for sai queue */
    HAL_DMAEx_List_SetCircularMode(&DMic_OnBoard_MdfQueue);

    DMic_OnBoard_DmaHandle.InitLinkedList.Priority          = DMA_HIGH_PRIORITY;
    DMic_OnBoard_DmaHandle.InitLinkedList.LinkStepMode      = DMA_LSM_FULL_EXECUTION;
    DMic_OnBoard_DmaHandle.InitLinkedList.LinkAllocatedPort = DMA_LINK_ALLOCATED_PORT0;
    DMic_OnBoard_DmaHandle.InitLinkedList.TransferEventMode = DMA_TCEM_LAST_LL_ITEM_TRANSFER;
    DMic_OnBoard_DmaHandle.InitLinkedList.LinkedListMode    = DMA_LINKEDLIST_CIRCULAR;

    /* DMA linked list init */
    HAL_DMAEx_List_Init(&DMic_OnBoard_DmaHandle);

    /* Link SAI queue to DMA channel */
    HAL_DMAEx_List_LinkQ(&DMic_OnBoard_DmaHandle, &DMic_OnBoard_MdfQueue);

    /* Associate the DMA handle */
    __HAL_LINKDMA(hmdf, hdma, DMic_OnBoard_DmaHandle);

    HAL_NVIC_SetPriority(DMIC_ONBAORD_DMA_IRQn, 0x01, 0);
    HAL_NVIC_EnableIRQ(DMIC_ONBAORD_DMA_IRQn);
  }
}

/**
* @brief This function handles MDF DMA interrupt request.
* @param None
* @retval None
*/
void GPDMA1_Channel1_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&DMic_OnBoard_DmaHandle);
}

#ifdef __cplusplus
}
#endif

#endif /* ARDUINO_STEVAL_MKBOXPRO */
