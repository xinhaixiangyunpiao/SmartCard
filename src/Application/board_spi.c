/*********************************************************************
 * INCLUDES
 */
#include "nrf_drv_spi.h"
#include "nrf_gpio.h"
#include "app_error.h"

#include "board_spi.h"

static void spiCallbackFunc(nrf_drv_spi_evt_t const *pEvent, void *arg);

/*********************************************************************
 * LOCAL VARIABLES
 */
static volatile bool s_transferOk = true;  										// SPI数据传输完成标志
static const nrf_drv_spi_t s_spiHandle = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);	// SPI instance

/*********************************************************************
 * PUBLIC FUNCTIONS
 */
/**
 @brief SPI的初始化函数
 @param 无
 @return 无
*/
void SPI_Init(void)
{
	ret_code_t errCode;
	
	nrf_drv_spi_config_t spiConfig = NRF_DRV_SPI_DEFAULT_CONFIG;				// 使用SPI默认配置
	// 配置SPI端口，注意CSN不要在这设置，另外用GPIO口控制
	spiConfig.miso_pin = BOARD_SPI0_MISO_IO;
	spiConfig.mosi_pin = BOARD_SPI0_MOSI_IO;
	spiConfig.sck_pin = BOARD_SPI0_CLK_IO;
	spiConfig.mode = NRF_DRV_SPI_MODE_0;
	spiConfig.frequency = SPI_FREQUENCY_FREQUENCY_K250;	
	spiConfig.irq_priority = 4;													// 在定时器中使用优先级需小于6
	
	errCode = nrf_drv_spi_init(&s_spiHandle, &spiConfig, spiCallbackFunc, NULL);
	APP_ERROR_CHECK(errCode);
	
	nrf_gpio_cfg_output(BOARD_SPI0_CSN_IO);
}

/**
 @brief SPI片选信号设置
 @param pinState -[in] 引脚状态
 @return 无
*/
void SPI_CsnSet(bool pinState)
{
    if(pinState == BOARD_SPI_CS_OFF)
    {
		nrf_gpio_pin_write(BOARD_SPI0_CSN_IO, 1);
    }
    else if(pinState == BOARD_SPI_CS_ON)
    {
        nrf_gpio_pin_write(BOARD_SPI0_CSN_IO, 0);
    }
}

/**
 @brief SPI读出写入数据
 @param pWriteData -[in] 写入数据
 @param pReadData -[out] 读出数据
 @param writeDataLen -[in] 写入数据长度
 @return 无
*/
void SPI_ReadWriteData(uint8 *pWriteData, uint8 *pReadData, uint8 writeDataLen)
{
	s_transferOk = false;

	APP_ERROR_CHECK(nrf_drv_spi_transfer(&s_spiHandle, pWriteData, writeDataLen, pReadData, writeDataLen));

	while(!s_transferOk)
	{
		__WFE();
	}																			// Error in SPI or transfer already in progress.
}

/**
 @brief 开启SPI，与初始化区别：没有初始化CS引脚
 @param 无
 @return 无
*/
void SPI_Enable(void)
{
	ret_code_t errCode;
	
	nrf_drv_spi_config_t spiConfig = NRF_DRV_SPI_DEFAULT_CONFIG;				// 使用SPI默认配置
	// 配置SPI端口，注意CSN不要在这设置，另外用GPIO口控制
	spiConfig.miso_pin = BOARD_SPI0_MISO_IO;
	spiConfig.mosi_pin = BOARD_SPI0_MOSI_IO;
	spiConfig.sck_pin = BOARD_SPI0_CLK_IO;
	spiConfig.mode = NRF_DRV_SPI_MODE_0;
	spiConfig.frequency = SPI_FREQUENCY_FREQUENCY_M8;	
	spiConfig.irq_priority = 4;													// 在定时器中使用优先级需小于6
	
	errCode = nrf_drv_spi_init(&s_spiHandle, &spiConfig, spiCallbackFunc, NULL);
	APP_ERROR_CHECK(errCode);
}

/**
 @brief 禁用SPI
 @param 无
 @return 无
*/
void SPI_Disable(void)
{
	nrf_drv_spi_uninit(&s_spiHandle);
}


/*********************************************************************
 * LOCAL FUNCTIONS
 */
/**
 @brief SPI中断处理回调函数
 @param 无
 @return 无
*/
static void spiCallbackFunc(nrf_drv_spi_evt_t const *pEvent, void *arg)
{
	s_transferOk = true;
}

/****************************************************END OF FILE****************************************************/
