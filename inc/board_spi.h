#ifndef _BOARD_SPI_H_
#define _BOARD_SPI_H_

/*********************************************************************
 * INCLUDES
 */
#include <stdbool.h>

/*********************************************************************
 * DEFINITIONS
 */
#define BOARD_SPI_CS_ON             0
#define BOARD_SPI_CS_OFF            1

#define SPI_CS_HIGH                 SPI_CsnSet(BOARD_SPI_CS_OFF)
#define SPI_CS_LOW                  SPI_CsnSet(BOARD_SPI_CS_ON)

#define BOARD_SPI0_MISO_IO          0xFF
#define BOARD_SPI0_MOSI_IO          45
#define BOARD_SPI0_CLK_IO           47
#define BOARD_SPI0_CSN_IO           43

#define SPI_INSTANCE  				0			// SPI instance index 
#define uint8 unsigned char

/*********************************************************************
 * API FUNCTIONS
 */
void SPI_Init(void);
void SPI_CsnSet(bool pinState);
void SPI_ReadWriteData(uint8 *pWriteData, uint8 *pReadData, uint8 writeDataLen);
void SPI_Enable(void);
void SPI_Disable(void);

#endif /* _BOARD_SPI_H_ */
