#ifndef __UTILS_H
#define __UTILS_H

#include "stm32f10x.h"
#include "ff.h"


#define ASSERT(e)  if (e) ; \
        else assertFail( #e, __FILE__, __LINE__ )

void Erase_SPI_Flash(void);
FRESULT scan_files (char* path);
void FileSystem_Init(void);
void FileSystem_Test(void);

void assertFail(char *exp, char *file, int line);

uint16_t CRC_Table(uint8_t *ptr, uint16_t len);
				
extern const uint8_t crcTableHigh[];
extern const uint8_t crcTableLow[];

#endif //__UTILS_H
