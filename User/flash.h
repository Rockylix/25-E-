#ifndef __STMFLASH_H
#define __STMFLASH_H

#include "stm32f4xx_hal.h"  // 必须包含，因为用到了 HAL_FLASH 的类型

#ifdef __cplusplus
extern "C" {
#endif


#define FMC_FLASH_BASE      0x08000000   // FLASH的起始地址
#define FMC_FLASH_END       0x080E0000   // FLASH的结束地址

#define FLASH_WAITETIME     50000        //FLASH等待超时时间

//FLASH 扇区的起始地址
#define ADDR_FLASH_SECTOR_0     ((uint32_t)0x08000000) 	//扇区0起始地址, 16 Kbytes  
#define ADDR_FLASH_SECTOR_1     ((uint32_t)0x08004000) 	//扇区1起始地址, 16 Kbytes  
#define ADDR_FLASH_SECTOR_2     ((uint32_t)0x08008000) 	//扇区2起始地址, 16 Kbytes  
#define ADDR_FLASH_SECTOR_3     ((uint32_t)0x0800C000) 	//扇区3起始地址, 16 Kbytes  
#define ADDR_FLASH_SECTOR_4     ((uint32_t)0x08010000) 	//扇区4起始地址, 64 Kbytes  
#define ADDR_FLASH_SECTOR_5     ((uint32_t)0x08020000) 	//扇区5起始地址, 128 Kbytes  
#define ADDR_FLASH_SECTOR_6     ((uint32_t)0x08040000) 	//扇区6起始地址, 128 Kbytes  
#define ADDR_FLASH_SECTOR_7     ((uint32_t)0x08060000) 	//扇区7起始地址, 128 Kbytes  
#define ADDR_FLASH_SECTOR_8     ((uint32_t)0x08080000) 	//扇区8起始地址, 128 Kbytes  
#define ADDR_FLASH_SECTOR_9     ((uint32_t)0x080A0000) 	//扇区9起始地址, 128 Kbytes  
#define ADDR_FLASH_SECTOR_10    ((uint32_t)0x080C0000) 	//扇区10起始地址,128 Kbytes  
#define ADDR_FLASH_SECTOR_11    ((uint32_t)0x080E0000) 	//扇区11起始地址,128 Kbytes 

// 获取地址所在的扇区编号（0~11）
uint8_t STMFLASH_GetFlashSector(uint32_t addr);

// 从Flash中读取一个32位字
uint32_t STMFLASH_ReadWord(uint32_t faddr);

// 向Flash写入数据（自动擦除扇区）
// WriteAddress: 起始地址
// data: 要写入的数据指针
// length: 写入字节长度（不是字数！）
void WriteFlashData(uint32_t WriteAddress, uint8_t *data, uint32_t length);

#ifdef __cplusplus
}
#endif

#endif // __STMFLASH_H
