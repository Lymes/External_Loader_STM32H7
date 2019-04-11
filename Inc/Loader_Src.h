/**
  ******************************************************************************
  * @file    Loader_Src.h
  * @author  MCD Tools Team
  * @date    October-2015
  * @brief   Loader Header file.
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LOADER_SRC_H
#define __LOADER_SRC_H

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

#if defined(__CC_ARM)
extern uint32_t Load$$QSPI$$Base;
extern uint32_t Load$$QSPI$$Length;
#elif defined(__ICCARM__)
#pragma section =".qspi"
#pragma section =".qspi_init"
#elif defined(__GNUC__)
extern uint32_t _qspi_init_base;
extern uint32_t _qspi_init_length;
#endif

#ifdef __ICCARM__                //IAR
#define KeepInCompilation __root 
#elif __CC_ARM                   //MDK-ARM
#define KeepInCompilation __attribute__((used))
#elseif TASKING                  //TrueStudio
#define KeepInCompilation __attribute__((used))
#endif

#define StartRamAddress          0x20000000
#define EndRamAddress            0x20080000

#define QSPI_ADDRESS_WIDTH QSPI_ADDRESS_32_BITS

/* Definition for QSPI clock resources */
#define QSPI_CLK_ENABLE()           __HAL_RCC_QSPI_CLK_ENABLE()
#define QSPI_CLK_DISABLE()          __HAL_RCC_QSPI_CLK_DISABLE()

#define QSPI_CS_GPIO_CLK_ENABLE()   __HAL_RCC_GPIOB_CLK_ENABLE()
#define QSPI_CS_GPIO_CLK_DISABLE()  __HAL_RCC_GPIOB_CLK_DISABLE()
#define QSPI_CLK_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOB_CLK_ENABLE()
#define QSPI_CLK_GPIO_CLK_DISABLE() __HAL_RCC_GPIOB_CLK_DISABLE()

#define QSPI_D0_GPIO_CLK_ENABLE()   __HAL_RCC_GPIOD_CLK_ENABLE()
#define QSPI_D1_GPIO_CLK_ENABLE()   __HAL_RCC_GPIOD_CLK_ENABLE()
#define QSPI_D2_GPIO_CLK_ENABLE()   __HAL_RCC_GPIOE_CLK_ENABLE()
#define QSPI_D3_GPIO_CLK_ENABLE()   __HAL_RCC_GPIOD_CLK_ENABLE()

#define QSPI_D0_GPIO_CLK_DISABLE()   __HAL_RCC_GPIOD_CLK_DISABLE()
#define QSPI_D1_GPIO_CLK_DISABLE()   __HAL_RCC_GPIOD_CLK_DISABLE()
#define QSPI_D2_GPIO_CLK_DISABLE()   __HAL_RCC_GPIOE_CLK_DISABLE()
#define QSPI_D3_GPIO_CLK_DISABLE()   __HAL_RCC_GPIOD_CLK_DISABLE()

#define QSPI_FORCE_RESET()          __HAL_RCC_QSPI_FORCE_RESET()
#define QSPI_RELEASE_RESET()        __HAL_RCC_QSPI_RELEASE_RESET()

/* Definition for QSPI Pins */
#define QSPI_CS_PIN                GPIO_PIN_6
#define QSPI_CS_GPIO_PORT          GPIOB
#define QSPI_CLK_PIN               GPIO_PIN_2
#define QSPI_CLK_GPIO_PORT         GPIOB
#define QSPI_D0_PIN                GPIO_PIN_11
#define QSPI_D0_GPIO_PORT          GPIOD
#define QSPI_D1_PIN                GPIO_PIN_12
#define QSPI_D1_GPIO_PORT          GPIOD
#define QSPI_D2_PIN                GPIO_PIN_2
#define QSPI_D2_GPIO_PORT          GPIOE
#define QSPI_D3_PIN                GPIO_PIN_13
#define QSPI_D3_GPIO_PORT          GPIOD

   
/**
  * @brief LED_PARKING
  */
#define LED_PARKING_GPIO_PORT                   GPIOG
#define LED_PARKING_GPIO_CLK_ENABLE()           __HAL_RCC_GPIOG_CLK_ENABLE()
#define LED_PARKING_GPIO_CLK_DISABLE()          __HAL_RCC_GPIOG_CLK_DISABLE()
#define LED_PARKING_PIN                         GPIO_PIN_3

/**
  * @brief LED_LEFT
  */
#define LED_LEFT_GPIO_PORT                   GPIOG
#define LED_LEFT_GPIO_CLK_ENABLE()           __HAL_RCC_GPIOG_CLK_ENABLE()
#define LED_LEFT_GPIO_CLK_DISABLE()          __HAL_RCC_GPIOG_CLK_DISABLE()
#define LED_LEFT_PIN                         GPIO_PIN_2

/**
  * @brief LED_HIGH_BEAM
  */
#define LED_HIGH_BEAM_GPIO_PORT                   GPIOH
#define LED_HIGH_BEAM_GPIO_CLK_ENABLE()           __HAL_RCC_GPIOH_CLK_ENABLE()
#define LED_HIGH_BEAM_GPIO_CLK_DISABLE()          __HAL_RCC_GPIOH_CLK_DISABLE()
#define LED_HIGH_BEAM_PIN                         GPIO_PIN_6
   
/** 
  * @brief  W25QXX Configuration
  */  
#define W25QXX_FLASH_SIZE                  (8*0x100000)  /* 8 * 8 MBits => 8MBytes */
#define W25QXX_SECTOR_SIZE                 0x10000   /* 256 sectors of 64KBytes */
#define W25QXX_SUBSECTOR_SIZE              0x1000    /* 4096 subsectors of 4kBytes */
#define W25QXX_PAGE_SIZE                   0x100     /* 65536 pages of 256 bytes */

#define W25QXX_DUMMY_CYCLES_READ           8
#define W25QXX_DUMMY_CYCLES_READ_QUAD      8

#define W25QXX_BULK_ERASE_MAX_TIME         250000
#define W25QXX_SECTOR_ERASE_MAX_TIME       3000
#define W25QXX_SUBSECTOR_ERASE_MAX_TIME    800

/* JEDEC Manufacturer ID */
#define MF_ID                   (0xEF)
/* JEDEC Device ID: Memory type and Capacity */
#define MTC_W25Q16_BV_CL_CV     (0x4015) /* W25Q16BV W25Q16CL W25Q16CV  */
#define MTC_W25Q16_DW           (0x6015) /* W25Q16DW  */
#define MTC_W25Q32_BV           (0x4016) /* W25Q32BV */
#define MTC_W25Q32_DW           (0x6016) /* W25Q32DW */
#define MTC_W25Q64_BV_CV        (0x4017) /* W25Q64BV W25Q64CV */
#define MTC_W25Q64_DW           (0x4017) /* W25Q64DW */
#define MTC_W25Q128_BV          (0x4018) /* W25Q128BV */
 
/* READ_MANUFACTURER_CMD */    
#define W25Q80 	                0XEF13 	
#define W25Q16 	                0XEF14
#define W25Q32 	                0XEF15
#define W25Q64 	                0XEF16
#define W25Q128	                0XEF17
#define W25Q256                 0XEF18   

/** 
  * @brief  W25QXX Commands, W25Q16JV
  */  
/* Reset Operations */
#define RESET_ENABLE_CMD                     0x66
#define RESET_MEMORY_CMD                     0x99

/* Identification Operations */
#define READ_JEDEC_ID_CMD                    0x9F
#define READ_UNIQUE_ID_CMD                   0x4B
#define READ_SERIAL_FLASH_DISCO_PARAM_CMD    0x5A
#define READ_MANUFACTURER_CMD    	     0x90
#define READ_MANUFACTURER_DUAL_CMD    	     0x92
#define READ_MANUFACTURER_QUAD_CMD    	     0x94
#define READ_SERIAL_FLASH_DISCO_PARAM_CMD    0x5A
#define READ_SERIAL_FLASH_DISCO_PARAM_CMD    0x5A
#define READ_SERIAL_FLASH_DISCO_PARAM_CMD    0x5A

/* Read Operations */
#define READ_CMD                             0x03
#define FAST_READ_CMD                        0x0B
#define DUAL_OUT_FAST_READ_CMD               0x3B
#define QUAD_OUT_FAST_READ_CMD               0x6B
#define DUAL_INOUT_FAST_READ_CMD             0xBB
#define QUAD_INOUT_FAST_READ_CMD             0xEB

/* Write Operations */
#define WRITE_ENABLE_CMD                     0x06
#define WRITE_DISABLE_CMD                    0x04

/* Register Operations */
#define READ_STATUS_REG1_CMD                 0x05
#define WRITE_STATUS_REG1_CMD                0x01
#define READ_STATUS_REG2_CMD                 0x35
#define WRITE_STATUS_REG2_CMD                0x31
#define READ_STATUS_REG3_CMD                 0x15
#define WRITE_STATUS_REG3_CMD                0x11

/* Program Operations */
#define PAGE_PROG_CMD                        0x02
#define QUAD_IN_FAST_PROG_CMD                0x32

/* Erase Operations */
#define SUBSECTOR_ERASE_4K_CMD               0x20
#define SECTOR_ERASE_32K_CMD                 0x52
#define SECTOR_ERASE_64K_CMD                 0xD8
#define CHIP_ERASE_CMD                       0xC7

#define PROG_ERASE_RESUME_CMD                0x7A
#define PROG_ERASE_SUSPEND_CMD               0x75

/* Security Operations */
#define READ_SECURITY_CMD                    0x48
#define PROG_SECURITY_CMD                    0x42

/** 
  * @brief  W25QXX Registers
  */ 
/* Status Register */
#define W25QXX_SR_WIP                      ((uint8_t)0x01)    /*!< Write in progress */
#define W25QXX_SR_WREN                     ((uint8_t)0x02)    /*!< Write enable latch */
#define W25QXX_SR_BLOCKPR                  ((uint8_t)0x5C)    /*!< Block protected against program and erase operations */
#define W25QXX_SR_PRBOTTOM                 ((uint8_t)0x20)    /*!< Protected memory area defined by BLOCKPR starts from top or bottom */
#define W25QXX_SR_SRWREN                   ((uint8_t)0x80)    /*!< Status register write enable/disable */

#define W25QXX_SR_QEN                      ((uint8_t)0x02)    /*!< Quad enable */

#define QSPI_OK         0
#define QSPI_ERROR      1
    
/* Size of the flash */
#define QSPI_FLASH_SIZE                 W25QXX_FLASH_SIZE
#define QSPI_PAGE_SIZE                  W25QXX_PAGE_SIZE
#define QSPI_SECTOR_SIZE                W25QXX_SUBSECTOR_SIZE

/* QSPI Base Address */
#define QSPI_BASE_ADDRESS               0x90000000    

/* Private function prototypes -----------------------------------------------*/
//All system initialisation
uint8_t Init (void);
void    SystemClock_Config(void);
HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority);
void    HAL_Delay(__IO uint32_t Delay);
void    Display_Init(void);
void    Display_Error(uint32_t Cnt, uint32_t ms);    
void    Display_Process_On(void);
void    Display_Process_Off(void);
void    Display_Process_Toggle(void);

uint8_t QSPI_EraseSector(uint32_t address);
uint8_t QSPI_Write(uint32_t Address, uint32_t Size , uint8_t* buffer);
uint8_t QSPI_WriteEnable(QSPI_HandleTypeDef *hqspi);
uint8_t QSPI_EnterMode(QSPI_HandleTypeDef *hqspi);

uint8_t QSPI_AutoPollingMemReady(QSPI_HandleTypeDef *hqspi, uint32_t Timeout);
uint8_t QSPI_ResetMemory(QSPI_HandleTypeDef *hqspi);

uint8_t QSPI_EnableMemoryMappedMode(void);
uint16_t QSPI_ReadID(void);
uint8_t QSPI_Send_CMD(uint32_t instruction,uint32_t address,uint32_t dummyCycles,uint32_t instructionMode,uint32_t addressMode,uint32_t addressSize,uint32_t dataMode);
uint8_t QSPI_Receive(uint8_t* buf,uint32_t datalen);

//QSPI operation functions
KeepInCompilation int Write (uint32_t Address, uint32_t Size, uint8_t* buffer);
KeepInCompilation int Read (uint32_t Address, uint32_t Size, uint8_t* buffer);
KeepInCompilation int Verify (uint32_t MemoryAddr, uint32_t RAMBufferAddr, uint32_t Size);
KeepInCompilation int SectorErase (uint32_t EraseStartAddress ,uint32_t EraseEndAddress);

#endif /* __LOADER_SRC_H */
