/**
  ******************************************************************************
  * @file    Loader_Src.c
  * @author  HuyHT1 - VF Team
  * @date    2019
  * @brief   This file defines the operations of the external loader for
  *          W25q64 QSPI memory of STM32H743II-HMI_NEO.
  ******************************************************************************
  */
#include "Loader_Src.h"
#include <string.h>

/* Private variables ---------------------------------------------------------*/
QSPI_HandleTypeDef              QSPIHandle;
QSPI_CommandTypeDef             s_command;
QSPI_AutoPollingTypeDef         s_config;

/**
  * @brief  System initialization.
  * @param  None
  * @retval  1      : Operation succeeded
  * @retval  0      : Operation failed
  */
uint8_t Init (void)
{ 
  uint8_t Result = 1;  
  
  SystemInit();
  
  SystemClock_Config();        
 
  /* must be erase to init qspi */
  memset(&QSPIHandle, 0, sizeof(QSPIHandle));
  memset(&s_command, 0, sizeof(s_command));
  memset(&s_config, 0, sizeof(s_config));
  
  QSPIHandle.Instance = QUADSPI;
  HAL_QSPI_DeInit(&QSPIHandle) ; 

  /* QSPI initialization */
  QSPIHandle.Init.ClockPrescaler     = 1; /* QSPI freq = 200 MHz/(1+1) = 100 Mhz */
  QSPIHandle.Init.FifoThreshold      = 32;
  QSPIHandle.Init.SampleShifting     = QSPI_SAMPLE_SHIFTING_HALFCYCLE;
  QSPIHandle.Init.FlashSize          = POSITION_VAL(QSPI_FLASH_SIZE) - 1;
  QSPIHandle.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_5_CYCLE; /* Min 50ns for nonRead commands */
  QSPIHandle.Init.ClockMode          = QSPI_CLOCK_MODE_0;
  QSPIHandle.Init.FlashID            = QSPI_FLASH_ID_1;
  QSPIHandle.Init.DualFlash          = QSPI_DUALFLASH_DISABLE;

  if (HAL_QSPI_Init(&QSPIHandle) != HAL_OK)
  {
    Result = 0;
  }

  /* QSPI memory reset */
  if (QSPI_ResetMemory(&QSPIHandle) != QSPI_OK)
  {
    Result = 0;
  }

  /* QSPI enter mode */
  if (QSPI_EnterMode(&QSPIHandle) != QSPI_OK)
  {
    Result = 0;
  }
  
  /* QSPI check ID */
  if(QSPI_ReadID() != W25Q64)
  {
    Result = 0;
  }
  
  /* Turn on LED if init OK */
  if(Result == 1)
  {
    Display_Init();
    Display_Process_On();    
  }
  
  return Result;
}


/*******************************************************************************
Description :		       Initialisation of ticks counter 														       
Inputs :						     
TickPriority : Set tick priority 
outputs :																				           
NONE
********************************************************************************/
HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority)
{
  return HAL_OK;
}

/*******************************************************************************
Description :		       Delay in mS.														       
Inputs :						     
Delay : Time in mS	
outputs :																				           
NONE
********************************************************************************/
void HAL_Delay(__IO uint32_t Delay)
{
  for(int i=0;i<Delay*8000;i++);
}

void Display_Init(void)
{
  GPIO_InitTypeDef  GPIO_InitStruct;
  
  /* Enable the GPIO_LED Clock */
  LED_PARKING_GPIO_CLK_ENABLE();
  LED_LEFT_GPIO_CLK_ENABLE();

  /* Configure the GPIO_LED pin */  
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  
  GPIO_InitStruct.Pin = LED_PARKING_PIN;
  HAL_GPIO_Init(LED_PARKING_GPIO_PORT, &GPIO_InitStruct); 
  
  GPIO_InitStruct.Pin = LED_LEFT_PIN;
  HAL_GPIO_Init(LED_LEFT_GPIO_PORT, &GPIO_InitStruct);   
}

void Display_Error(uint32_t Cnt, uint32_t ms)
{
  while(Cnt--)
  {
    HAL_GPIO_TogglePin(LED_PARKING_GPIO_PORT, LED_PARKING_PIN);
    HAL_Delay(ms);
  }  
}
    
void Display_Process_On(void)
{
  HAL_GPIO_WritePin(LED_LEFT_GPIO_PORT, LED_LEFT_PIN, GPIO_PIN_SET);
}

void Display_Process_Off(void)
{
  HAL_GPIO_WritePin(LED_LEFT_GPIO_PORT, LED_LEFT_PIN, GPIO_PIN_RESET);
}

void Display_Process_Toggle(void)
{
  HAL_GPIO_TogglePin(LED_LEFT_GPIO_PORT, LED_LEFT_PIN);
}

/**
  * @brief   Program memory.
  * @param   Address: page address
  * @param   Size   : size of data
  * @param   buffer : pointer to data buffer
  * @retval  1      : Operation succeeded
  * @retval  0      : Operation failed
  */
KeepInCompilation int Write (uint32_t Address, uint32_t Size, uint8_t* buffer)
{	  
  return (QSPI_Write(Address, Size, buffer) == QSPI_ERROR) ? 0 : 1;
}

/**
  * @brief   Sector erase.
  * @param   EraseStartAddress :  erase start address
  * @param   EraseEndAddress   :  erase end address
  * @retval  1      : Operation succeeded
  * @retval  0      : Operation failed
  */
KeepInCompilation int SectorErase (uint32_t EraseStartAddress ,uint32_t EraseEndAddress)
{
  uint32_t BlockAddr;
  
  EraseStartAddress = EraseStartAddress -  (EraseStartAddress % QSPI_SECTOR_SIZE);

  while (EraseEndAddress>=EraseStartAddress)
  {
    BlockAddr = EraseStartAddress & 0x0FFFFFFF;
    
    if(QSPI_EraseSector(BlockAddr) != QSPI_OK)
    {
      return 0;
    }
    
    EraseStartAddress += QSPI_SECTOR_SIZE;
    
    Display_Process_Toggle();
  }
  
  return 1;	
}

/*******************************************************************************
 Description :																			
 Read data from the device	 														
 Inputs :																					
 				Address 	: Write location  										
 				Size 		: Length in bytes 										
 				buffer 		: Address where to get the data to write		
 outputs :																				
 				"1" 		: Operation succeeded								
 				"0" 		: Operation failure										
 Note : Not Mandatory                               
********************************************************************************/	
KeepInCompilation int Read (uint32_t Address, uint32_t Size, uint8_t* buffer)
{
  uint32_t i;
  
  if(QSPI_EnableMemoryMappedMode() != QSPI_OK)
  {
    Display_Error(50,100);
    return 0;
  }     
  
  for (i = 0; i < Size; i++)
  { 
    *(uint8_t*)buffer++ = *(__IO uint8_t*)Address;
    Address ++;
  }
  
  return 1;
}

/*******************************************************************************
 Description :																		
 Verify the data 	 														    
 Inputs :																					
 				MemoryAddr 	: Write location  					
 				RAMBufferAddr 	: RAM Address		          
 				Size 		: Length in bytes 								
 outputs :																				
 				"0" 		: Operation succeeded						
 Note : Not Mandatory                             	
********************************************************************************/
KeepInCompilation int Verify (uint32_t MemoryAddr, uint32_t RAMBufferAddr, uint32_t Size)
{
  uint32_t VerifiedData = 0;

  Size*=4;
  
  Display_Process_On();
  
  if(QSPI_EnableMemoryMappedMode() != QSPI_OK)
  {
    Display_Error(100,100);
    return 1;
  }
  
  while (Size > VerifiedData)
  {
    if ( *(__IO uint8_t*)MemoryAddr++ != *((uint8_t*)RAMBufferAddr + VerifiedData))
    {
      Display_Error(50,100);
      return (MemoryAddr + VerifiedData);
    }      
  }  
  
  Display_Process_Off();
  
  return 0;
}
/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 400000000 (CPU Clock)
  *            HCLK(Hz)                       = 200000000 (AXI and AHBs Clock)
  *            AHB Prescaler                  = 2
  *            D1 APB3 Prescaler              = 2 (APB3 Clock  100MHz)
  *            D2 APB1 Prescaler              = 2 (APB1 Clock  100MHz)
  *            D2 APB2 Prescaler              = 2 (APB2 Clock  100MHz)
  *            D3 APB4 Prescaler              = 2 (APB4 Clock  100MHz)
  *            HSE Frequency(Hz)              = 25000000
  *            PLL_M                          = 5
  *            PLL_N                          = 160
  *            PLL_P                          = 2
  *            PLL_Q                          = 4
  *            PLL_R                          = 2
  *            VDD(V)                         = 3.3
  *            Flash Latency(WS)              = 4
  * @param  None
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  
  HAL_StatusTypeDef ret = HAL_OK;
  
  /*!< Supply configuration update enable */
  MODIFY_REG(PWR->CR3, PWR_CR3_SCUEN, 0);

  /* The voltage scaling allows optimizing the power consumption when the device is
     clocked below the maximum system frequency, to update the voltage scaling value
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
  
  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_OFF;
  RCC_OscInitStruct.CSIState = RCC_CSI_OFF;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;

  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 160;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLQ = 4;

  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  ret = HAL_RCC_OscConfig(&RCC_OscInitStruct);
  if(ret != HAL_OK)
  {
    //while(1) { ; }
  }
  
/* Select PLL as system clock source and configure  bus clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_D1PCLK1 | RCC_CLOCKTYPE_PCLK1 | \
                                 RCC_CLOCKTYPE_PCLK2  | RCC_CLOCKTYPE_D3PCLK1);

  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;  
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2; 
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2; 
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2; 
  ret = HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4);
  if(ret != HAL_OK)
  {
    //while(1) { ; }
  } 
  
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_QSPI;
  PeriphClkInitStruct.PLL2.PLL2M = 1;
  PeriphClkInitStruct.PLL2.PLL2N = 100;
  PeriphClkInitStruct.PLL2.PLL2P = 2;
  PeriphClkInitStruct.PLL2.PLL2Q = 2;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_3;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.QspiClockSelection = RCC_QSPICLKSOURCE_D1HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    //Error_Handler();
  }
    
  /*activate CSI clock mondatory for I/O Compensation Cell*/ 
  __HAL_RCC_CSI_ENABLE() ;
    
  /* Enable SYSCFG clock mondatory for I/O Compensation Cell */
  __HAL_RCC_SYSCFG_CLK_ENABLE() ;
  
  /* Enables the I/O Compensation Cell */ 
  HAL_EnableCompensationCell();
      
}

/**
  * @brief  This function send a Write Enable command.
  * @param  hqspi: QSPI handle
  * @retval None
  */
uint8_t QSPI_WriteEnable(QSPI_HandleTypeDef *hqspi)
{
  /* Enable write operations */
  s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  s_command.Instruction       = WRITE_ENABLE_CMD;
  s_command.AddressMode       = QSPI_ADDRESS_NONE;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.DataMode          = QSPI_DATA_NONE;
  s_command.DummyCycles       = 0;
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  if (HAL_QSPI_Command(hqspi, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_ERROR;
  }

  /* Configure automatic polling mode to wait for write enabling */
  s_config.Match           = W25QXX_SR_WREN;
  s_config.Mask            = W25QXX_SR_WREN;
  s_config.MatchMode       = QSPI_MATCH_MODE_AND;
  s_config.StatusBytesSize = 1;
  s_config.Interval        = 0x10;
  s_config.AutomaticStop   = QSPI_AUTOMATIC_STOP_ENABLE;

  s_command.Instruction    = READ_STATUS_REG1_CMD;
  s_command.DataMode       = QSPI_DATA_1_LINE;

  if (HAL_QSPI_AutoPolling(hqspi, &s_command, &s_config, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_ERROR;
  }
  return QSPI_OK;
}

/**
  * @brief   Write QSPI memory page.
  * @param   Address: page address
  * @param   Size   : size of data
  * @param   buffer : pointer to data buffer
  * @retval  None
  */
uint8_t QSPI_Write(uint32_t Address, uint32_t Size , uint8_t* buffer)
{   
  uint32_t end_addr, current_size, current_addr;
  uint32_t Wcnt = 0;
  
  Display_Process_On();
  
  if(Address >= (QSPI_BASE_ADDRESS + QSPI_FLASH_SIZE))
  {
    Display_Error(20,100);
    return QSPI_ERROR;
  }
  
  if(Address >= QSPI_BASE_ADDRESS)
  {    
    Address -= QSPI_BASE_ADDRESS;
  }
  
  if( (Address + Size) > QSPI_FLASH_SIZE)
  {
    Display_Error(20,100); 
    return QSPI_ERROR;
  }

  /* Calculation of the size between the write address and the end of the page */
  current_size = QSPI_PAGE_SIZE - (Address % QSPI_PAGE_SIZE);

  /* Check if the size of the data is less than the remaining place in the page */
  if (current_size > Size)
  {
    current_size = Size;
  }

  /* Initialize the adress variables */
  current_addr = Address;
  end_addr = Address + Size;  

  /* Perform the write page by page */
  do
  {
    /* Enable write operations */
    if (QSPI_WriteEnable(&QSPIHandle) != QSPI_OK)
    {
      return QSPI_ERROR;
    }
    /* Initialize the program command */
    s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
    s_command.Instruction       = QUAD_IN_FAST_PROG_CMD;
    s_command.AddressMode       = QSPI_ADDRESS_1_LINE;
    s_command.AddressSize       = QSPI_ADDRESS_24_BITS;
    s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
    s_command.DataMode          = QSPI_DATA_4_LINES;
    s_command.DummyCycles       = 0;
    s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
    s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
    s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;
    s_command.Address           = current_addr;
    s_command.NbData            = current_size;        

    /* Configure the command */
    if (HAL_QSPI_Command(&QSPIHandle, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    {
      return QSPI_ERROR;
    }

    /* Transmission of the data */
    if (HAL_QSPI_Transmit(&QSPIHandle, buffer, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    {
      return QSPI_ERROR;
    }        
    
    /* Configure automatic polling mode to wait for end of program */
    if (QSPI_AutoPollingMemReady(&QSPIHandle, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != QSPI_OK)
    {
      return QSPI_ERROR;
    }
    
    if((++Wcnt % 64) == 0)
    {
      Display_Process_Toggle();
    }

    /* Update the address and size variables for next page programming */
    current_addr += current_size;
    buffer += current_size;
    current_size = ((current_addr + QSPI_PAGE_SIZE) > end_addr) ? (end_addr - current_addr) : QSPI_PAGE_SIZE;
    
  } while (current_addr < end_addr);
  
  Display_Process_Off();
  
  return QSPI_OK;
}

/**
  * @brief   Erase sector.
  * @param   address :  sector address
  * @retval  1       : Operation succeeded
  * @retval  0       : Operation failed
  */
uint8_t QSPI_EraseSector(uint32_t address)
{
  /* Enable write operations */
  if (QSPI_WriteEnable(&QSPIHandle) != QSPI_OK)
  {
    return QSPI_ERROR;
  }

  /* Initialize the erase command */
  s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  s_command.Instruction       = SUBSECTOR_ERASE_4K_CMD;
  s_command.AddressMode       = QSPI_ADDRESS_1_LINE;
  s_command.AddressSize       = QSPI_ADDRESS_24_BITS;
  s_command.Address           = address;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.DataMode          = QSPI_DATA_NONE;
  s_command.DummyCycles       = 0;
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  /* Send the command */
  if (HAL_QSPI_Command(&QSPIHandle, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_ERROR;
  }

  /* Configure automatic polling mode to wait for end of erase */
  if (QSPI_AutoPollingMemReady(&QSPIHandle, W25QXX_SUBSECTOR_ERASE_MAX_TIME) != QSPI_OK)
  {
    return QSPI_ERROR;
  }
  
  return QSPI_OK;
}


/**     OK
  * @brief  This function reset the QSPI memory.
  * @param  hqspi: QSPI handle
  */
uint8_t QSPI_ResetMemory(QSPI_HandleTypeDef *hqspi)
{
  /* Initialize the reset enable command */
  s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  s_command.AddressMode       = QSPI_ADDRESS_NONE;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.DataMode          = QSPI_DATA_NONE;
  s_command.DummyCycles       = 0;
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  /* Send the command */
  s_command.Instruction       = RESET_ENABLE_CMD;
  if (HAL_QSPI_Command(hqspi, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_ERROR;
  }

  /* Send the reset memory command */
  s_command.Instruction = RESET_MEMORY_CMD;
  if (HAL_QSPI_Command(hqspi, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_ERROR;
  }

  /* Configure automatic polling mode to wait the memory is ready */
  if (QSPI_AutoPollingMemReady(hqspi, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != QSPI_OK)
  {
    return QSPI_ERROR;
  }

  /* Add Delay > 30us */

  return QSPI_OK;
}


/**     OK
  * @brief  This function wrtie the SR2 of the memory and wait the EOP.
  * @param  hqspi: QSPI handle
  */
uint8_t QSPI_EnterMode(QSPI_HandleTypeDef *hqspi)
{
  uint8_t StReg2 = 0;
  /* Initialize the read command */
  s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  s_command.AddressMode       = QSPI_ADDRESS_NONE;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.DataMode          = QSPI_DATA_1_LINE;
  s_command.DummyCycles       = 0;
  s_command.NbData            = 1;
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  /* Configure the command */
  s_command.Instruction = READ_STATUS_REG2_CMD;
  if (HAL_QSPI_Command(&QSPIHandle, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_ERROR;
  }

  /* Set S# timing for Read command: Min 20ns for W25QXX memory */
  MODIFY_REG(QSPIHandle.Instance->DCR, QUADSPI_DCR_CSHT, QSPI_CS_HIGH_TIME_2_CYCLE);

  /* Reception of the data */
  if (HAL_QSPI_Receive(&QSPIHandle, &StReg2, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_ERROR;
  }

  /* Restore S# timing for nonRead commands */
  MODIFY_REG(QSPIHandle.Instance->DCR, QUADSPI_DCR_CSHT, QSPI_CS_HIGH_TIME_5_CYCLE);

  if(StReg2 & W25QXX_SR_QEN)
  {
    return QSPI_OK;
  }

  StReg2 |= W25QXX_SR_QEN;

  /* Enable write operations */
  if (QSPI_WriteEnable(&QSPIHandle) != QSPI_OK)
  {
    return QSPI_ERROR;
  }

  /* Configure the command */
  /* Initialize the read command */
  s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  s_command.AddressMode       = QSPI_ADDRESS_NONE;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.DataMode          = QSPI_DATA_1_LINE;
  s_command.DummyCycles       = 0;
  s_command.NbData            = 1;
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;
  s_command.Instruction = WRITE_STATUS_REG2_CMD;
  if (HAL_QSPI_Command(&QSPIHandle, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_ERROR;
  }

  /* Transmission of the data */
  if(HAL_QSPI_Transmit(&QSPIHandle, &StReg2, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_ERROR;
  }

  /* Configure automatic polling mode to wait for end of program */
  if (QSPI_AutoPollingMemReady(&QSPIHandle, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != QSPI_OK)
  {
    return QSPI_ERROR;
  }

  return QSPI_OK;
}


/**
  * @brief  Configure the QSPI in memory-mapped mode
  * @retval QSPI memory status
  */
uint8_t QSPI_EnableMemoryMappedMode(void)
{
  QSPI_MemoryMappedTypeDef s_mem_mapped_cfg;

  /* Configure the command for the read instruction */
  s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  s_command.Instruction       = QUAD_OUT_FAST_READ_CMD;
  s_command.AddressMode       = QSPI_ADDRESS_1_LINE;
  s_command.AddressSize       = QSPI_ADDRESS_24_BITS;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.DataMode          = QSPI_DATA_4_LINES;
  s_command.DummyCycles       = W25QXX_DUMMY_CYCLES_READ_QUAD;
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  /* Configure the memory mapped mode */
  s_mem_mapped_cfg.TimeOutActivation = QSPI_TIMEOUT_COUNTER_DISABLE;

  if (HAL_QSPI_MemoryMapped(&QSPIHandle, &s_command, &s_mem_mapped_cfg) != HAL_OK)
  {
    return QSPI_ERROR;
  }

  return QSPI_OK;
}

/**     OK
  * @brief  This function read the SR of the memory and wait the EOP.
  * @param  hqspi: QSPI handle
  * @param  Timeout
  */
uint8_t QSPI_AutoPollingMemReady(QSPI_HandleTypeDef *hqspi, uint32_t Timeout)
{
  /* Configure automatic polling mode to wait for memory ready */
  s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  s_command.Instruction       = READ_STATUS_REG1_CMD;
  s_command.AddressMode       = QSPI_ADDRESS_NONE;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.DataMode          = QSPI_DATA_1_LINE;
  s_command.DummyCycles       = 0;
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  s_config.Match           = 0;		/* expected value return*/
  s_config.Mask            = W25QXX_SR_WIP;   /* Mask 0x01 */
  s_config.MatchMode       = QSPI_MATCH_MODE_AND;       /* And */
  s_config.StatusBytesSize = 1;         /* Number of byte return */
  s_config.Interval        = 0x10;
  s_config.AutomaticStop   = QSPI_AUTOMATIC_STOP_ENABLE;

  if (HAL_QSPI_AutoPolling(hqspi, &s_command, &s_config, Timeout) != HAL_OK)
  {
    return QSPI_ERROR;
  }

  return QSPI_OK;
}

uint16_t QSPI_ReadID(void)
{
  uint8_t temp[2];
  uint16_t deviceid;
  QSPI_Send_CMD(READ_MANUFACTURER_CMD, 0, 0, QSPI_INSTRUCTION_1_LINE, QSPI_ADDRESS_1_LINE, QSPI_ADDRESS_24_BITS, QSPI_DATA_1_LINE);
  QSPI_Receive(temp, 2);
  deviceid = (temp[0] << 8) | temp[1];
  return deviceid;
}

uint8_t QSPI_Send_CMD(uint32_t instruction,uint32_t address,uint32_t dummyCycles,uint32_t instructionMode,uint32_t addressMode,uint32_t addressSize,uint32_t dataMode)
{
    QSPI_CommandTypeDef s_command;

    s_command.Instruction              = instruction;
    s_command.Address                  = address;
    s_command.DummyCycles              = dummyCycles;
    s_command.InstructionMode          = instructionMode;
    s_command.AddressMode              = addressMode;
    s_command.AddressSize              = addressSize;
    s_command.DataMode                 = dataMode;
    s_command.SIOOMode                 = QSPI_SIOO_INST_EVERY_CMD;
    s_command.AlternateByteMode        = QSPI_ALTERNATE_BYTES_NONE;
    s_command.DdrMode                  = QSPI_DDR_MODE_DISABLE;
    s_command.DdrHoldHalfCycle         = QSPI_DDR_HHC_ANALOG_DELAY;

    if (HAL_QSPI_Command(&QSPIHandle, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    {
      return QSPI_ERROR;
    }
    return QSPI_OK;
}

uint8_t QSPI_Receive(uint8_t* buf,uint32_t datalen)
{
    QSPIHandle.Instance->DLR = datalen - 1;
    if(HAL_QSPI_Receive(&QSPIHandle,buf,HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    {
      return QSPI_ERROR;
    }
    return QSPI_OK;
}
