/**
  ******************************************************************************
  * File Name          : USART.c
  * Description        : This file provides code for the configuration
  *                      of the USART instances.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2015 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "usart.h"

//#include "gpio.h"
//#include "dma.h"
#include "Switchbot.h"
#include <stdarg.h>

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

#define PACKET_SIZE       3
#define UART_TIMEOUT		 270 					// milliseconds

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

DMA_HandleTypeDef hdma_usart2_tx;
uint8_t buffer[20] = {0};
uint32_t timestamp1 = 0;
uint32_t timestamp6 = 0;
uint8_t  RX3Data[PACKET_SIZE] = {0};						//UART0BytesSent is a value set in Robot Values that signifies the number of bytes that will be sent by UART0
uint8_t  RX1Data[PACKET_SIZE] = {0};	


uint32_t millis(void) {
  return HAL_GetTick();
}

void uart_put(uint8_t uart_port, uint8_t ch)
{
	
}	


void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_StatusTypeDef status =  HAL_UART_Init(&huart1);
  HAL_UART_Receive_IT(&huart1, RX1Data, PACKET_SIZE);

  LOG("uart 1 status: %d \n", status);
}

/* USART3 init function */
void MX_USART3_UART_Init(void)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_StatusTypeDef status =  HAL_UART_Init(&huart3);
  HAL_UART_Receive_IT(&huart3, RX3Data, PACKET_SIZE);
  
  LOG("uart 3 status: %d \n", status);
}

/* USER CODE BEGIN 1 */

/**
* @brief This function handles USART1 global interrupt / USART1 wake-up interrupt through EXTI line 25.
*/
void USART1_IRQHandler(void)
{
   /* USER CODE BEGIN USART1_IRQn 0 */
 
  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
	
  /* USER CODE BEGIN USART1_IRQn 1 */

	
  /* USER CODE END USART1_IRQn 1 */

}

/**
* @brief This function handles USART2 global interrupt.
*/
void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */
 
  /* USER CODE END USART2_IRQn 0 */
  
	HAL_UART_IRQHandler(&huart3);
  /* USER CODE BEGIN USART2_IRQn 1 */

	
  /* USER CODE END USART2_IRQn 1 */
}

void reset_IT(UART_HandleTypeDef *UartHandle)
{

	if (UartHandle == &huart1)
	{
		HAL_UART_Receive_IT(UartHandle, RX1Data, PACKET_SIZE);
		timestamp1 = millis();
//		LOG("uart1 %d %d %d %d \n", RX1Data[0], RX1Data[1], RX1Data[2], timestamp1);
	}
	else
	{
		HAL_UART_Receive_IT(UartHandle, RX3Data, PACKET_SIZE);
		timestamp6 = millis();
//		LOG("uart6 %d %d %d ts: %d \n", RX3Data[0], RX3Data[1],				RX3Data[2], timestamp6);
	}

}

/**
  * @brief  Rx Transfer completed callback
  * @param  UartHandle: UART handle
  * @note   This example shows a simple way to report end of DMA Rx transfer, and 
  *         you can add your own implementation.
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
  
   //LOG("Rx completed ... \n");
  reset_IT(UartHandle);
	 
}

/**
  * @brief  UART error callbacks
  * @param  UartHandle: UART handle
  * @note   This example shows a simple way to report transfer error, and you can
  *         add your own implementation.
  * @retval None
  */
 void HAL_UART_ErrorCallback(UART_HandleTypeDef *UartHandle)
{
	 
	 //LOG4(" Error callback ... error code : %d ", UartHandle->ErrorCode);
	   reset_IT(UartHandle);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle)
{
   LOG("Tx completed ... \n");
  
}




/////////////////////////////////////////////////////////////////////////////////////////////////////////
// parse UART0 buffer and set command variables

int parseUARTbuffer(float joy[])
{
	int button = 0;
  float m = 0;
	joy[0] = 0;
	joy[1] = 0;
	joy[2] = 0;

// Handling UART3, commands from the BLE
   m = (millis() - timestamp6);
	if( m < 0) {  // bug fix where the millis() restart counting after ~ 3min, maybe we should change millis() implementation ?	          
	  RX3Data[0] = 0;
	} else if ( m < UART_TIMEOUT)
	{
		switch(RX3Data[0])
		{
			
	
			case 0x50: // push to talk has been pressed
				LOG("Push button ... uart6 \n");
				RX3Data[0] = 0;
				break;
			case ESTOP: // emergency stop
				button = ESTOP;
				RX3Data[0] = 0;
				break;
			case CLEAR_ESTOP: // reset emergency stop
				button = CLEAR_ESTOP;
				RX3Data[0] = 0;
				break;
			case DRIVE: // drive, any mode
		   	LOG("drive 0x81\t%3d\t%3d \n", RX3Data[1], RX3Data[2]);
			  // LOG("drive 0x78 \n");
				// 0 is stop, 01-32 is forward, 33-64 is backwards
				// 0 is straight, 65-96 is right, 97-128 is left
/*				if (RX3Data[1] > 0) // not stopped
				{
					if (RX3Data[1] < 33) // forward
						joy[1] = ((float) RX3Data[1]) / 32.0;
					else if (RX3Data[1] < 65)// backward
						joy[1] = -((float) (RX3Data[1] - 32)) / 32.0;
				}
				if (RX3Data[2] > 64) // not straight
				{
					if (RX3Data[2] < 97) // right
						joy[0] = ((float) (RX3Data[2] - 64)) / 32.0;
					else if (RX3Data[2] < 129)// left
						joy[0] = -((float) (RX3Data[2] - 96)) / 32.0;
				}
*/
				break;
				
			case NOTF_GET_NEXT_BEACON:
			case NOTF_DP_CLOSEST_BEACON:{
				 uint8_t pData[3] = {RX3Data[0], RX3Data[1], 0x00};
				 HAL_UART_Transmit(&huart1, pData, 3, 1000);
			     RX3Data[0] = 0;      // we reset the command data to avoid resending it again
			     }
			   break;
		       case  RGB_CTRL_COMMAND: 
		    	   HAL_UART_Transmit(&huart1, RX3Data, 3, 1000);
			   RX3Data[0] = 0; 
		    //   LOG("echo test ... \r\n");
		       break;
			 default:	break;
		}
	} 
	
	
	// Handling UART1, commands from android board
  m = (millis() - timestamp1); //;
	
	if( m < 0) {  // bug fix where the millis() restart counting        
	  RX1Data[0] = 0;
	} else if ( m < UART_TIMEOUT)
	{
	//	HAL_UART_Transmit(&huart1, RX1Data, 3, 1000);
		//LOG("-- %04x %04x %04x \r\n", RX1Data[0], RX1Data[1], RX1Data[2]);
		switch(RX1Data[0])
		{
			
			case 0x50: // push to talk has been pressed
				LOG("Push button ... uart1  \n");RX1Data[0] = 0;
        	                uint8_t test[3] = { RGB_CTRL_COMMAND ,0x00 , 0x00};
        	                HAL_UART_Transmit(&huart3, test, 3, 1000);
				break;
			case ESTOP: // emergency stop
				button = ESTOP; RX1Data[0] = 0;
				break;
			case CLEAR_ESTOP: // reset emergency stop
		//	/	button = CLEAR_ESTOP;RX1Data[0] = 0;
				break;
			case DRIVE: // drive, any mode
				
		   	LOG("drive 0x81 \r\n");
				// 0 is stop, 01-32 is forward, 33-64 is backwards
				// 0 is straight, 65-96 is right, 97-128 is left
			/*
				if (RX1Data[1] > 0) // not stopped
				{
					if (RX1Data[1] < 33) // forward
						joy[1] = ((float) RX1Data[1]) / 32.0;
					else if (RX1Data[1] < 65)// backward
						joy[1] = -((float) (RX1Data[1] - 32)) / 32.0;
				}
				if (RX1Data[2] > 64) // not straight
				{
					if (RX1Data[2] < 97) // right
						joy[0] = ((float) (RX1Data[2] - 64)) / 32.0;
					else if (RX1Data[2] < 129)// left
						joy[0] = -((float) (RX1Data[2] - 96)) / 32.0;
				}
		*/		
				break;
				
		  case DP_STOP:
      case DP_CHANGE_RANGE:
      case NOTF_DP_TARGET_REACHED:{
                 uint8_t pData[3] = {RX1Data[0], 0x00, 0x00};
        	     HAL_UART_Transmit(&huart3, pData, 3, 1000);
			     RX1Data[0] = 0;      // we reset the command data to avoid resending it again
			     }
    			 break;
	
			
			case RGB_CTRL_COMMAND:
				
		    HAL_UART_Transmit(&huart3, RX1Data, 3, 1000);
	   // 		   HAL_UART_Transmit(&huart1, RX1Data, 3, 1000);
		 	    LOG(" uart1 %d %d %d \r\n", RX1Data[0], RX1Data[1], RX1Data[2]);  
			    button = RGB_CTRL_COMMAND;
			    joy[0] = RX1Data[1];
			    RX1Data[0] = 0;      // we reset the command data to avoid resending it again

				break;

                   case DP_GET_CLOSEST_BEACON:
		      case DP_GOTO_BEACON: 
        	     HAL_UART_Transmit(&huart3, RX1Data, 3, 1000);
                 RX1Data[0] = 0;      // we reset the command data to avoid resending it again
//	                   button = (int)DRIVE;
                LOG("Drive ... \r\n"); 					
  		           break; 
		     
			default:break;
		}
	}
	

	return button;
}



/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
