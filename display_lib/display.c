#include "display.h"
#include "gpio.h"

int bit_mask(number)
{
	switch(number)
	{
		case 0:
			GPIOA->ODR = 0xC000; //Displaying 0
		break;
		
		case 1:  
			GPIOA->ODR = 0xF900; //Displaying 1
		break;
		
		case 2:
			GPIOA->ODR = 0xA400; //Displaying 2
		break;
		
		case 3:
			GPIOA->ODR = 0xB000; //Displaying 3
		break;
		
		case 4:
			GPIOA->ODR = 0x9900; //Displaying 4
		break;
		
		case 5:
			GPIOA->ODR = 0x9200; //Displaying 5
		break;
		
		case 6:
			GPIOA->ODR = 0x8200; //Displaying 6
		break;
		
		case 7:
			GPIOA->ODR = 0xF800; //Displaying 7
		break;
		
		case 8:
			GPIOA->ODR = 0x8000; //Displaying 8
		break;
		
		case 9:
			GPIOA->ODR = 0x9800; //Displaying 9
		break;
	}
	
}

void Digit(int j)
{
	switch (j)
	{
		case 1:
		{
			HAL_GPIO_WritePin(GPIOC, T1_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC, T2_Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC, T3_Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC, T4_Pin,GPIO_PIN_SET);
			//HAL_GPIO_WritePin(GPIOE, DP_Pin,GPIO_PIN_RESET);
			break;
		}
		case 2:
		{
			HAL_GPIO_WritePin(GPIOC, T1_Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC, T2_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC, T3_Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC, T4_Pin,GPIO_PIN_SET);
			//HAL_GPIO_WritePin(GPIOE, DP_Pin,GPIO_PIN_RESET);
			break;
		}
		case 3:
		{
			HAL_GPIO_WritePin(GPIOC, T1_Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC, T2_Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC, T3_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC, T4_Pin,GPIO_PIN_SET);
			//HAL_GPIO_WritePin(GPIOE, DP_Pin,GPIO_PIN_RESET);
			break;
		}
		case 4:
		{
			HAL_GPIO_WritePin(GPIOC, T1_Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC, T2_Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC, T3_Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC, T4_Pin,GPIO_PIN_RESET);
			//HAL_GPIO_WritePin(GPIOE, DP_Pin,GPIO_PIN_RESET);
			break;
		}
		case 5:
		{
			HAL_GPIO_WritePin(GPIOC, T1_Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC, T2_Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC, T3_Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC, T4_Pin,GPIO_PIN_SET);
			//HAL_GPIO_WritePin(GPIOE, DP_Pin,GPIO_PIN_SET);	
			break;
		}
	}
}



