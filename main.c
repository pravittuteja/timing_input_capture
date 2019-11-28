#include "init.h"

char* retry_msg = "Input waveform is not detected, would you like to try again?(Y/N)";
char* terminate_msg = "Program terminated.\r\n";
char y_n;



int main(void){
	/*Initializes system clock to 80Mhz*/
	System_Clock_Init(); 
	/*Initializes the GPIO Pins*/
	gpio_init();
	/*Initializes UART2*/
	UART2_Init();
	/*Initializes time 4 channel 1 in INPUT CAPTURE mode*/
	TIM4_init();
	
	/*Power On Self Test UI*/
	while(!POST()){
		USART_Write(USART2, (uint8_t*)retry_msg, strlen(retry_msg));
		y_n = (uint8_t)USART_Read(USART2);
		USART_Write(USART2, (uint8_t*)&y_n, 1);
		USART_Write(USART2, (uint8_t *)"\r\n", 2);
		if (y_n == 0x59 || y_n == 0x79){
			POST();
		}
		else if (y_n == 0x4E || y_n == 0x6E){
			USART_Write(USART2, (uint8_t*)terminate_msg, strlen(terminate_msg));
			return 0;
		}
	}
	
	/*Infinite loop to get user input -> Capture the timer values -> Process and transmit the data*/
	while(1){
		get_UI_limit();
		capture();
		process();
	}
}



