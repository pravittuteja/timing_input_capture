#include "init.h"
char uart_data[32];
char ui_buffer1[BufferSize];
char ui_buffer2[BufferSize];
char *ui_buffer2_ptr = ui_buffer2;
char* welcome_msg = "Power On Self Test\r\n";
char* end_msg = "Power On Self Test Successful\r\n";
char* fail_msg = "POST FAILED\r\n";
char str[] = "Please Enter the lower limit: ";
char error_msg1[] = "\r\nPlease Enter a valid number!";
char error_msg2[] = "\r\nPlease Enter a value within range of 50 - 9950";

volatile uint32_t overflow = 0;
volatile uint32_t capture_new = 0;
volatile uint32_t capture_old = 0;
volatile uint32_t tim_us = 0;
volatile char captured = 0;
volatile uint16_t count_g =0;
int lower_limit = 0;
int upper_limit = 0;
int time_array[1002];
int array_index = 0;
int freq[1002];
int out_of_range=0;

void System_Clock_Init(void){
	
	uint32_t HSITrim;

	// To correctly read data from FLASH memory, the number of wait states (LATENCY)
  // must be correctly programmed according to the frequency of the CPU clock
  // (HCLK) and the supply voltage of the device.		
	FLASH->ACR &= ~FLASH_ACR_LATENCY;
	FLASH->ACR |=  FLASH_ACR_LATENCY_2WS;
		
	// Enable the Internal High Speed oscillator (HSI
	RCC->CR |= RCC_CR_HSION;
	while((RCC->CR & RCC_CR_HSIRDY) == 0);
	// Adjusts the Internal High Speed oscillator (HSI) calibration value
	// RC oscillator frequencies are factory calibrated by ST for 1 % accuracy at 25oC
	// After reset, the factory calibration value is loaded in HSICAL[7:0] of RCC_ICSCR	
	HSITrim = 16; // user-programmable trimming value that is added to HSICAL[7:0] in ICSCR.
	RCC->ICSCR &= ~RCC_ICSCR_HSITRIM;
	RCC->ICSCR |= HSITrim << 24;
	
	RCC->CR    &= ~RCC_CR_PLLON; 
	while((RCC->CR & RCC_CR_PLLRDY) == RCC_CR_PLLRDY);
	
	// Select clock source to PLL
	RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLSRC;
	RCC->PLLCFGR |= RCC_PLLCFGR_PLLSRC_HSI; // 00 = No clock, 01 = MSI, 10 = HSI, 11 = HSE
	
	// Make PLL as 80 MHz
	// f(VCO clock) = f(PLL clock input) * (PLLN / PLLM) = 16MHz * 20/2 = 160 MHz
	// f(PLL_R) = f(VCO clock) / PLLR = 160MHz/2 = 80MHz
	RCC->PLLCFGR = (RCC->PLLCFGR & ~RCC_PLLCFGR_PLLN) | 20U << 8;
	RCC->PLLCFGR = (RCC->PLLCFGR & ~RCC_PLLCFGR_PLLM) | 1U << 4; // 000: PLLM = 1, 001: PLLM = 2, 010: PLLM = 3, 011: PLLM = 4, 100: PLLM = 5, 101: PLLM = 6, 110: PLLM = 7, 111: PLLM = 8

	RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLR;  // 00: PLLR = 2, 01: PLLR = 4, 10: PLLR = 6, 11: PLLR = 8	
	RCC->PLLCFGR |= RCC_PLLCFGR_PLLREN; // Enable Main PLL PLLCLK output 

	RCC->CR   |= RCC_CR_PLLON; 
	while((RCC->CR & RCC_CR_PLLRDY) == 0);
	
	// Select PLL selected as system clock
	RCC->CFGR &= ~RCC_CFGR_SW;
	RCC->CFGR |= RCC_CFGR_SW_PLL; // 00: MSI, 01:HSI, 10: HSE, 11: PLL
	
	// Wait until System Clock has been selected
	while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);
	
	// The maximum frequency of the AHB, the APB1 and the APB2 domains is 80 MHz.
	RCC->CFGR &= ~RCC_CFGR_HPRE;  // AHB prescaler = 1; SYSCLK not divided
	RCC->CFGR &= ~RCC_CFGR_PPRE1; // APB high-speed prescaler (APB1) = 1, HCLK not divided
	RCC->CFGR &= ~RCC_CFGR_PPRE2; // APB high-speed prescaler (APB2) = 1, HCLK not divided
	
	// RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLM;
	// RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLN;
	// RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLP; 
	// RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLQ;	
	// RCC->PLLCFGR |= RCC_PLLCFGR_PLLPEN; // Enable Main PLL PLLSAI3CLK output enable
	// RCC->PLLCFGR |= RCC_PLLCFGR_PLLQEN; // Enable Main PLL PLL48M1CLK output enable
	
	RCC->CR &= ~RCC_CR_PLLSAI1ON;  // SAI1 PLL enable
	while ( (RCC->CR & RCC_CR_PLLSAI1ON) == RCC_CR_PLLSAI1ON );
	
	// Configure and enable PLLSAI1 clock to generate 11.294MHz 
	// 8 MHz * 24 / 17 = 11.294MHz
	// f(VCOSAI1 clock) = f(PLL clock input) *  (PLLSAI1N / PLLM)
	// PLLSAI1CLK: f(PLLSAI1_P) = f(VCOSAI1 clock) / PLLSAI1P
	// PLLUSB2CLK: f(PLLSAI1_Q) = f(VCOSAI1 clock) / PLLSAI1Q
	// PLLADC1CLK: f(PLLSAI1_R) = f(VCOSAI1 clock) / PLLSAI1R
	RCC->PLLSAI1CFGR &= ~RCC_PLLSAI1CFGR_PLLSAI1N;
	RCC->PLLSAI1CFGR |= 24U<<8;
	
	// SAI1PLL division factor for PLLSAI1CLK
	// 0: PLLSAI1P = 7, 1: PLLSAI1P = 17
	RCC->PLLSAI1CFGR |= RCC_PLLSAI1CFGR_PLLSAI1P;
	RCC->PLLSAI1CFGR |= RCC_PLLSAI1CFGR_PLLSAI1PEN;
	
	// SAI1PLL division factor for PLL48M2CLK (48 MHz clock)
	// RCC->PLLSAI1CFGR &= ~RCC_PLLSAI1CFGR_PLLSAI1Q;
	// RCC->PLLSAI1CFGR |= U<<21;
	// RCC->PLLSAI1CFGR |= RCC_PLLSAI1CFGR_PLLSAI1QEN;
	
	// PLLSAI1 division factor for PLLADC1CLK (ADC clock)
	// 00: PLLSAI1R = 2, 01: PLLSAI1R = 4, 10: PLLSAI1R = 6, 11: PLLSAI1R = 8
	// RCC->PLLSAI1CFGR &= ~RCC_PLLSAI1CFGR_PLLSAI1R; 
	// RCC->PLLSAI1CFGR |= U<<25;
	// RCC->PLLSAI1CFGR |= RCC_PLLSAI1CFGR_PLLSAI1REN;
	
	RCC->CR |= RCC_CR_PLLSAI1ON;  // SAI1 PLL enable
	while ( (RCC->CR & RCC_CR_PLLSAI1ON) == 0);
	
	// SAI1 clock source selection
	// 00: PLLSAI1 "P" clock (PLLSAI1CLK) selected as SAI1 clock
	// 01: PLLSAI2 "P" clock (PLLSAI2CLK) selected as SAI1 clock
	// 10: PLL "P" clock (PLLSAI3CLK) selected as SAI1 clock
	// 11: External input SAI1_EXTCLK selected as SAI1 clock	
	RCC->CCIPR &= ~RCC_CCIPR_SAI1SEL;

	RCC->APB2ENR |= RCC_APB2ENR_SAI1EN;
}

void gpio_init(void){
  // Enable the clock to GPIO Port E & B
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOEEN; 
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN; 		

	// MODE: 00: Input mode, 01: General purpose output mode
  //       10: Alternate function mode, 11: Analog mode (reset state)
  GPIOE->MODER &= ~(0x03<<(8*2)) ;   
  GPIOE->MODER |= (1UL<<8*2);
	GPIOB->MODER &= ~(0x03<<(2*2)) ;  
  GPIOB->MODER |= (1UL<<2*2);

}

void UART2_Init(void) {
	// Enable the clock of USART 1 & 2
	RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN;  // Enable USART 2 clock		
	
	// Select the USART1 clock source
	// 00: PCLK selected as USART2 clock
	// 01: System clock (SYSCLK) selected as USART2 clock
	// 10: HSI16 clock selected as USART2 clock
	// 11: LSE clock selected as USART2 clock
	RCC->CCIPR &= ~RCC_CCIPR_USART2SEL;
	RCC->CCIPR |=  RCC_CCIPR_USART2SEL_0;
	
	UART2_GPIO_Init();
	USART_Init(USART2);
	
	//NVIC_SetPriority(USART2_IRQn, 0);			// Set Priority to 1
	//NVIC_EnableIRQ(USART2_IRQn);					// Enable interrupt of USART1 peripheral
}


void UART2_GPIO_Init(void) {
	
	// Enable the peripheral clock of GPIO Port
	RCC->AHB2ENR |=   RCC_AHB2ENR_GPIODEN;
	
	// ********************** USART 2 ***************************
	// PD5 = USART2_TX (AF7)
	// PD6 = USART2_RX (AF7)
	// Alternate function, High Speed, Push pull, Pull up
	// **********************************************************
	// Input(00), Output(01), AlterFunc(10), Analog(11)
	GPIOD->MODER   &= ~(0xF << (2*5));	// Clear bits
	GPIOD->MODER   |=   0xA << (2*5);      		
	GPIOD->AFR[0]  |=   0x77<< (4*5);       	
	// GPIO Speed: Low speed (00), Medium speed (01), Fast speed (10), High speed (11)
	GPIOD->OSPEEDR |=   0xF<<(2*5); 					 	
	// GPIO Push-Pull: No pull-up, pull-down (00), Pull-up (01), Pull-down (10), Reserved (11)
	GPIOD->PUPDR   &= ~(0xF<<(2*5));
	GPIOD->PUPDR   |=   0x5<<(2*5);    				
	// GPIO Output Type: Output push-pull (0, reset), Output open drain (1) 
	GPIOD->OTYPER  &=  ~(0x3<<5) ;       	
}


void USART_Init (USART_TypeDef * USARTx) {
	// Default setting: 
	//     No hardware flow control, 8 data bits, no parity, 1 start bit and 1 stop bit		
	USARTx->CR1 &= ~USART_CR1_UE;  // Disable USART
	
	// Configure word length to 8 bit
	USARTx->CR1 &= ~USART_CR1_M;   // M: 00 = 8 data bits, 01 = 9 data bits, 10 = 7 data bits
	
	// Configure oversampling mode: Oversampling by 16 
	USARTx->CR1 &= ~USART_CR1_OVER8;  // 0 = oversampling by 16, 1 = oversampling by 8
	
	// Configure stop bits to 1 stop bit
	//   00: 1 Stop bit;      01: 0.5 Stop bit
	//   10: 2 Stop bits;     11: 1.5 Stop bit
	USARTx->CR2 &= ~USART_CR2_STOP;   
                                    
	// CSet Baudrate to 9600 using APB frequency (80,000,000 Hz)
	// If oversampling by 16, Tx/Rx baud = f_CK / USARTDIV,  
	// If oversampling by 8,  Tx/Rx baud = 2*f_CK / USARTDIV
  // When OVER8 = 0, BRR = USARTDIV
	// USARTDIV = 80MHz/9600 = 8333 = 0x208D
	USARTx->BRR  = 0x208D; // Limited to 16 bits

	USARTx->CR1  |= (USART_CR1_RE | USART_CR1_TE);  	// Transmitter and Receiver enable
	
  if (USARTx == UART4){	
		USARTx->CR1 |= USART_CR1_RXNEIE;  			// Received Data Ready to be Read Interrupt  
		USARTx->CR1 &= ~USART_CR1_TCIE;    			// Transmission Complete Interrupt 
		USARTx->CR1 &= ~USART_CR1_IDLEIE;  			// Idle Line Detected Interrupt 
		USARTx->CR1 &= ~USART_CR1_TXEIE;   			// Transmit Data Register Empty Interrupt 
		USARTx->CR1 &= ~USART_CR1_PEIE;    			// Parity Error Interrupt 
		USARTx->CR1 &= ~USART_CR2_LBDIE;				// LIN Break Detection Interrupt Enable
		USARTx->CR1 &= ~USART_CR3_EIE;					// Error Interrupt Enable (Frame error, noise error, overrun error) 
		//USARTx->CR3 &= ~USART_CR3_CTSIE;				// CTS Interrupt
	}

	if (USARTx == USART2){
		USARTx->ICR |= USART_ICR_TCCF;
		USART1->CR3 |= USART_CR3_DMAT | USART_CR3_DMAR;
	}
	
	USARTx->CR1  |= USART_CR1_UE; // USART enable                 
	
	while ( (USARTx->ISR & USART_ISR_TEACK) == 0); // Verify that the USART is ready for reception
	while ( (USARTx->ISR & USART_ISR_REACK) == 0); // Verify that the USART is ready for transmission
}


uint8_t USART_Read (USART_TypeDef * USARTx) {
	// SR_RXNE (Read data register not empty) bit is set by hardware
	while (!(USARTx->ISR & USART_ISR_RXNE));  // Wait until RXNE (RX not empty) bit is set
	// USART resets the RXNE flag automatically after reading DR
	return ((uint8_t)(USARTx->RDR & 0xFF));
	// Reading USART_DR automatically clears the RXNE flag 
}


void USART_Write(USART_TypeDef * USARTx, uint8_t *buffer, uint32_t nBytes) {
	int i;
	// TXE is cleared by a write to the USART_DR register.
	// TXE is set by hardware when the content of the TDR 
	// register has been transferred into the shift register.
	for (i = 0; i < nBytes; i++) {
		while (!(USARTx->ISR & USART_ISR_TXE));   	// wait until TXE (TX empty) bit is set
		// Writing USART_DR automatically clears the TXE flag 	
		USARTx->TDR = buffer[i] & 0xFF;
		USART_Delay(300);
	}
	while (!(USARTx->ISR & USART_ISR_TC));   		  // wait until TC bit is set
	USARTx->ISR &= ~USART_ISR_TC;
}




/*Initializes Timer 4 Channel 1*/
void TIM4_init(void){
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;        // Enable GPIOB clock
	GPIOB->MODER   &=   ~(0x03 << 12);     // Clear bit 12 & 13 Alternate function mode 
  GPIOB->MODER   |=   (0x02 << 12);               // set as Alternate function mode
	GPIOB->PUPDR &=         ~(0X3<<12);                         // NO PULL-UP PULL-DOWN 
  GPIOB->OTYPER &=        ~(1<<6);                            // PUSH-PULL 
  GPIOB->AFR[0] &= ~GPIO_AFRL_AFRL6;  // Clear pin 6 for alternate function
  GPIOB->AFR[0] |=        0x2 << (4*6);                   // set PB pin 6 as AF2 (TIM4_CH1)
	GPIOB->PUPDR &=         ~(0X3<<12);                         // NO PULL-UP PULL-DOWN 
  GPIOB->OTYPER &=        ~(1<<6);                            // PUSH-PULL 
  GPIOB->AFR[0] &= ~GPIO_AFRL_AFRL6;  // Clear pin 6 for alternate function
  GPIOB->AFR[0] |=        0x2 << (4*6);                   // set PB pin 6 as AF2 (TIM4_CH1)
	RCC->APB1ENR1 |= RCC_APB1ENR1_TIM4EN;                 // ENABLE TIM4 CLOCK
	TIM4->PSC = 79;                                     // SET APPROPRAIT PRESCALER TO SLOW DOWN THE CLOCK        
	TIM4->CCMR1 &= ~TIM_CCMR1_CC1S;                         // CLEAR CAPTURE/COMPARE REGISTER
  TIM4->CCMR1 |= 0X1;                                                 // SELECT CH1 INPUTE CAPTURE 
  TIM4->CCMR1 &= ~TIM_CCMR1_IC1F;                         // DISABLE DIGITAL FILTERING
	TIM4->CCER &= ~((1<<1 | 1<<3));                                // SELECT BOTH RISING AND FALLING EDGE DETECTION CC1P & CC1NP
	TIM4->CCMR1 &= ~(TIM_CCMR1_IC1PSC);                 // INPUT PRESCALER 0 TO CAPTURE EACH VALID EDGE
  TIM4->CCER |= TIM_CCER_CC1E;                                // ENABLE COUNTER CAPTURE
	TIM4->DIER |= TIM_DIER_CC1IE;                               // ENABLE CH1 CAPTURE/COMPARE INTERRUPT
  TIM4->DIER |= TIM_DIER_CC1DE;   
  TIM4->DIER |= TIM_DIER_UIE;                                 // UPDATE INTERRUPT ENABLE
	TIM4->CR1 &= ~TIM_CR1_DIR;                                      // Set downcounting counter direction
  TIM4->CR1 |= TIM_CR1_CEN;                                       // Enable the counter
	NVIC_SetPriority(TIM4_IRQn, 1);                         // SET PRIORITY TO 1
  NVIC_EnableIRQ(TIM4_IRQn);                                  //ENABLE TIM4 INTERRUPT IN NVIC
	


	
}



/*Initializes Timer 2 Channel 3*/
void TIM2_init(void){
		RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;        // Enable GPIOB clock
		GPIOA->MODER   &=   ~(0x03 << (2*2));     // Clear bit 12 & 13 Alternate function mode 
    GPIOA->MODER   |=   0x02 << (2*2);                 // set as Alternate function mode 
    GPIOA->PUPDR &=         ~(1<<2);                           // NO PULL-UP PULL-DOWN 
    GPIOA->OTYPER &=        ~(1<<2);                           // PUSH-PULL 
    GPIOA->AFR[0] |=        0x1 << (4*2);
	
		RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;                 // ENABLE TIM2 CLOCK
    TIM2->PSC = 80;                                                        // SET APPROPRAIT PRESCALER TO SLOW DOWN THE CLOCK
    TIM2->ARR = 1020;                                                 // SET MAX PULSE WIDTH OF 65536us FOR 16-BIT TIMER
    TIM2->CR1 |= TIM_CR1_DIR;                                       // Set downcounting counter direction
    TIM2->CCMR2 &= ~(TIM_CCMR2_OC3M);                       // Clear OC3M (Channel 3)
		TIM2->CCMR2 |= TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2;
    TIM2->CCMR2 |= TIM_CCMR2_OC3PE;                         // CH3 Output Preload Enable
    TIM2->CR1 |= TIM_CR1_ARPE;                                  // Auto-reload Prelaod Enable
    TIM2->CCER |= TIM_CCER_CC3E;                                // Enable Output for CH3
    TIM2->EGR |= TIM_EGR_UG;                                        // Force Update
    TIM2->SR &= ~TIM_SR_UIF;                                        // Clear the Update Flag
    TIM2->DIER |= TIM_DIER_UIE;                                 // Enable Interrupt on Update
    TIM2->CCR3 &= ~(TIM_CCR3_CCR3);                     // Clear CCR3 (Channel 3) 
    TIM2->CCR3 |= 500;                                            // Load the register 
  	TIM2->CR1 |= TIM_CR1_CEN; 


}



/*Power On Self Test*/
int POST(void){	
	USART_Write(USART2, (uint8_t*)welcome_msg, strlen(welcome_msg));
	Delay(1000);
	if(tim_us > 0){
		USART_Write(USART2, (uint8_t*)end_msg, strlen(end_msg));
		Delay(1000);
		reset();
		GPIOB->ODR &= ~(1<<2);
		GPIOE->ODR |= 1<<8;
		return 1;
	}
	else{
		GPIOB->ODR |= 1<<2;
		USART_Write(USART2, (uint8_t*)fail_msg, strlen(fail_msg));
		return 0;
	}
}


/*Delay in Milli-Seconds*/
void Delay(uint32_t ms) {
	uint32_t time = 25*800*ms;    
	while(--time);   
}


void USART_Delay(uint32_t us) {
	uint32_t time = 100*us/7;    
	while(--time);   
}



/*Interrup Service Routine for INPUT CAPTURE*/
void TIM4_IRQHandler(void){
	
		if ((TIM4->SR & TIM_SR_UIF) != 0){         /*Checks for timer overflow*/         
        overflow++;                                
        TIM4->SR &= ~TIM_SR_UIF;                                    
    }
		if ((TIM4->SR & TIM_SR_CC1IF) != 0){      
			capture_new = TIM4->CCR1;                                      
			tim_us = (capture_new - capture_old) + (65536 * overflow);       
			capture_old = capture_new;
			overflow = 0;
			captured = 1;
			count_g ++;
    }
}


/*Captures the timer values (micro-seconds) in Array...*/
void capture(void){
	
	while(1){
		
		if(captured){      // Flag to check if a value is captured
			captured = 0;
			if(count_g <= NO_OF_EDGES){
				if(tim_us < upper_limit && tim_us > lower_limit){    // Check upper and lower limtis
					time_array[array_index] = tim_us;
					array_index ++;
				}
			}
			else
				break;
			
		}
	}
}

int cmpfunc (const void * a, const void * b) {
   return ( *(int*)a - *(int*)b );
}



/*Processes and transmits the data VIA UART*/ 
void process(void){
	int i, j, count;
	
	qsort(time_array, NO_OF_EDGES+1, sizeof(int),cmpfunc ); // Sorts the time_array
	
	
	for(i=0; i<=NO_OF_EDGES; i++)
    {
      freq[i] = -1;
    }
	for(i=0; i<=NO_OF_EDGES; i++)  // Calculates the frequency of each time value in time_array
    {
        count = 1;
        for(j=i+1; j<=NO_OF_EDGES; j++)
        {
            if(time_array[i]==time_array[j])
            {
                count++;

                freq[j] = 0;
            }
        }

        if(freq[i] != 0)
        {
            freq[i] = count;
        }
	
	}
		strcpy(uart_data,"TIME(uS):\tCOUNT:\r\n");
		USART_Write(USART2, (uint8_t*)uart_data, strlen(uart_data));
		for(i=0; i<=NO_OF_EDGES; i++)
    {
        if(freq[i] != 0 && time_array[i] != 0 )   //Transmits the Data
        {
					sprintf(uart_data, "%u \t\t %d\r\n",time_array[i],freq[i]);
					USART_Write(USART2, (uint8_t*)uart_data, strlen(uart_data));
        }
				
    }
		
		reset();
}




/*Resets all the variables and arrays after the data is transmitted*/ 
void reset(void){
	int i =0;
	  overflow = 0;
	  capture_new = 0;
	  capture_old = 0;
	  tim_us = 0;
	  captured = 0;
	  count_g =0;
		array_index = 0;
	
		for(i = 0; i < NO_OF_EDGES+1; i++)
			uart_data[i] = 0;
		for(i = 0; i < NO_OF_EDGES+1; i++)
			time_array[i] = 0;
}


/*converts an int to a string*/
char* itoa(int val, int base){
	
	static char buf[32] = {0};
	
	int i = 30;
	
	for(; val && i ; --i, val /= base)
	
		buf[i] = "0123456789abcdef"[val % base];
	
	return &buf[i+1];
	
}


/*Takes the User Input for Lower and Upper limits*/
void get_UI_limit()
{
	lower_limit=0;
	upper_limit = 0;
	int	i = 0;
	int j = 0;
	/*flag indicates if the user input is acceptable*/
	/*flag = 0: acceptable input*/
	/*flag = 1: user did not enter a value consist of 0-9*/
	/*flag = 2: user did not enter a value between 50-9950*/
	int flag = 0; 
	
	//System_Clock_Init(); // Switch System Clock = 80 MHz
	//UART2_Init();
		
	USART_Write(USART2, (uint8_t *)str, strlen(str));	
	while(ui_buffer1[j-1] != '\r')
	{
			ui_buffer1[j] = (uint8_t)USART_Read(USART2);
			USART_Write(USART2, (uint8_t *)&ui_buffer1[j], 1);		
			j++;
	}
	lower_limit = atoi(ui_buffer1);
	upper_limit = lower_limit + 100;

	for (i = 0; i < j-1; i++)
	{
		if (ui_buffer1[i] > 0x39 || ui_buffer1[i] < 0x30)
		{
			flag = 1;
		}
		else if (lower_limit < 50 || upper_limit > 9950)
		{
			flag = 2;
		}
	}
	if (flag == 0)
	{
		USART_Write(USART2, (uint8_t *)"\r\nLower limit is: ", 18);
		USART_Write(USART2, (uint8_t *)ui_buffer1, j);
		USART_Write(USART2, (uint8_t *)"\r\n", 2);
		USART_Write(USART2, (uint8_t *)"Upper limit is: ", 17);
		ui_buffer2_ptr = itoa(upper_limit, 10);
		USART_Write(USART2, (uint8_t *)ui_buffer2_ptr, j);
		USART_Write(USART2, (uint8_t *)"\r\n", 2);
	}
	else if (flag == 2)
	{
		USART_Write(USART2, (uint8_t *)error_msg2, strlen(error_msg2));
		USART_Write(USART2, (uint8_t *)"\r\n", 2);
		lower_limit = 0;
		upper_limit = 0;
	}
	else
	{
		USART_Write(USART2, (uint8_t *)error_msg1, strlen(error_msg1));
		USART_Write(USART2, (uint8_t *)"\r\n", 2);
	}
	/*reset buffer index and flag*/
	j = 0;
	flag = 0;
	reset();
	
}
