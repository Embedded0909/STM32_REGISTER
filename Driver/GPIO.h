#ifndef GPIO_H
#define GPIO_H
#include <stdint.h>

/*
	MODEy[1:0]: Port x mode bits (y= 0 .. 7)
	00: Input mode (reset state)
	01: Output mode, max speed 10 MHz.
	10: Output mode, max speed 2 MHz.
	11: Output mode, max speed 50 MHz
*/

/*
	CNFy[1:0]: Port x configuration bits (y= 0 .. 7)
	In input mode (MODE[1:0]=00):
	00: Analog mode
	01: Floating input (reset state)
	10: Input with pull-up / pull-down
	11: Reserved
	In output mode (MODE[1:0] > 00):
	00: General purpose output push-pull
	01: General purpose output Open-drain
	10: Alternate function output Push-pull
	11: Alternate function output Open-drain
*/

typedef struct
{
	union{
		uint32_t REG;
		struct{
			uint32_t MODE0 : 2;       
			uint32_t CNF0  : 2;
			uint32_t MODE1 : 2;
			uint32_t CNF1  : 2;
			uint32_t MODE2 : 2;
			uint32_t CNF2  : 2;
			uint32_t MODE3 : 2;
			uint32_t CNF3  : 2;
			uint32_t MODE4 : 2;
			uint32_t CNF4  : 2;
			uint32_t MODE5 : 2;
			uint32_t CNF5  : 2;
			uint32_t MODE6 : 2;
			uint32_t CNF6  : 2;
			uint32_t MODE7 : 2;
			uint32_t CNF7  : 2;
		}BITS;
	}CRL;
	
	union{
		uint32_t REG;
		struct{
			uint32_t MODE8  : 2;       
			uint32_t CNF8   : 2;
			uint32_t MODE9  : 2;
			uint32_t CNF9   : 2;
			uint32_t MODE10 : 2;
			uint32_t CNF10  : 2;
			uint32_t MODE11 : 2;
			uint32_t CNF11  : 2;
			uint32_t MODE12 : 2;
			uint32_t CNF12  : 2;
			uint32_t MODE13 : 2;
			uint32_t CNF13  : 2;
			uint32_t MODE14 : 2;
			uint32_t CNF14  : 2;
			uint32_t MODE15 : 2;
			uint32_t CNF15  : 2;
		}BITS;
	}CRH;
	
	union{
		uint32_t REG;
		struct{
			uint32_t IDR0       : 1;    
			uint32_t IDR1       : 1;	
			uint32_t IDR2       : 1;
			uint32_t IDR3       : 1;
			uint32_t IDR4       : 1;
			uint32_t IDR5       : 1;
			uint32_t IDR6       : 1;
			uint32_t IDR7       : 1;
			uint32_t IDR8       : 1;
			uint32_t IDR9       : 1;
			uint32_t IDR10      : 1;
			uint32_t IDR11      : 1;
			uint32_t IDR12      : 1;
			uint32_t IDR13      : 1;
			uint32_t IDR14      : 1;
			uint32_t IDR15      : 1;
			uint32_t Reserved   : 16;
		}BITS;
	}IDR;
	
	union{
		uint32_t REG;
		struct{
			uint32_t ODR0       : 1;   
			uint32_t ODR1       : 1; 	
			uint32_t ODR2       : 1; 
			uint32_t ODR3       : 1; 
			uint32_t ODR4       : 1; 
			uint32_t ODR5       : 1; 
			uint32_t ODR6       : 1; 
			uint32_t ODR7       : 1; 
			uint32_t ODR8       : 1; 
			uint32_t ODR9       : 1; 
			uint32_t ODR10      : 1; 
			uint32_t ODR11      : 1; 
			uint32_t ODR12      : 1; 
			uint32_t ODR13      : 1; 
			uint32_t ODR14      : 1; 
			uint32_t ODR15      : 1; 
			uint32_t Reserved   : 16;
		}BITS;
	}ODR;
	
	union{
		uint32_t REG;
		struct{
			uint32_t BS0 : 1;  
			uint32_t BS1 : 1; 
			uint32_t BS2 : 1; 
			uint32_t BS3 : 1; 
			uint32_t BS4 : 1; 
			uint32_t BS5 : 1; 
			uint32_t BS6 : 1; 
			uint32_t BS7 : 1; 
			uint32_t BS8 : 1; 
			uint32_t BS9 : 1; 
			uint32_t BS10 : 1; 
			uint32_t BS11 : 1; 
			uint32_t BS12 : 1; 
			uint32_t BS13 : 1; 
			uint32_t BS14 : 1; 
			uint32_t BS15 : 1; 
			uint32_t BR0 : 1;  
			uint32_t BR1 : 1; 
			uint32_t BR2 : 1; 
			uint32_t BR3 : 1; 
			uint32_t BR4 : 1; 
			uint32_t BR5 : 1; 
			uint32_t BR6 : 1; 
			uint32_t BR7 : 1; 
			uint32_t BR8 : 1; 
			uint32_t BR9 : 1; 
			uint32_t BR10 : 1; 
			uint32_t BR11 : 1; 
			uint32_t BR12 : 1; 
			uint32_t BR13 : 1; 
			uint32_t BR14 : 1; 
			uint32_t BR15 : 1; 
		}BITS;
	}BSRR;
	
	union{
		uint32_t REG;
		struct{
			uint32_t BR0 : 1;  
			uint32_t BR1 : 1; 
			uint32_t BR2 : 1; 
			uint32_t BR3 : 1; 
			uint32_t BR4 : 1; 
			uint32_t BR5 : 1; 
			uint32_t BR6 : 1; 
			uint32_t BR7 : 1; 
			uint32_t BR8 : 1; 
			uint32_t BR9 : 1; 
			uint32_t BR10 : 1; 
			uint32_t BR11 : 1; 
			uint32_t BR12 : 1; 
			uint32_t BR13 : 1; 
			uint32_t BR14 : 1; 
			uint32_t BR15 : 1;     
			uint32_t Reserved : 16;
		}BITS;
	}BRR;
	
	 union{
		uint32_t REG;
		 struct{
			uint32_t LCK0 : 1; 
			uint32_t LCK1 : 1; 
			uint32_t LCK2 : 1; 
			uint32_t LCK3 : 1; 
			uint32_t LCK4 : 1; 
			uint32_t LCK5 : 1; 
			uint32_t LCK6 : 1; 
			uint32_t LCK7 : 1; 
			uint32_t LCK8 : 1; 
			uint32_t LCK9 : 1; 
			uint32_t LCK10 : 1; 
			uint32_t LCK11 : 1; 
			uint32_t LCK12 : 1; 
			uint32_t LCK13 : 1; 
			uint32_t LCK14 : 1; 
			uint32_t LCK15 : 1; 
			uint32_t LCKK : 1; 
			uint32_t Reserved  : 15;
		}BITS;
	}LCKR;

} GPIO_TypeDef;
#define GPIOA ((GPIO_TypeDef*) 0x40010800UL)
#define GPIOB ((GPIO_TypeDef*) 0x40010C00UL)
#define GPIOC ((GPIO_TypeDef*) 0x40011000UL)

void GPIO_config();

#endif