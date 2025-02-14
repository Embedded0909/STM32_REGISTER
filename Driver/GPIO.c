#include "GPIO.h"

void GPIO_config(){
	GPIOC->CRH.BITS.MODE13 = 3;
	GPIOC->CRH.BITS.CNF13 = 2;
}