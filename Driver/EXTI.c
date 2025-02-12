#include "EXTI.h"

void InteruptND(){}

void EXTI0_IRQHandler(void) {
    if (myEXTI->PR.BITS.BIT0) {  
        printf("Interupt line 0\r\n");
        InteruptND();
        // Ngắt bit pedding để đón nhận interupt tiếp theo
        myEXTI->PR.REGISTER |= (1<<6);   
    }
}

void EXTI_config(){
    
    //enable interupt line 0 - per
    myEXTI->IMR.BITS.BIT0 = 1;
    //falling mode
    myEXTI->FTSR.BITS.BIT0 = 1;
    myEXTI->RTSR.BITS.BIT0 = 0;
    //enable interupt line 0 - core
    ISER0->BITS.BIT6 = 1 ;
   
}