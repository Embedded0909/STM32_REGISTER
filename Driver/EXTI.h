#ifndef EXTI_H
#define EXTI_H
#include <stdint.h>
#include "type.h"
#include "core.h"


typedef struct {
    BIT32 IMR;
    BIT32 EMR;
    BIT32 RTSR;
    BIT32 FTSR;
    BIT32 SWIER;
    BIT32 PR;
}EXTI;

#define myEXTI ((EXTI *) 0x40010400)

void InteruptND();                      // __Wake    có thẻ ghi đè lại 
void EXTI0_IRQHandler(void);
void EXTI_config();

#endif

