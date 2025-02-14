#ifndef EXTI_H
#define EXTI_H
#include <stdint.h>
#include "type.h"
#include "core.h"


typedef struct {
    __BIT32 IMR;
    __BIT32 EMR;
    __BIT32 RTSR;
    __BIT32 FTSR;
    __BIT32 SWIER;
    __BIT32 PR;
}EXTI;

#define myEXTI ((EXTI *) 0x40010400)

void InteruptND();                      // __Wake    có thẻ ghi đè lại 
void EXTI0_IRQHandler(void);
void EXTI_config();

#endif

