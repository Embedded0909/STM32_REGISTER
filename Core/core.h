#ifndef CORE_H
#define CORE_H
#include "type.h"

//----------------------------------------NVIC---------------------------------------------

#define ADDRESS_ISER0          0xE000E100
#define ADDRESS_ICER0          0xE000E180
#define ADDRESS_ISPR0          0xE000E200
#define ADDRESS_ICPR0          0xE000E280
#define ADDRESS_IABR0          0xE000E300
#define ADDRESS_IPR0           0xE000E400


#define ISER0            ((volatile BIT32*) ADDRESS_ISER0)  // NVIC Interrupt Set Enable Register
#define ICER0            ((volatile BIT32*) ADDRESS_ICER0)  // NVIC Interrupt Clear Enable Register
#define ISPR0            ((volatile BIT32*) ADDRESS_ISPR0)  // NVIC Interrupt Set Pending Register
#define ICPR0            ((volatile BIT32*) ADDRESS_ICPR0)  // NVIC Interrupt Clear Pending Register
#define IABR0            ((volatile BIT32*) ADDRESS_IABR0)  // Read only
#define IPR0             ((volatile BIT32*) ADDRESS_IPR0 )  // NVIC Interrupt Priority Register

#endif