#ifndef CORE_H
#define CORE_H
#include "type.h"

//----------------------------------------NVIC---------------------------------------------

#define ADDREGS_ISER0          0xE000E100
#define ADDREGS_ICER0          0xE000E180
#define ADDREGS_ISPR0          0xE000E200
#define ADDREGS_ICPR0          0xE000E280
#define ADDREGS_IABR0          0xE000E300
#define ADDREGS_IPR0           0xE000E400


#define ISER0            ((volatile __BIT32*) ADDREGS_ISER0)  // NVIC Interrupt Set Enable Register
#define ICER0            ((volatile __BIT32*) ADDREGS_ICER0)  // NVIC Interrupt Clear Enable Register
#define ISPR0            ((volatile __BIT32*) ADDREGS_ISPR0)  // NVIC Interrupt Set Pending Register
#define ICPR0            ((volatile __BIT32*) ADDREGS_ICPR0)  // NVIC Interrupt Clear Pending Register
#define IABR0            ((volatile __BIT32*) ADDREGS_IABR0)  // Read only
#define IPR0             ((volatile __BIT32*) ADDREGS_IPR0 )  // NVIC Interrupt Priority Register

#endif