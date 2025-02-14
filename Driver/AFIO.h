#ifndef AFIO_H
#define AFIO_H
#include <stdint.h>

//------------------------------------

typedef struct{
    typedef union 
    {
        uint32_t REG;
        typedef struct{
            uint32_t PIN      : 4;  
            uint32_t PORT     : 1;  
            uint32_t EVOE     : 1;   
            uint32_t Reserved : 24; 
        }BITS;
    }AFIO_EVCR;
    uint32_t AFIO_MAPR; // Not use
    typedef union 
    {
        uint32_t REG;
        typedef struct {
            uint32_t EXTICR1_0 : 4;  
            uint32_t EXTICR1_1 : 4;  
            uint32_t EXTICR1_2 : 4;  
            uint32_t EXTICR1_3 : 4;  
            uint32_t Reserved  : 16; 
        } BITS;
    }AFIO_EXTICR1;
    typedef union {
        uint32_t REG;
        typedef struct {
            uint32_t EXTICR2_4 : 4; 
            uint32_t EXTICR2_5 : 4; 
            uint32_t EXTICR2_6 : 4;  
            uint32_t EXTICR2_7 : 4;  
            uint32_t Reserved  : 16; 
        } BITS;
    } AFIO_EXTICR2;

    typedef union {
        uint32_t REG;
        typedef struct {
            uint32_t EXTICR3_8 : 4;  
            uint32_t EXTICR3_9 : 4;  
            uint32_t EXTICR3_10 : 4; 
            uint32_t EXTICR3_11 : 4; 
            uint32_t Reserved    : 16; 
        } BITS;
    } AFIO_EXTICR3;

    typedef union {
        uint32_t REG;
        typedef struct {
            uint32_t EXTICR4_12 : 4;  
            uint32_t EXTICR4_13 : 4;  
            uint32_t EXTICR4_14 : 4;  
            uint32_t EXTICR4_15 : 4;  
            uint32_t Reserved    : 16;
        } BITS;
    } AFIO_EXTICR4;

    uint32_t AFIO_MAPR2; // Not use


    
}AFIO ;




#endif