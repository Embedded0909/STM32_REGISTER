#ifndef ADC_H
#define ADC_H

#include <stdint.h>
#include "RCC.h"

//-------------------------------ADDRESS BASE-------------------------------------

#define    ADC01_BASE               0x40012400
#define    ADC02_BASE               0x40012800 
#define    ADC03_BASE               0x40013C00 

//-----------------------------ADDRESS REGISRER-----------------------------------

#define    ADD_ADC_SR               0x00
#define    ADD_ADC_CR1              0x04
#define    ADD_ADC_CR2              0x08
#define    ADD_ADC_SMPR1            0x0C
#define    ADD_ADC_SMPR2            0x10
#define    ADD_ADC_JOFR1            0x14
#define    ADD_ADC_JOFR2            0x18
#define    ADD_ADC_JOFR3            0x1C
#define    ADD_ADC_JOFR4            0x20
#define    ADD_ADC_HTR              0x24
#define    ADD_ADC_LTR              0x28
#define    ADD_ADC_SQR1             0x2C
#define    ADD_ADC_SQR2             0x30
#define    ADD_ADC_SQR3             0x34
#define    ADD_ADC_JSQR             0x38
#define    ADD_ADC_JDR1             0x3C
#define    ADD_ADC_JDR2             0x40
#define    ADD_ADC_JDR3             0x44
#define    ADD_ADC_JDR4             0x48
#define    ADD_ADC_DR               0x4C 


//------------------------------ADC 01--------------------------------

#define    ADC01_SR               ((uint32_t*)(ADC01_BASE + ADD_ADC_SR))
#define    ADC01_CR1              ((uint32_t*)(ADC01_BASE + ADD_ADC_CR1))
#define    ADC01_CR2              ((uint32_t*)(ADC01_BASE + ADD_ADC_CR2))
#define    ADC01_SMPR1            ((uint32_t*)(ADC01_BASE + ADD_ADC_SMPR1))
#define    ADC01_SMPR2            ((uint32_t*)(ADC01_BASE + ADD_ADC_SMPR2))
#define    ADC01_JOFR1            ((uint32_t*)(ADC01_BASE + ADD_ADC_JOFR1))
#define    ADC01_JOFR2            ((uint32_t*)(ADC01_BASE + ADD_ADC_JOFR2))
#define    ADC01_JOFR3            ((uint32_t*)(ADC01_BASE + ADD_ADC_JOFR3))
#define    ADC01_JOFR4            ((uint32_t*)(ADC01_BASE + ADD_ADC_JOFR4))
#define    ADC01_HTR              ((uint32_t*)(ADC01_BASE + ADD_ADC_HTR))
#define    ADC01_LTR              ((uint32_t*)(ADC01_BASE + ADD_ADC_LTR))
#define    ADC01_SQR1             ((uint32_t*)(ADC01_BASE + ADD_ADC_SQR1))
#define    ADC01_SQR2             ((uint32_t*)(ADC01_BASE + ADD_ADC_SQR2))
#define    ADC01_SQR3             ((uint32_t*)(ADC01_BASE + ADD_ADC_SQR3))
#define    ADC01_JSQR             ((uint32_t*)(ADC01_BASE + ADD_ADC_JSQR))
#define    ADC01_JDR1             ((uint32_t*)(ADC01_BASE + ADD_ADC_JDR1))
#define    ADC01_JDR2             ((uint32_t*)(ADC01_BASE + ADD_ADC_JDR2))
#define    ADC01_JDR3             ((uint32_t*)(ADC01_BASE + ADD_ADC_JDR3))
#define    ADC01_JDR4             ((uint32_t*)(ADC01_BASE + ADD_ADC_JDR4))
#define    ADC01_DR               ((uint32_t*)(ADC01_BASE + ADD_ADC_DR))


//------------------------------ADC 02--------------------------------

#define    ADC02_SR               ((uint32_t*)(ADC02_BASE + ADD_ADC_SR))
#define    ADC02_CR1              ((uint32_t*)(ADC02_BASE + ADD_ADC_CR1))
#define    ADC02_CR2              ((uint32_t*)(ADC02_BASE + ADD_ADC_CR2))
#define    ADC02_SMPR1            ((uint32_t*)(ADC02_BASE + ADD_ADC_SMPR1))
#define    ADC02_SMPR2            ((uint32_t*)(ADC02_BASE + ADD_ADC_SMPR2))
#define    ADC02_JOFR1            ((uint32_t*)(ADC02_BASE + ADD_ADC_JOFR1))
#define    ADC02_JOFR2            ((uint32_t*)(ADC02_BASE + ADD_ADC_JOFR2))
#define    ADC02_JOFR3            ((uint32_t*)(ADC02_BASE + ADD_ADC_JOFR3))
#define    ADC02_JOFR4            ((uint32_t*)(ADC02_BASE + ADD_ADC_JOFR4))
#define    ADC02_HTR              ((uint32_t*)(ADC02_BASE + ADD_ADC_HTR))
#define    ADC02_LTR              ((uint32_t*)(ADC02_BASE + ADD_ADC_LTR))
#define    ADC02_SQR1             ((uint32_t*)(ADC02_BASE + ADD_ADC_SQR1))
#define    ADC02_SQR2             ((uint32_t*)(ADC02_BASE + ADD_ADC_SQR2))
#define    ADC02_SQR3             ((uint32_t*)(ADC02_BASE + ADD_ADC_SQR3))
#define    ADC02_JSQR             ((uint32_t*)(ADC02_BASE + ADD_ADC_JSQR))
#define    ADC02_JDR1             ((uint32_t*)(ADC02_BASE + ADD_ADC_JDR1))
#define    ADC02_JDR2             ((uint32_t*)(ADC02_BASE + ADD_ADC_JDR2))
#define    ADC02_JDR3             ((uint32_t*)(ADC02_BASE + ADD_ADC_JDR3))
#define    ADC02_JDR4             ((uint32_t*)(ADC02_BASE + ADD_ADC_JDR4))
#define    ADC02_DR               ((uint32_t*)(ADC02_BASE + ADD_ADC_DR))

//------------------------------ADC 03--------------------------------

#define    ADC03_SR               ((uint32_t*)(ADC03_BASE + ADD_ADC_SR))
#define    ADC03_CR1              ((uint32_t*)(ADC03_BASE + ADD_ADC_CR1))
#define    ADC03_CR2              ((uint32_t*)(ADC03_BASE + ADD_ADC_CR2))
#define    ADC03_SMPR1            ((uint32_t*)(ADC03_BASE + ADD_ADC_SMPR1))
#define    ADC03_SMPR2            ((uint32_t*)(ADC03_BASE + ADD_ADC_SMPR2))
#define    ADC03_JOFR1            ((uint32_t*)(ADC03_BASE + ADD_ADC_JOFR1))
#define    ADC03_JOFR2            ((uint32_t*)(ADC03_BASE + ADD_ADC_JOFR2))
#define    ADC03_JOFR3            ((uint32_t*)(ADC03_BASE + ADD_ADC_JOFR3))
#define    ADC03_JOFR4            ((uint32_t*)(ADC03_BASE + ADD_ADC_JOFR4))
#define    ADC03_HTR              ((uint32_t*)(ADC03_BASE + ADD_ADC_HTR))
#define    ADC03_LTR              ((uint32_t*)(ADC03_BASE + ADD_ADC_LTR))
#define    ADC03_SQR1             ((uint32_t*)(ADC03_BASE + ADD_ADC_SQR1))
#define    ADC03_SQR2             ((uint32_t*)(ADC03_BASE + ADD_ADC_SQR2))
#define    ADC03_SQR3             ((uint32_t*)(ADC03_BASE + ADD_ADC_SQR3))
#define    ADC03_JSQR             ((uint32_t*)(ADC03_BASE + ADD_ADC_JSQR))
#define    ADC03_JDR1             ((uint32_t*)(ADC03_BASE + ADD_ADC_JDR1))
#define    ADC03_JDR2             ((uint32_t*)(ADC03_BASE + ADD_ADC_JDR2))
#define    ADC03_JDR3             ((uint32_t*)(ADC03_BASE + ADD_ADC_JDR3))
#define    ADC03_JDR4             ((uint32_t*)(ADC03_BASE + ADD_ADC_JDR4))
#define    ADC03_DR               ((uint32_t*)(ADC03_BASE + ADD_ADC_DR))


//-----------------------------------API-------------------------------------

void ADC_Enable_Clock();



#endif