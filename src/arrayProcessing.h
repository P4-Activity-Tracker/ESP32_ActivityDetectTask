// Sikre at biblioteket kun inkluderes en gang
#ifndef _arrayProcessing_
#define _arrayProcessing_

// Inkluder standard biblioteker
#include <Arduino.h>

// Funktioner
//----------------------------------------------
void copyArray(double *fromArray, double *toArray, int16_t arrayLength);
void setArrayTo(double *arrayPointer, uint16_t arrayLength, double value);

double maxInArray(double *buffPointer, uint16_t buffSize);
double minInArray(double *buffPointer, uint16_t buffSize);
void normalizeArray(double *arrayPointer, uint16_t arrayLength, double maxValue);
void normalizeArray(double *arrayPointer, uint16_t arrayLength, double minValue, double maxValue);

double arraySum(double *arrayPointer, uint16_t startIndex, uint16_t endIndex);

void absComplexArray(double *realPointer, double *imagPointer, uint16_t arrayLength);

uint8_t findPeaksInArray(double *arrayPointer, uint16_t arrayLength, double threshold, uint16_t timeout);

bool isIdentical(double *arrayOne, double *arrayTwo, uint16_t arraySize);
void isSimilar(const double *arrayOne, double *arrayTwo, uint16_t arraySize);
#endif