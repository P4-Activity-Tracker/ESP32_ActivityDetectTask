#include <Arduino.h>
// Inkluder header fil (funktions fil)
#include "arrayProcessing.h"

// Kopier int16_t array til double array
void copyArray(double *fromArray, double *toArray, int16_t arrayLength) {
	for (uint16_t i = 0; i < arrayLength; i++) {
		*(toArray+i) = *(fromArray+i);
	}
}

// Sæt alle værdier i array til given værdi
void setArrayTo(double *arrayPointer, uint16_t arrayLength, double value) {
	for (uint16_t i = 0; i < arrayLength; i++) {
		*(arrayPointer+i) = 0;
	}
}

// Finder maksimal værdi i array
double maxInArray(double *buffPointer, uint16_t buffSize) {
	double maxValue = *(buffPointer);
	#ifdef printFunc_maxInArray
		Serial.print("Finding max. Start value: ");
		Serial.println(maxValue);
		uint16_t maxIndex = 0;
	#endif
	for (uint16_t i = 1; i < buffSize; i++) {
		if (maxValue < *(buffPointer+i)) {
			#ifdef printFunc_maxInArray
				Serial.print(maxValue);
				Serial.print(" > ");
				Serial.println(*(buffPointer+i));
				maxIndex = i;
			#endif
			maxValue = *(buffPointer+i);
		}
	}
	#ifdef printFunc_maxInArray
		Serial.print("maxInArray(), max is: ");
		Serial.print(maxValue);
		Serial.print(" at index: ");
		Serial.println(maxIndex);
	#endif
	return maxValue;	
}

// Finder maksimal værdi i array
uint32_t maxInArray(uint32_t *buffPointer, uint16_t buffSize) {
	double maxValue = *(buffPointer);
	#ifdef printFunc_maxInArray
		Serial.print("Finding max. Start value: ");
		Serial.println(maxValue);
		uint16_t maxIndex = 0;
	#endif
	for (uint16_t i = 1; i < buffSize; i++) {
		if (maxValue < *(buffPointer+i)) {
			#ifdef printFunc_maxInArray
				Serial.print(maxValue);
				Serial.print(" > ");
				Serial.println(*(buffPointer+i));
				maxIndex = i;
			#endif
			maxValue = *(buffPointer+i);
		}
	}
	#ifdef printFunc_maxInArray
		Serial.print("maxInArray(), max is: ");
		Serial.print(maxValue);
		Serial.print(" at index: ");
		Serial.println(maxIndex);
	#endif
	return maxValue;	
}

// Finder minimal værdi i array
double minInArray(double *buffPointer, uint16_t buffSize) {
	double minValue = *(buffPointer);
	#ifdef printFunc_maxInArray
		Serial.print("Finding min. Start value: ");
		Serial.println(minValue);
		uint16_t minIndex = 0;
	#endif
	for (uint16_t i = 1; i < buffSize; i++) {
		if (minValue > *(buffPointer+i)) {
			#ifdef printFunc_maxInArray
				Serial.print(minValue);
				Serial.print(" > ");
				Serial.println(*(buffPointer+i));
				minIndex = i;
			#endif
			minValue = *(buffPointer+i);
		}
	}
	#ifdef printFunc_minInArray
		Serial.print("minInArray(), min is: ");
		Serial.print(minValue);
		Serial.print(" at index: ");
		Serial.println(minIndex);
	#endif
	return minValue;
}

// Normaliser array med max
void normalizeArray(double *arrayPointer, uint16_t arrayLength, double maxValue){
	#ifdef printFunc_normalizeArray
		Serial.print("normalizeArray(), Normalizing, with max: ");
		Serial.println(maxValue);
	#endif
	for(int i=0; i < arrayLength; i++) {
		*(arrayPointer+i) = *(arrayPointer+i) / maxValue ;
	}
}

// Normaliser array med min og max
void normalizeArray(double *arrayPointer, uint16_t arrayLength, double minValue, double maxValue){
	#ifdef printFunc_normalizeArray
		Serial.print("normalizeArray(), Normalizing, with min/max: ");
		Serial.print(minValue);
		Serial.print(" ");
		Serial.println(maxValue);
	#endif
	for(int i=0; i < arrayLength; i++) {
		*(arrayPointer+i) = (*(arrayPointer+i) - minValue) / (maxValue - minValue);
	}
}

// Bereng summen af alle datapunkter i et array fra startIndex til endIndex. endIndex bør ikke overstige størrelsen af arrayet. startindex er inklusiv, endIndex er eksklusiv
double arraySum(double *arrayPointer, uint16_t startIndex, uint16_t endIndex) {
	double sumOfArray = 0;
	for (uint16_t i = startIndex; i < endIndex; i++) {
		sumOfArray = sumOfArray + *(arrayPointer+i);
	}
	#ifdef printFunc_arraySum
		Serial.print("arraySum(), Finding sum, with start/stop and sum: ");
		Serial.print(startIndex);
		Serial.print(" ");
		Serial.print(endIndex);
		Serial.print(" ");
		Serial.println(sumOfArray);
	#endif
	return sumOfArray;
}

uint64_t arraySum(uint32_t *arrayPointer, uint16_t startIndex, uint16_t endIndex) {
	uint64_t sumOfArray = 0;
	for (uint16_t i = startIndex; i < endIndex; i++) {
		sumOfArray = sumOfArray + *(arrayPointer+i);
	}
	#ifdef printFunc_arraySum
		Serial.print("arraySum(), Finding sum, with start/stop and sum: ");
		Serial.print(startIndex);
		Serial.print(" ");
		Serial.print(endIndex);
		Serial.print(" ");
		Serial.println(sumOfArray);
	#endif
	return sumOfArray;
}


// Finder absolutte værdi af komplex nummer gem i real array og imaginær array, gem i real
void absComplexArray(double *realPointer, double *imagPointer, uint16_t arrayLength) {
	for (int i=0; i < arrayLength; i++) {
		*(realPointer+i) = sqrt(pow(*(realPointer+i),2) + pow(*(imagPointer+i),2));
	}
}

// Find peaks i array
uint8_t findPeaksInArray(double *arrayPointer, uint16_t arrayLength, double threshold, uint16_t timeout) {
	uint8_t peakCount = 0; // Antal peaks fundet
	int32_t lastPeakIndex = -timeout; // Timeout
	for (uint16_t i = 1; i < arrayLength - 1; i++) {
		// Gem array værdi
		double indexValue = *(arrayPointer+i);
		// Tjek om timeout er overstået
		if ((i - lastPeakIndex) > timeout) {
			// Tjek om over threshold
			if (indexValue > threshold) {
				// Tjek om værdi er mindre før og efter
				if (indexValue >= (*(arrayPointer+i+1)) && indexValue > (*(arrayPointer+i-1))) {
					// Optæl antal peaks
					peakCount++;
					// Gem ny lastPeakIndex
					lastPeakIndex = i;
				}
			}
		}
	}
	#ifdef printFunc_findPeaksInArray
		Serial.print("findPeaksInArray(), Found peaks: ");
		Serial.println(peakCount);
	#endif
	return peakCount;
}

bool isIdentical(double *arrayOne, double *arrayTwo, uint16_t arraySize) {
	bool identical = true; 
	for (uint16_t i = 0; i < arraySize; i++) {
		if (*(arrayOne+i) != *(arrayTwo+i)) {
			identical = false;
			Serial.println("Not identical!");
		}
	}
	return identical;
}

void isSimilar(const double *arrayOne, double *arrayTwo, uint16_t arraySize) {
	double difference = 0;
	double sumOfDifference = 0;
	double largesDifference = 0;
	Serial.println("Comparing similarity: ");
	for (uint16_t i = 0; i < arraySize; i++) {
		//Serial.print("Diff: ");
		//Serial.println(*(arrayOne+i) - *(arrayTwo+i),6);
		difference = abs(*(arrayOne+i) - *(arrayTwo+i));
		if (difference > largesDifference) {
			largesDifference = difference;
		}
		sumOfDifference = sumOfDifference + difference;
	}
	Serial.print("Summed diff: ");
	Serial.println(sumOfDifference,6);
	Serial.print("Largest diff: ");
	Serial.println(largesDifference,6);
	Serial.print("Average diff: ");
	Serial.println(largesDifference/arraySize,6);
}