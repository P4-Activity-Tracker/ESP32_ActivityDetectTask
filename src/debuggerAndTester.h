#ifndef _debuggerAndTester_
#define _debuggerAndTester_

#include "Arduino.h"
#include "arrayProcessing.h"

// Debug funktionalitet
#define printFunc // Udkommenter denne for at stoppe funktions print.
#ifdef printFunc
	#define printFunc_normalizeArray
	#define printFunc_arraySum
	#define printFunc_estimateActivity
	#define printFunc_findPeaksInArray
	#define printFunc_specifyActivity
	#define printFunc_maxInArray
	#define printFunc_minInArray
#endif

//#define testRunTime
#ifdef testRunTime
	uint32_t sampleStart, sampleStop, procStart, procStop = 0;
	uint16_t sampleIndex = 0;
	uint32_t sampleAll[bufferSize];

	bool runningTest = false;

	void printSampleTime() {
		uint64_t allSampleTime = arraySum(sampleAll, 0, bufferSize);
		uint32_t maxSampleTime = maxInArray(sampleAll, bufferSize);
		Serial.print("Acerage sample time in microseconds:");
		Serial.println((uint32_t)allSampleTime/bufferSize);
		Serial.print("Max sample time in microseconds: ");
		Serial.println(maxSampleTime);
	}

	void printElapsedTime(uint32_t startTime, uint32_t stopTime) {
		Serial.print("Elapsed time in microseconds: ");
		Serial.println(stopTime - startTime);
	}
#endif

//#define countSteps
#ifdef countSteps
	uint16_t steps = 0;
	#ifndef useArrayData
		#define useArrayData
	#endif

#endif

//#define useArrayData
#ifdef useArrayData
	//#include "jacob_run_knee_omgang1_40sec.h"
	//#include "jacob_bike150_knee_omgang1_40sec.h"
	//#include "Jacob_run_knee_omgang1.h" // 166 skridt
	//#include "Jacob_walk_knee_omgang1.h" // 120 skridt	
	//#include "Kenneth_run_knee_omgang1.h" // 154 skridt	
	//#include "Kenneth_walk_knee_omgang1.h" // 122 skridt	
	//#include "Nikolaj_run_knee_omgang1.h" //  166 skridt	
	#include "Nikolaj_walk_knee_omgang1.h" //  166 skridt

	#define arrayDataSize 36866 + 12288 // Længde af 40 sec rå accl- og gyrodata array
	uint16_t testDataIndex = 12288; // Index tæller for rå data array

	//#define arrayDataSize 36865 + 9216 // Længde af 40 sec rå accl- og gyrodata array
	//uint16_t testDataIndex = 0; // Index tæller for rå data array


	bool getDataArrayM6(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz) {
		*ax = testData[testDataIndex]; // Accl x-akse
		*ay = testData[testDataIndex + 1]; // Accl y-akse
		*az = testData[testDataIndex + 2]; // Accl z-akse
		*gx = testData[testDataIndex + 3]; // Gyro x-akse
		*gy = testData[testDataIndex + 4]; // Gyro y-akse
		*gz = testData[testDataIndex + 5]; // Gyro z-akse
		testDataIndex = testDataIndex + 6; // Optæl tæller, til næste sæt af accl og gyro data
		if (testDataIndex > arrayDataSize) { // Stop data sampling hvis ende af array er nået
			return false;
		}
		return true;
	}
#endif

// #define useDataSerial
#ifdef useDataSerial
	#define printSerialData
	#include <HardwareSerial.h>
	HardwareSerial DataSerial(1);
	uint8_t s1RXpin = 17;
	uint8_t s1TXpin = 16;
	int16_t serialValue = 0;
	uint32_t serialBufferSize = 1024;

	// Debug functions
	void readTwoBytes(int16_t *val) {
		if (DataSerial.available() > 1) {
			byte lowbyte = DataSerial.read();
			*val = DataSerial.read();
			*val = ((*val)<<8) + lowbyte;
		}
	}

	// Debug functions
	void getAcclDataFromSerialM6(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz) {
		if (DataSerial.available() > 11) {
			readTwoBytes(ax);
			readTwoBytes(ay);
			readTwoBytes(az);
			readTwoBytes(gx);
			readTwoBytes(gy);
			readTwoBytes(gz);	
		}
	}

	void doPrintSerialData(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz, uint16_t dataIndex) {
		Serial.print("Data: ");
		Serial.print(*ax);
		Serial.print(" ");
		Serial.print(*ay);
		Serial.print(" ");
		Serial.print(*az);
		Serial.print(" ");
		Serial.print(*gx);
		Serial.print(" ");
		Serial.print(*gy);
		Serial.print(" ");
		Serial.print(*gz);
		Serial.print(" index: ");
		Serial.print(dataIndex);
		Serial.print(" avaliable: ");
		Serial.println(DataSerial.available());
	}
	#endif

#endif