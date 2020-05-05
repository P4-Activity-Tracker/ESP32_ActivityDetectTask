#ifndef _debuggerAndTester_
#define _debuggerAndTester_

#include "Arduino.h"

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

#define useArrayData
#ifdef useArrayData
	#include "jacob_bike150_knee.h"
	// #include "jacob_run_knee.h"

	#define arrayDataSize 30006

	uint16_t testDataIndex = 0;

	bool getArrayDataM6(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz) {
		*ax = testData[testDataIndex];
		*ay = testData[testDataIndex + 1];
		*az = testData[testDataIndex + 2];
		*gx = testData[testDataIndex + 3];
		*gy = testData[testDataIndex + 4];
		*gz = testData[testDataIndex + 5];

		testDataIndex = testDataIndex + 6;
		if (testDataIndex > arrayDataSize) {
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