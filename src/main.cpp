//----------------------------------------------
// Compiler definationer
// FreeRTOS kerne definationer
#if CONFIG_FREERTOS_UNICORE
	#define ARDUINO_RUNNING_CORE 0
#else
	#define ARDUINO_RUNNING_CORE 1
#endif

// FreeRTOS funktion inklusioner
#ifndef INCLUDE_vTaskSuspend
	#define INCLUDE_vTaskSuspend 1
#endif
#ifndef INCLUDE_vTaskDelayUntil
	#define INCLUDE_vTaskDelayUntil 1
#endif

// Detektions parametre
// Samplerate i Hz
#define fs 100
// Data buffer størrelse (skal kunne divideres med 2 for at FFT bibliotek virker)
#define bufferSize 512
// Single sided FFT buffer size
#define singleSize (bufferSize / 2) + 1
// Frekvens i Hz hvor over og under sammen skal sammenlignes
#define sumFreq 8
// Peak detektion threshold
#define acclPeakThreshold 0.65
#define gyroPeakThreshold 0.65 
// Peak detektion timeout
#define acclPeakTimeout 60
#define gyroPeakTimeout 15
// Rotationer per minut for hurtig cykling
#define fastBikeSPM 155
// Skridt per minut for løb
#define runSPM 130

// Pins to I2C kommunikation
#define SDA_pin 21
#define SCL_pin 20

// Debug funktionalitet
#define useDebug // Udkommenter dette for at slå alt debug fra
#ifdef useDebug
	#include "debuggerAndTester.h"
#endif

//----------------------------------------------
// Bibliotek inklusioner
#include <Arduino.h>
#include "FreeRTOS.h"
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "arduinoFFT.h"

//----------------------------------------------
// Variable definationer
// Aktivitets typer (som numbereret liste via enum)
typedef enum {
	UNKNOWN, // 0 - Ukendt aktivitet
	RUN_WALK, // 1 - Løb eller gang aktivitet
	RUN, // 2 - Løb
	WALK, // 3 - Gang
	BIKE, // 4 - Cykling med ukendt hastighed
	BIKE_SLOW, // 5 - Langsom cykling
	BIKE_FAST // 6 - Hurtig cykling
} activityTypes;

// Task til sampling af aktivitetsdata
void sampleActivityDataTask(void *pvParamaters);
// Task til detektion af aktivity og optlæning af skridt
void processActivityDataTask(void *pvParameters);

// Task handler til sample activity data task
TaskHandle_t sadTaskHandler;
// Task handler til handle activity data task
TaskHandle_t padTaskHandler;

// Antal akser som samples fra accelerometer og gyro tilsammen
#define imuAxis 6
// Lav sample data buffer
double acclRollingData[bufferSize];
// Lav statisk data buffer
double acclStaticData[bufferSize];
// Lav sample data buffer
double gyroRollingData[bufferSize];
// Lav statisk data buffer
double gyroStaticData[bufferSize];

//----------------------------------------------
// Initializering af klasse instancer
MPU6050 imu; // MPU-6050 klasse
arduinoFFT FFT = arduinoFFT(); // FFT klasse

//----------------------------------------------
// Funktionsdefinationer
// Finder absolutte værdi af komplex nummer gem i real array og imaginær array, gem i real
void absComplexArray(double *realPointer, double *imagPointer, uint16_t arrayLength) {
	for (int i=0; i < arrayLength; i++) {
		*(realPointer+i) = sqrt(pow(*(realPointer+i),2) + pow(*(imagPointer+i),2));
	}
}

// Normaliser array
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

// Beregn Pythagoras bevægelsesvektor fra x, y og z
double calculatePythagoras(int16_t x, int16_t y, int16_t z) {
	double pythagoras = (pow(x,2) + pow(y,2) + pow(z,2));
	pythagoras = sqrt(pythagoras);
	return pythagoras;
}

// Kopier int16_t array til double array
void copyArray(double *fromArray, double *toArray, int16_t arrayLength) {
	for (uint16_t i = 0; i < arrayLength; i++) {
		*(toArray+i) = *(fromArray+i);
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

// Estimer aktivitet baseret på summen af frekvensindhold over og under hzIndex
uint8_t estimateActivity(double *acclSumBelow, double *acclSumAbove, double *gyroSumBelow, double *gyroSumAbove) {
	if ((*acclSumAbove > *acclSumBelow) && (*gyroSumAbove > *gyroSumBelow)) {
		#ifdef printFunc_estimateActivity
			Serial.println("estimateActivity(), Activity is RUN_WALK");
		#endif
		return RUN_WALK;
	} else if ((*acclSumAbove < *acclSumBelow) && (*gyroSumAbove < *gyroSumBelow)) {
		#ifdef printFunc_estimateActivity
			Serial.println("estimateActivity(), Activity is BIKE");
		#endif
		return BIKE;
	}
	#ifdef printFunc_estimateActivity
		Serial.println("estimateActivity(), Activity is UNKNOWN");
	#endif
	return UNKNOWN;
}

// Find peaks i array
uint8_t findPeaksInArray(double *arrayPointer, uint16_t arrayLength, double threshold, uint16_t timeout) {
	uint8_t peakCount = 0;
	int32_t lastPeakIndex = -timeout; 
	for (uint16_t i = 1; i > arrayLength - 1; i++) {
		// Tjek om timeout er overstået
		if ((i - lastPeakIndex) > timeout) {
			// Tjek om over threshold
			if (*(arrayPointer+i) > threshold) {
				// Tjek om værdi er mindre før og efter
				if ((*(arrayPointer+i)) >= (*(arrayPointer+i+1)) && (*(arrayPointer+i)) > (*(arrayPointer+i-1))) {
					// Optæl antal peaks
					peakCount++;
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

// Korriger tidligere estimeret aktivitet
uint8_t specifyActivity(uint8_t activity, int16_t peaks) {
	#ifdef printFunc_specifyActivity
		Serial.print("specifyActivity(), Specified activity is: ");
	#endif
	switch (activity) {
		case BIKE: {
			float spinsPerMin = (peaks / 5) * 60;
			if (spinsPerMin > fastBikeSPM) {
				#ifdef printFunc_specifyActivity
					Serial.println("FAST BIKE");
				#endif
				activity = BIKE_FAST;
			} else {
				#ifdef printFunc_specifyActivity
					Serial.println("SLOW BIKE");
				#endif				
				activity = BIKE_SLOW;
			}
		} break;
		case RUN_WALK: {
			float stepsPerMin = ((peaks * 2) / 5) * 60;
			if (stepsPerMin > runSPM) {
				#ifdef printFunc_specifyActivity
					Serial.println("RUN");
				#endif				
				activity = RUN;
			} else {
				#ifdef printFunc_specifyActivity
					Serial.println("WALK");
				#endif				
				activity = WALK;
			}
		} break;
	}
	return activity;
}

// Finder maksimal værdi i array
double maxInArray(double *buffPointer, uint16_t buffSize) {
	double maxValue = *(buffPointer);
	#ifdef printFunc_maxInArray
		uint16_t maxIndex = 0;
	#endif
	for (uint16_t i = 1; i < buffSize; i++) {
		if (maxValue < *(buffPointer+i)) {
			maxValue = *(buffPointer+i);
			#ifdef printFunc_maxInArray
				maxIndex = i;
			#endif
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
		uint16_t minIndex = 0;
	#endif
	for (uint16_t i = 1; i < buffSize; i++) {
		if (minValue > *(buffPointer+i)) {
			minValue = *(buffPointer+i);
			#ifdef printFunc_maxInArray
				minIndex = i;
			#endif
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

// Sæt alle værdier i array til given værdi
void setArrayTo(double *arrayPointer, uint16_t arrayLength, double value) {
	for (uint16_t i = 0; i < arrayLength; i++) {
		*(arrayPointer+i) = 0;
	}
}

// Udfør FFT på data og kopier absolut single sided FFT til absFFTout array (output array (absFFTout) længde er (arrayLength/2)+1)
void getAbsoluteSingleFFT(double *rawDataIn, double *absFFTout, uint16_t arrayLength) {
	// FFT reel og imaginær array
	double realFFT[arrayLength];
	double imagFFT[arrayLength];
	// Kopier data til FFT array
	copyArray(rawDataIn, realFFT, arrayLength); 
	// Reset alle værdier i imagFFT (nødvendigt ifølge arduinoFFT bibliotek)
	setArrayTo(imagFFT, arrayLength, 0); 
	// Udfør FFT
	FFT.Compute(realFFT, imagFFT, arrayLength, FFT_FORWARD);
	// Beregn absolute værdi af FFT 
	absComplexArray(realFFT, imagFFT, arrayLength); 
	// Kopier data til FFT array
	copyArray(&realFFT[1], absFFTout, (arrayLength / 2) + 1); 
}

// Initialiser IMU, set samplerate og akserange
void setupIMU() {
	// Start Wire bibkiotek til I2C kommunikation
	Wire.begin(SDA_pin, SCL_pin); 
	// Start IMU ( MPU-6050) instance
	imu.initialize();
	// Slå digital lavpasfilter fra
	imu.setDLPFMode(0);
	// Sæt samplerate (7 => 8kHz/(7+1) = 1kHz)
	imu.setRate(7);
	// Sæt range af accelerometer (3 => +/- 16g)
	imu.setFullScaleAccelRange(3);
	// Sæt range af gyroskop (2 => +/- 1000 deg/s)
	imu.setFullScaleGyroRange(2);
}

//----------------------------------------------
// Setup
void setup() {
	// Start serial kommunikation
	#ifdef useDebug
		Serial.begin(115200);
		// Data loader debug
		#ifdef useDataSerial
			DataSerial.begin(115200, SERIAL_8N1, s1RXpin, s1TXpin);
			DataSerial.setRxBufferSize(serialBufferSize);
		#endif
	#endif
	// Setup IMU
	setupIMU();

	//  INDSÆT BLUETOOTH

	// Lav data behandler task
	xTaskCreate(
		sampleActivityDataTask,
		"Sample activity data",
		2048, // Hukommelses mængde
		NULL,
		2, // Priotitet
		&sadTaskHandler // Håndtag til task
	);
	// Lav data behandler task
	xTaskCreate(
		processActivityDataTask,
		"Activity data handler",
		16384,
		NULL,
		1, // Priotitet
		&padTaskHandler // Håndtag til task
	);
	// Lad begge tasks blive færdig med setup
	delay(2);

	vTaskResume(sadTaskHandler);

	// bool waitingForStartByte = true;
	// while(waitingForStartByte) {
	// 	if (waitingForStartByte) {
	// 		if (DataSerial.available() > 0) {
	// 			int8_t input = DataSerial.read();
	// 			Serial.write(input);
	// 			if (input == 101) {
	// 				waitingForStartByte = false;
	// 				Serial.println("Starting!");
	// 				vTaskResume(sadTaskHandler);
	// 			}
	// 		}
	// 	}
	// }
}

//----------------------------------------------
// Main loop
void loop() {

		// Indsæt bluetooth kode til læsning

		// TaskStatus_t sampleTaskStatus = eTaskGetState(sadTaskHandler);
		// if (sampleTaskStatus == eSuspended) {
		// 	vTaskResume(sadTaskHandler);
		// }
}

/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/
// Task til sampling af aktivitetsdata
void sampleActivityDataTask(void *pvParamaters) {
	// Task setup
	Serial.println("Setting up sampling task");
	// Tick frekvens af task (en tick er 1 ms, 100 Hz svarer til en gang per 10 ms, derfor tickrate på 10.). Omregn fs til ms ved 1/fs, derefter omregn til ms.
	//const TickType_t frequency = ((1/(float)fs)*1000);
	const TickType_t frequency = 10;
	// Accelerometer og gyroskop data pladsholdere
	int16_t ax, ay, az, gx, gy, gz;
	// Tæller til at holde styr på nuværende index af data buffer
	uint16_t dataIndex = 0;
	// Sæt task på pause indtil start signal gives
	vTaskSuspend(NULL);	
	// Gem nuværende tid
	TickType_t lastWakeTime = xTaskGetTickCount();
	// Task loop
	for (;;) {
		#if defined(useDataSerial)
			// Simuler sampling af data via Serial
			getAcclDataFromSerialM6(&ax, &ay, &az, &gx, &gy, &gz);
			#ifdef printSerialData
				doPrintSerialData(&ax, &ay, &az, &gx, &gy, &gz, dataIndex);
			#endif
		#elif defined(useArrayData)
			if(!getArrayDataM6(&ax, &ay, &az, &gx, &gy, &gz)) {
				vTaskSuspend(NULL);
			}
		#else
			// Sampler data via IMU
			imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
		#endif

		Serial.print(ax);
		Serial.print(" ");
		Serial.print(ay);
		Serial.print(" ");
		Serial.print(az);
		Serial.print(" ");
		Serial.print(gx);
		Serial.print(" ");
		Serial.print(gy);
		Serial.print(" ");
		Serial.println(gz);								

		// Beregn produktet af det samplede værdier
		acclRollingData[dataIndex] = calculatePythagoras(ax, ay, az);
		gyroRollingData[dataIndex] = calculatePythagoras(gx, gy, gz);
		
		Serial.println(acclRollingData[dataIndex]);
		Serial.println(gyroRollingData[dataIndex]);
		
		vTaskSuspend(NULL);

		// Tjek om ny min eller max
		// isNewMin(acclRollingData[dataIndex], gyroRollingData[dataIndex]);
		// isNewMax(acclRollingData[dataIndex], gyroRollingData[dataIndex]);
		// Tjek om det er tid til at lave databehandling
		if (dataIndex >= bufferSize - 1) {
			// Sæt data index tilbage til 0
			dataIndex = 0;
			// Kopier min og max
			// copyArray(acclRollingMinMax, acclStaticMinMax, 2);
			// copyArray(gyroRollingMinMax, gyroStaticMinMax, 2);
			// Sæt min og max tilbage til nul
			// resetMinMax();
			// Kopier alt data fra rolling buffer til static buffer
			copyArray(acclRollingData, acclStaticData, bufferSize);
			copyArray(gyroRollingData, gyroStaticData, bufferSize);
			
			// isIdentical(acclRollingData, acclStaticData, bufferSize);
			// isIdentical(gyroRollingData, gyroStaticData, bufferSize);
						
			
			// Genstart data processing task
			vTaskResume(padTaskHandler);
			#ifdef useDataSerial
				vTaskSuspend(NULL);
			#endif
		} else {
			// Opsæt data index
			dataIndex++;
		}
		#ifdef useDataSerial
			vTaskDelay(50);
		#else
			// Vent indtil der skal samples igen
			vTaskDelayUntil(&lastWakeTime, frequency);
		#endif
	}
}

// Task til detektion af aktivity og optlæning af skridt
void processActivityDataTask(void *pvParameters) {
	// Task setup
	Serial.println("Setting up data processing task");
	// Forbered tæller af antal peaks
	uint8_t peakCount = 0;
	// Forbered aktivitets indikator
	uint8_t activity = 0;
	// FFT data array
	double acclSingleFFT[singleSize];
	double gyroSingleFFT[singleSize];
	// Min og max variabler
	double dataMax;
	double dataMin;
	// FFT index hvoromkring summen af FFT skal beregnes
	uint16_t fftIndexSummed = ((bufferSize / fs) * sumFreq);
	// Sæt task på pause indtil den skal bruges
	vTaskSuspend(NULL);
	// Task loop
	for (;;) {
		#ifdef useDebug
			Serial.println("Running data processing");
		#endif
		// Find min og max i accelerometerdata
		dataMax = maxInArray(acclStaticData, bufferSize);
		dataMin = minInArray(acclStaticData, bufferSize);
		// Normaliser accelerometerdata
		normalizeArray(acclStaticData, bufferSize, dataMin, dataMax);
		// Find min og max i gyroskop data
		dataMax = maxInArray(gyroStaticData, bufferSize);
		dataMin = minInArray(gyroStaticData, bufferSize);
		// Normaliser gyroskopdata
		normalizeArray(gyroStaticData, bufferSize, dataMin, dataMax);
		// Beregn FFT af data
		getAbsoluteSingleFFT(acclStaticData, acclSingleFFT, bufferSize);
		getAbsoluteSingleFFT(gyroStaticData, gyroSingleFFT, bufferSize);
		// Find sum under hzIndex og over hzIndex
		double acclBelowSum = arraySum(acclSingleFFT, 4, fftIndexSummed + 1);
		double acclAboveSum = arraySum(acclSingleFFT, fftIndexSummed + 2, singleSize);
		double gyroBelowSum = arraySum(gyroSingleFFT, 4, fftIndexSummed + 1);
		double gyroAboveSum = arraySum(gyroSingleFFT, fftIndexSummed + 2, singleSize);
		// Gæt aktivitet baseret på FFT
		activity = estimateActivity(&acclBelowSum, &acclAboveSum, &gyroBelowSum, &gyroAboveSum);
		// Find peaks i data
		switch (activity) {
			case RUN_WALK: {
				peakCount = findPeaksInArray(acclStaticData, bufferSize, acclPeakThreshold, acclPeakTimeout);
			} break;
			case BIKE: {
				peakCount = findPeaksInArray(gyroStaticData, bufferSize, gyroPeakThreshold, gyroPeakTimeout);
			} break;
		}
		// Korriger aktivitet
		activity = specifyActivity(activity, peakCount);

		#ifdef useDebug
			Serial.print("Activity is: ");
			Serial.println(activity);
		#endif

		#ifdef useDebug
			Serial.println("Data processing done");
			Serial.println();
		#endif

		// Klargør task til næste data processering
		peakCount = 0;
		activity = 0;
		//Suspend task indtil der er ny data klar
		vTaskSuspend(NULL);
	}
}