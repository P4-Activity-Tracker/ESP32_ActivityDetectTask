//----------------------------------------------
// Bibliotek inklusioner til MPU6050
#include <Arduino.h>
#include "FreeRTOS.h"
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "arduinoFFT.h"
#include "slaveBLE.h"
#include "arrayProcessing.h"

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
const uint16_t singleSize = (bufferSize / 2);
// Frekvens i Hz hvor over og under sammen skal sammenlignes
#define sumFreq 8
// FFT index hvoromkring summen af FFT skal beregnes
const uint16_t fftIndexSummed = (((bufferSize / fs) * sumFreq) + 1);
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
#define SCL_pin 22

// Debug funktionalitet
#define useDebug // Udkommenter dette for at slå alt debug fra
#ifdef useDebug
	#include "debuggerAndTester.h"
#endif

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

void startSampleTask () {
	#ifdef testRunTime
		if (!runningTest) {
	#endif
	vTaskResume(sadTaskHandler);
	#ifdef testRunTime
		}
	#endif
}

void stopSampleTask () {
	#ifdef testRunTime
		if (!runningTest) {
	#endif
	vTaskSuspend(sadTaskHandler);
	#ifdef testRunTime
		}
	#endif
}

// Beregn Pythagoras bevægelsesvektor fra x, y og z
double calculatePythagoras(int16_t x, int16_t y, int16_t z) {
	double pythagoras = (pow(x,2) + pow(y,2) + pow(z,2));
	pythagoras = sqrt(pythagoras);
	return pythagoras;
}

// Udfør FFT på data og kopier absolut single sided FFT til absFFTout array (output array (absFFTout) længde er (arrayLength/2)+1)
void getAbsoluteSingleFFT(double *rawDataIn, double *absFFTout, uint16_t arrayLength) {
	// FFT reel og imaginær array
	double realFFT[bufferSize];
	double imagFFT[bufferSize];
	// Kopier data til FFT array
	copyArray(rawDataIn, realFFT, arrayLength); 
	// Reset alle værdier i imagFFT (nødvendigt ifølge arduinoFFT bibliotek)
	setArrayTo(imagFFT, arrayLength, 0); 
	// Udfør FFT
	FFT.Compute(realFFT, imagFFT, arrayLength, FFT_FORWARD);
	// Beregn absolute værdi af FFT 
	absComplexArray(realFFT, imagFFT, arrayLength); 
	// Kopier data til FFT array
	copyArray(realFFT, absFFTout, (arrayLength / 2)); 
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

// Korriger tidligere estimeret aktivitet
uint8_t specifyActivity(uint8_t activity, int16_t peaks) {
	#ifdef printFunc_specifyActivity
		Serial.print("specifyActivity(), ");
		Serial.print(" new steps/spins: ");
		Serial.println(peaks);
	#endif
	switch (activity) {
		case BIKE: {
			float spinsPerMin = ((float)peaks / 5.0f) * 60.0f;
			#ifdef printFunc_specifyActivity
				Serial.print("specifyActivity(), Spins per min is: ");
				Serial.print(spinsPerMin);
				Serial.print(" , percise activity is: ");
			#endif	
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
			float stepsPerMin = (((float)peaks * 2.0f) / 5.0f) * 60.0f;
			#ifdef printFunc_specifyActivity
				Serial.print("specifyActivity(), Steps per min is: ");
				Serial.print(stepsPerMin);
				Serial.print(" , specified activity is: ");
			#endif	
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
		default: {
			#ifdef printFunc_specifyActivity
				Serial.print("Steps per min is: ");
				Serial.print(0);
				Serial.print(" , specified activity is: ");
				Serial.println("UNKNOWN");
			#endif	
		} break;
	}
	return activity;
}

// funktion som sørger for at konvertere heltal til en string af længden characters. 
String dataToCharacters (int32_t data ,uint8_t characters){ //input data kan bestå af HR, EE, antal skridt, og characters beskriver antallet af karaktere i output string. 
	String dataS = String(data); // Ex: 12 skridt -> "12"
	for (uint8_t i = dataS.length(); i < characters ; i++){ // Ex: længde 2, ønskede karaktere 5, mangele 3 nuller
		dataS = "0" + dataS; // Tilfæl nul foran de andre karaktere
	}
	return dataS; // Ex: "00012"
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
//----------------------------------------------
void setup() {
	// Start serial kommunikation
	#ifdef useDebug
		Serial.begin(115200);
	#endif
	#ifndef useArrayData
		// Setup IMU
		setupIMU();
		// Funktioner pointers
		startSampleFuncPointer = startSampleTask;
		stopSampleFuncPointer = stopSampleTask;
		// Prøv at forbid til BlE server
		BLEDevice::init("");
		// Vent på BLE forbindelse
		while (!connectToServer()) {
			delay(5000);
		}  
	#endif
	// Lav data behandler task
	xTaskCreate(
		sampleActivityDataTask,
		"Sample activity data",
		4096, // Hukommelses mængde
		NULL,
		19, // Priotitet
		&sadTaskHandler // Håndtag til task
	);
	// Lav data behandler task
	xTaskCreate(
		processActivityDataTask,
		"Activity data handler",
		16384,
		NULL,
		5, // Priotitet
		&padTaskHandler // Håndtag til task
	);
	// Lad begge tasks blive færdig med setup
	delay(2);
	#ifdef useArrayData
		vTaskResume(sadTaskHandler); // Start data sampling (til debug)
	#endif
}

//----------------------------------------------
// Main loop
//----------------------------------------------
void loop() {
	delay(5000);
	if (!connected) { // Prøv at forbinde til server, hvis ikke forbundet
		connectToServer();
	}
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
		#ifdef testRunTime
			runningTest = true;
			sampleStart = micros();
		#endif
		#if defined(useArrayData)
			// Brug data fil
			if(!getDataArrayM6(&ax, &ay, &az, &gx, &gy, &gz)) {
				vTaskSuspend(NULL);
			}
		#else
			// Sampler data via IMU
			imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
		#endif
		// Beregn produktet af det samplede værdier
		acclRollingData[dataIndex] = calculatePythagoras(ax, ay, az);
		gyroRollingData[dataIndex] = calculatePythagoras(gx, gy, gz);
		// Tjek om det er tid til at lave databehandling
		if (dataIndex >= (bufferSize - 1)) {
			// Sæt data index tilbage til 0
			dataIndex = 0;
			// Kopier alt data fra rolling buffer til static buffer
			copyArray(acclRollingData, acclStaticData, bufferSize);
			copyArray(gyroRollingData, gyroStaticData, bufferSize);
			// Genstart data processing task
			vTaskResume(padTaskHandler);
			#ifdef testRunTime
				// Stop samplings task hvis der testes eksekveringstid
				vTaskSuspend(NULL);
			#endif
		} else {
			// Opsæt data index
			dataIndex++;
		}
		#ifdef testRunTime
			sampleStop = micros();
			sampleAll[sampleIndex] = sampleStop - sampleStart;
			sampleIndex = sampleIndex + 1;
		#endif
		// Vent indtil der skal samples igen
		vTaskDelayUntil(&lastWakeTime, frequency);
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
	// Accelerometer og gyroskop single FFT array
	double acclSingleFFT[singleSize];
	double gyroSingleFFT[singleSize];
	// Sæt task på pause indtil den skal bruges
	vTaskSuspend(NULL);
	// Task loop
	for (;;) {
		#ifdef testRunTime
			procStart = micros();
		#endif
		#ifdef useDebug
			//Serial.println("Running data processing");
		#endif
		// Find max i accelerometerdata
		double dataMax = maxInArray(acclStaticData, bufferSize);
		// Find threshold for accelerometerdata
		double acclThreshold = dataMax * acclPeakThreshold;
		// Find max i gyroskop data
		dataMax = maxInArray(gyroStaticData, bufferSize);
		// Find threshold for accelerometerdata
		double gyroThreshold = dataMax * gyroPeakThreshold;
		// Beregn FFT af data
		getAbsoluteSingleFFT(acclStaticData, acclSingleFFT, bufferSize);
		getAbsoluteSingleFFT(gyroStaticData, gyroSingleFFT, bufferSize);
		// Fjern DC fra FFT
		setArrayTo(acclSingleFFT, 5, 0); 
		setArrayTo(gyroSingleFFT, 5, 0); 
		// Find sum under hzIndex og over hzIndex
		double acclBelowSum = arraySum(acclSingleFFT, 0, fftIndexSummed);
		double acclAboveSum = arraySum(acclSingleFFT, fftIndexSummed, singleSize);
		double gyroBelowSum = arraySum(gyroSingleFFT, 0, fftIndexSummed);
		double gyroAboveSum = arraySum(gyroSingleFFT, fftIndexSummed, singleSize);
		// Gæt aktivitet baseret på FFT
		activity = estimateActivity(&acclBelowSum, &acclAboveSum, &gyroBelowSum, &gyroAboveSum);
		// Find peaks i data
		switch (activity) {
			case RUN_WALK: {
				peakCount = findPeaksInArray(acclStaticData, bufferSize, acclThreshold, acclPeakTimeout);
			} break;
			case BIKE: {
				peakCount = findPeaksInArray(gyroStaticData, bufferSize, gyroThreshold, gyroPeakTimeout);
			} break;
		}
		// Korriger aktivitet
		activity = specifyActivity(activity, peakCount);
		#ifndef useArrayData
			// Her benyttes fuktionen "dataToCharacters" til activity og peakcount, hvor det gemmes i "dataOut" som derefter skrives til med funktionen "writeToServer"
			String dataOut = dataToCharacters(activity,1) + dataToCharacters(peakCount,2);
			writeToServer(dataOut);
		#endif
		#ifdef useDebug
			//Serial.println("Data processing done");
			//Serial.println();
		#endif
		// Klargør task til næste data processering
		peakCount = 0;
		activity = 0;
		#ifdef testRunTime
			procStop = micros();
			printSampleTime();
			Serial.println("Processing task time:");
			printElapsedTime(procStart, procStop);
		#endif
		//Suspend task indtil der er ny data klar
		vTaskSuspend(NULL);
	}
}