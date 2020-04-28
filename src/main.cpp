#if CONFIG_FREERTOS_UNICORE
#define ARDUINO_RUNNING_CORE 0
#else
#define ARDUINO_RUNNING_CORE 1
#endif

#ifndef INCLUDE_vTaskSuspend
#define INCLUDE_vTaskSuspend 1
#endif

#include <Arduino.h>

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

// Detektions parametre
// Data buffer størrelse
#define bufferSize 500
// FFT single sided buffer size
#define singleFFTbufferSize 250
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

// Antal akser som samples fra accelerometer og gyro tilsammen
#define imuAxis 6
// Lav sample data buffer
float acclRollingData[bufferSize];
// Lav statisk data buffer
float acclStaticData[bufferSize];
// Lav sample data buffer
float gyroRollingData[bufferSize];
// Lav statisk data buffer
float gyroStaticData[bufferSize];

#define sMIN 0
#define sMAX 1
// Static accelerometer min og max
int16_t acclStaticMinMax[2] = {0, 0}; // [Min, Max]
// Static gyroskop min og max
int16_t gyroStaticMinMax[2] = {0, 0}; // [Min, Max]

// Finder maksimal værdi i array
float maxInArray(float *buffPointer, uint16_t buffSize) {
	float maxValue = *buffPointer;
	for (uint16_t i = 1; i < buffSize; i++) {
		if (maxValue < *(buffPointer+i)) {
			maxValue = *(buffPointer+i);
		}
	}
	return maxValue;	
}

// Finder minimal værdi i array
float minInArray(float *buffPointer, uint16_t buffSize) {
	float minValue = *buffPointer;
	for (uint16_t i = 1; i < buffSize; i++) {
		if (minValue > *(buffPointer+i)) {
			minValue = *(buffPointer+i);
		}
	}
	return minValue;
}

// Finder absolutte værdi i array
void absArray(float *buffPointer, uint16_t buffSize) {
	for (int i=0; i < buffSize; i++) {
		*(buffPointer+i) = abs(*(buffPointer+i));
	}
}

void normalizeArray(float *buffPointer, uint16_t buffSize, float minValue, float maxValue){
	//float maxValue = maxInArray(buffPointer, buffSize);
	//float minValue = minInArray(buffPointer, buffSize);
	for(int i=0; i<buffSize; i++) {
		*(buffPointer+i) = (*(buffPointer+i) - minValue) / (maxValue - minValue);
	}
}

float calculatePythagoras(int32_t x, int32_t y, int32_t z) {
	uint32_t product = (pow(x,2) + pow(y,2) + pow(z,2));
	return sqrt(product);
}

void copyArrayData(float *fromArray, float *toArray, int16_t arrayLength) {
	for (uint16_t i = 0; i < arrayLength; i++) {
		*(toArray+i) = *(fromArray+i);
	}
}

// Bereng summen af alle datapunkter i et array fra startIndex til endIndex. endIndex bør ikke overstige størrelsen af arrayet. startindex er inklusiv, endIndex er eksklusiv
float arraySum(float *arrayPointer, uint16_t startIndex, uint16_t endIndex) {
	float sumOfArray = 0;
	for (uint16_t i = startIndex; i < endIndex; i++) {
		sumOfArray = sumOfArray + *(arrayPointer+i);
	}
	return sumOfArray;
}

// Estimer aktivitet baseret på summen af frekvensindhold over og under hzIndex
uint8_t estimateActivity(float acclSumBelow, float acclSumAbove, float gyroSumBelow, float gyroSumAbove) {
	if ((acclSumAbove > acclSumBelow) and (gyroSumAbove > gyroSumBelow)) {
		return RUN_WALK;
	} else if ((acclSumAbove < acclSumBelow) and (gyroSumAbove < gyroSumBelow)) {
		return BIKE;
	}
	return UNKNOWN;
}

// Find peaks i array
uint8_t findPeaksInArray(float *arrayPointer, uint16_t arrayLength, float threshold, uint16_t timeout) {
	uint8_t peakCount = 0;
	int32_t lastPeakIndex = -timeout; 
	for (uint16_t i = 1; i > arrayLength - 1; i++) {
		// Tjek om timeout er overstået
		if ((i - lastPeakIndex) > timeout) {
			// Tjek om over threshold
			if (*(arrayPointer+i) > threshold) {
				// Tjek om værdi er mindre før og efter
				if ((*(arrayPointer+i)) > (*(arrayPointer+i+1)) and (*(arrayPointer+i)) > (*(arrayPointer+i-1))) {
					// Optæl antal peaks
					peakCount++;
				}
			}
		}
	}
	return peakCount;
}

// Korriger tidligere estimeret aktivitet
uint8_t specifyActivity(uint8_t activity, int16_t peaks) {
	switch (activity) {
		case BIKE: {
			float spinsPerMin = (peaks / 5) * 60;
			if (spinsPerMin > fastBikeSPM) {
				activity = BIKE_FAST;
			} else {
				activity = BIKE_SLOW;
			}
		} break;
		case RUN_WALK: {
			float stepsPerMin = ((peaks * 2) / 5) * 60;
			if (stepsPerMin > runSPM) {
				activity = RUN;
			} else {
				activity = WALK;
			}
		} break;
	}
	return activity;
}

// Tjek om ny accl og gyro værdi er ny maks
void isNewMax(float newAccl, float newGyro) {
	// Acclerometer
	if (newAccl > acclStaticMinMax[sMAX]) {
		// Gem ny max værdi
		acclStaticMinMax[sMAX] = newAccl;
	}
	// Gyroskop
	if (newGyro > gyroStaticMinMax[sMAX]) {
		// Gem ny max værdi
		gyroStaticMinMax[sMAX] = newGyro;
	}
}

// Tjek om ny accl og gyro værdi er ny min
void isNewMin(float newAccl, float newGyro) {
	// Acclerometer
	if (newAccl < acclStaticMinMax[sMIN]) {
		// Gem ny min værdi
		acclStaticMinMax[sMIN] = newAccl;
	}
	// Gyroskop
	if (newGyro < gyroStaticMinMax[sMIN]) {
		// Gem ny min værdi
		gyroStaticMinMax[sMIN] = newGyro;
	}	
}

// Reset min og max pladsholder
void resetMinMax() {
	// Reset accelerometer min og max
	acclStaticMinMax[sMIN] = 0;
	acclStaticMinMax[sMAX] = 0;
	// Reset gyroskop min og max
	gyroStaticMinMax[sMIN] = 0;
	gyroStaticMinMax[sMAX] = 0;
}

// Debug functions
void readTwoBytes(int16_t *val) {
	if (Serial.available() > 1) {
		byte lowbyte = Serial.read();
		*val = Serial.read();
		*val = ((*val)<<8) + lowbyte;
	}
}

// Debug functions
void getAcclDataFromSerialM6(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz) {
	if (Serial.available() >= 12) {
		readTwoBytes(ax);
		readTwoBytes(ay);
		readTwoBytes(az);
		readTwoBytes(gx);
		readTwoBytes(gy);
		readTwoBytes(gz);	
	} else {
		*ax = 0; *ay = 0; *az = 0;
		*gx = 0; *gy = 0; *gz = 0;
	}
}

// Setup
void setup() {

	Serial.begin(115200);

	// Lav data behandler task
	xTaskCreate(
		sampleActivityDataTask,
		"Sample activity data",
		1024, // Hukommelses mængde
		NULL,
		2, // Priotitet
		&sadTaskHandler // Håndtag til task
	);
	// Lav data behandler task
	xTaskCreate(
		processActivityDataTask,
		"Activity data handler",
		8192,
		NULL,
		1, // Priotitet
		&padTaskHandler // Håndtag til task
	);

	delay(100);
	vTaskResume(sadTaskHandler);
}

// Main loop
void loop() {

}

/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/
// Task til sampling af aktivitetsdata
void sampleActivityDataTask(void *pvParamaters) {
	// Task setup
	Serial.println("Setting up sampling task");
	// Tick frekvens af task
	const TickType_t frequency = 100;
	// Gem nuværende tid
	TickType_t lastWakeTime = xTaskGetTickCount();
	// Accelerometer og gyroskop data pladsholdere
	int16_t ax, ay, az, gx, gy, gz;
	// Tæller til at holde styr på nuværende index af data buffer
	uint16_t dataIndex = 0;
	// Sæt task på pause indtil start signal gives
	vTaskSuspend(NULL);	
	// Task loop
	for (;;) {
		// Sample data, ÆNDRE DENNE TIL MPU6050 GETMOTION6
		getAcclDataFromSerialM6(&ax, &ay, &az, &gx, &gy, &gz);
		// Beregn produktet af det samplede værdier
		acclRollingData[dataIndex] = calculatePythagoras(ax, ay, az);
		gyroRollingData[dataIndex] = calculatePythagoras(ax, ay, az);
		// Tjek om ny min eller max
		isNewMin(acclRollingData[dataIndex], gyroRollingData[dataIndex]);
		isNewMax(acclRollingData[dataIndex], gyroRollingData[dataIndex]);
		// Tjek om det er tid til at lave databehandling
		if (dataIndex >= bufferSize - 1) {
			// Sæt data index tilbage til 0
			dataIndex = 0;
			// Sæt min og max tilbage til nul
			resetMinMax();
			// Kopier alt data fra rolling buffer til static buffer
			copyArrayData(acclRollingData, acclStaticData, bufferSize);
			copyArrayData(gyroRollingData, gyroStaticData, bufferSize);
			// Genstart data processing task
			vTaskResume(padTaskHandler);
		} else {
			// Opsæt data index
			dataIndex++;
		}
		// Vent indtil der skal samples igen
		vTaskDelayUntil(&lastWakeTime, frequency);
		//vTaskDelay(5 / portTICK_RATE_MS);
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
	// 

	// Sæt task på pause indtil den skal bruges
	vTaskSuspend(NULL);
	// Task loop
	for (;;) {
		Serial.println("Running data processing");
		// Normaliser accelerometer- og gyroskopdata
		normalizeArray(acclStaticData, bufferSize, acclStaticMinMax[sMIN], acclStaticMinMax[sMAX]);
		normalizeArray(gyroStaticData, bufferSize, gyroStaticMinMax[sMIN], gyroStaticMinMax[sMAX]);
		// FFT data
		float acclFFT[singleFFTbufferSize];
		float gyroFFT[singleFFTbufferSize];


		// Fjern DC fra FFT

		// Find sum under hzIndex og over hzIndex
		float acclBelowSum = arraySum(acclFFT, 0, sumFreq);
		float acclAboveSum = arraySum(acclFFT, sumFreq, singleFFTbufferSize);
		float gyroBelowSum = arraySum(gyroFFT, 0, sumFreq);
		float gyroAboveSum = arraySum(gyroFFT, sumFreq, singleFFTbufferSize);
		// Gæt aktivitet baseret på FFT
		activity = estimateActivity(acclBelowSum, acclAboveSum, gyroBelowSum, gyroAboveSum);
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


		Serial.print("Activity is: ");
		Serial.println(activity);

		Serial.println("Data processing done");

		// Klargør task til næste data processering
		peakCount = 0;
		activity = 0;
		//Suspend task indtil der er ny data klar
		vTaskSuspend(NULL);
	}
}