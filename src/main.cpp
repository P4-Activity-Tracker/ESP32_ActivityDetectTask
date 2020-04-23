#if CONFIG_FREERTOS_UNICORE
#define ARDUINO_RUNNING_CORE 0
#else
#define ARDUINO_RUNNING_CORE 1
#endif

#ifndef INCLUDE_vTaskSuspend
#define INCLUDE_vTaskSuspend 1
#endif

#include <Arduino.h>

// Task til sampling af aktivitetsdata
void sampleActivityDataTask(void *pvParamaters);
// Task til detektion af aktivity og optlæning af skridt
void handleActivityDataTask(void *pvParameters);

// Task handler til sample activity data task
TaskHandle_t sadTaskHandler;
// Task handler til handle activity data task
TaskHandle_t hadTaskHandler;

// Data buffer størrelse
#define bufferSize 500
// Antal akser som samples fra accelerometer og gyro tilsammen
#define imuAxis 6
// Lav sample data buffer
int16_t acclRollingData[bufferSize];
// Lav statisk data buffer
float acclStaticData[bufferSize];
// Lav sample data buffer
int16_t gyroRollingData[bufferSize];
// Lav statisk data buffer
float gyroStaticData[bufferSize];
// Tæller til at holde styr på nuværende index af data buffer
uint16_t dataIndex = 0;
// Indikator for at det er tid til at lave databehandling
bool runActivityDataHandler = false;

// Finder maksimal værdi i array
int16_t maxInArray(float *buffPointer, uint16_t buffSize) {
	int16_t maxValue = *buffPointer;
	for (uint16_t i = 1; i < buffSize; i++) {
		if (maxValue < *(buffPointer+i)) {
			maxValue = *(buffPointer+i);
		}
	}
	return maxValue;	
}

// Finder minimal værdi i array
int16_t minInArray(float *buffPointer, uint16_t buffSize) {
	int16_t minValue = *buffPointer;
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

void normalizeArray(float *buffPointer, uint16_t buffSize){
	int16_t maxValue = maxInArray(buffPointer, buffSize);
	int16_t minValue = minInArray(buffPointer, buffSize);
	for(int i=0; i<buffSize; i++) {
		*(buffPointer+i) = (*(buffPointer+i) - minValue) / (maxValue - minValue);
	}
}


float calculatePythagoras(int32_t x, int32_t y, int32_t z) {
	uint32_t product = (pow(x,2) + pow(y,2) + pow(z,2));
	return sqrt(product);
}

void setup() {

	Serial.begin(115200);

	// Lav data behandler task
	xTaskCreate(
		sampleActivityDataTask,
		"Sample activity data",
		1024,
		NULL,
		2,
		&sadTaskHandler
	);
	// Lav data behandler task
	xTaskCreate(
		handleActivityDataTask,
		"Activity data handler",
		8192,
		NULL,
		1,
		&hadTaskHandler
	);

	delay(100);
	vTaskResume(sadTaskHandler);
}

void loop() {
  // put your main code here, to run repeatedly:
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
	// Sæt task på pause indtil start signal gives
	vTaskSuspend(NULL);
	// Task loop
	for (;;) {
		// Sample data, ÆNDRE DENNE TIL MPU6050 GETMOTION6
		getAcclDataFromSerialM6(&ax, &ay, &az, &gx, &gy, &gz);
		// Beregn produktet af det samplede værdier
		acclRollingData[dataIndex] = calculatePythagoras(ax, ay, az);
		gyroRollingData[dataIndex] = calculatePythagoras(ax, ay, az);
		// Tjek om det er tid til at lave databehandling
		if (dataIndex >= bufferSize - 1) {
			// Sæt data index tilbage til 0
			dataIndex = 0;
			// Kopier alt data fra rolling buffer til static buffer
			for (uint16_t i = 0; i < bufferSize; i++) {
				acclStaticData[i] = acclRollingData[i];
				gyroStaticData[i] = gyroRollingData[i];
			}
			vTaskResume(hadTaskHandler);
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
void handleActivityDataTask(void *pvParameters) {
	// Task setup
	Serial.println("Setting up data processing task");
	// Sæt task på pause indtil den skal bruges
	vTaskSuspend(NULL);
	// Task loop
	for (;;) {
		Serial.println("Running data processing");
		// Normaliser accelerometer- og gyroskopdata


		
		Serial.println("Data processing done");
		//Suspend task indtil der er ny data klar
		vTaskSuspend(NULL);
	}
}




