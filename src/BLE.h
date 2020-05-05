// Biblioketer inklisioner til BLE 
#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
//#include <BLEServer.h>
//----------------------------------------------
// BLE variabler
// Den service vi gerne vil have forbindelse til, fra den trådløse server.
BLEUUID serviceUUID("4fafc201-1fb5-459e-8fcc-c5c9c331914b");
// Karateristiken af serveren vi er intereseret i. I dette tilfælde er det modtager og sender UUID.
BLEUUID r_RX_charUUID("beb5483e-36e1-4688-b7f5-ea07361b26a8");
BLEUUID r_TX_charUUID("12ee6f51-021d-438f-8094-bf5c5b36eab9");

static boolean doConnect = false;
static boolean connected = false;
static boolean doScan = false;
static BLERemoteCharacteristic* pRemoteCharacteristic_RX;
static BLERemoteCharacteristic* pRemoteCharacteristic_TX;
static BLEAddress masterAddress = BLEAddress((uint8_t*)"\67\145\43\35\76\57");

void ( *startSampleFuncPointer) ();
void ( *stopSampleFuncPointer) ();
//----------------------------------------------
// Funktinoer
static void notifyCallback(
  BLERemoteCharacteristic* pBLERemoteCharacteristic,
  uint8_t* pData,
  size_t length,
  bool isNotify) {
    Serial.println("Callback triggered");
    Serial.print("Notify callback for characteristic ");
    Serial.print(pBLERemoteCharacteristic->getUUID().toString().c_str());
    Serial.print(" of data length ");
    Serial.println(length);
    Serial.print("data: ");
    Serial.println((char*)pData);
	String incommingBLE = "";
	for (uint8_t i = 0; i < length; i++) {
		incommingBLE = incommingBLE + *((char*)pData + i); 
	}
	if (incommingBLE.indexOf ("startSampling") >= 0) {
		startSampleFuncPointer();
	}else if (incommingBLE.indexOf ("stopSampling") >= 0){
		stopSampleFuncPointer ();
	}
}

class MyClientCallback : public BLEClientCallbacks {
	void onConnect(BLEClient* pclient) {
		Serial.println("onConnect");
	}

	void onDisconnect(BLEClient* pclient) {
		connected = false;
		Serial.println("onDisconnect");
	}
};

// laver en connectiong til server
bool connectToServer() {
    Serial.print("Forming a connection to ");
    Serial.println(masterAddress.toString().c_str());
    
    BLEClient*  pClient  = BLEDevice::createClient();
    Serial.println(" - Created client");

    pClient->setClientCallbacks(new MyClientCallback());

    // former en trådløs BLE forbindelse til server.
    if (!pClient->connect(masterAddress, BLE_ADDR_TYPE_RANDOM)) {;  // if you pass BLEAdvertisedDevice instead of address, it will be recognized type of peer device address (public or private)
		return false;
	}
    Serial.println(" - Connected to server");

    // Får en reference til den service vi er efter til den trådløse BLE server.
    BLERemoteService* pRemoteService = pClient->getService(serviceUUID);
    if (pRemoteService == nullptr) { // Dette if statment benyttes hvis den ikke får forbindelse med servicen
		Serial.print("Failed to find our service UUID: ");
		Serial.println(serviceUUID.toString().c_str());
		pClient->disconnect();
		return false;
    }
    Serial.println(" - Found our service");

	// Får en reference til modtager og sender service vi er efter i den trådløse BLE server.
    pRemoteCharacteristic_RX = pRemoteService->getCharacteristic(r_RX_charUUID);
    if (pRemoteCharacteristic_RX == nullptr) {
		Serial.print("Failed to find our characteristic UUID: ");
		Serial.println(r_TX_charUUID.toString().c_str());
		pClient->disconnect();
		return false;
    }

    Serial.println(" - Found our r_RX_characteristic");
    // samme som ovenover..
    pRemoteCharacteristic_TX = pRemoteService->getCharacteristic(r_TX_charUUID);
    if (pRemoteCharacteristic_TX == nullptr) {
		Serial.print("Failed to find our characteristic UUID: ");
		Serial.println(r_TX_charUUID.toString().c_str());
		pClient->disconnect();
		return false;
    }
    Serial.println(" - Found our r_TX_characteristic");

 
    if(pRemoteCharacteristic_TX->canNotify()) {
		Serial.println("Characteristic has notify property");
		pRemoteCharacteristic_TX->registerForNotify(notifyCallback);
    }
    
    connected = true;
    return true;
}

void writeToServer (String message){
	if (connected) {
		pRemoteCharacteristic_TX->writeValue(message.c_str(), message.length());
	} else {
		if (connectToServer()) {
			Serial.println("We are now connected to the BLE Server.");
		}else {
			Serial.println("We have failed to connect to the server; there is nothin more we will do.");
		}
	}
}