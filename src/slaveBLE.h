// Sikre at biblioteket kun inkluderes en gang
#ifndef _slaveBLE_
#define _slaveBLE_

// Biblioketer inklisioner til BLE 
#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
//#include <BLEServer.h>

//----------------------------------------------
// BLE variabler
// Den service vi gerne vil have forbindelse til, fra den trådløse server.
static BLEUUID serviceUUID("4fafc201-1fb5-459e-8fcc-c5c9c331914b");
// Karateristiken af serveren vi er intereseret i. I dette tilfælde er det modtager og sender UUID.
static BLEUUID serverDataOut_charUUID("beb5483e-36e1-4688-b7f5-ea07361b26a8"); // Client reviece, server transmit
static BLEUUID serverDataIn_charUUID("12ee6f51-021d-438f-8094-bf5c5b36eab9"); // Client transmit, server reviece

uint8_t MasterMacAddr[] = {0x24, 0x0A, 0xC4, 0x32, 0x1B, 0x22}; //{0x22, 0x1B, 0x32, 0xC4, 0x0A, 0x24}; // {0x24, 0x0A, 0xC4, 0x32, 0x1B, 0x22}
//String MasterMACstring = "24:0a:c4:32:1b:22";

static boolean connected = false;
static BLERemoteCharacteristic* pRemoteCharacteristic_ServerDataOut;
static BLERemoteCharacteristic* pRemoteCharacteristic_ServerDataIn;
//static BLEAddress masterAddress = MasterMACstring.c_str();
static BLEAddress masterAddress = MasterMacAddr;
//static BLEAddress masterAddress = BLEAddress(MasterMacAddr);
	//(uint8_t*)"\36\10\196\50\27\34");
// {0x24, 0x0A, 0xC4, 0x32, 0x1B, 0x22});

// Funktions pointers
void ( *startSampleFuncPointer) ();
void ( *stopSampleFuncPointer) ();

//----------------------------------------------
// Funktinoer
// Connect callback
class MyClientCallback : public BLEClientCallbacks {
	void onConnect(BLEClient* pclient) {
		Serial.println(" - onConnect");
	}

	void onDisconnect(BLEClient* pclient) {
		connected = false;
		stopSampleFuncPointer(); // Stop sampling ved disconnect
		Serial.println("onDisconnect");
	}
};

// Callback funktion, når det modtages beskeder fra characteristics
static void notifyCallback(
  BLERemoteCharacteristic* pBLERemoteCharacteristic,
  uint8_t* pData,
  size_t length,
  bool isNotify) {
    Serial.println("notifyCallback(), Callback triggered");
    Serial.print("notifyCallback(), Characteristic ");
    Serial.print(pBLERemoteCharacteristic->getUUID().toString().c_str());
    Serial.print(" , data length ");
    Serial.println(length);
    Serial.print("notifyCallback(), data: ");
	String incommingBLE = "";
	for (uint8_t i = 0; i < length; i++) {
		incommingBLE = incommingBLE + *((char*)pData + i); 
	}
	if (incommingBLE.indexOf ("start") >= 0) { // Finder index til "start" i besked, -1 hvis ikke fundet
		startSampleFuncPointer();
	}else if (incommingBLE.indexOf ("stop") >= 0){
		stopSampleFuncPointer ();
	}
	Serial.println(incommingBLE);
}

// laver en connectiong til server
bool connectToServer() {
    Serial.print("Forming a connection to ");
    Serial.println(masterAddress.toString().c_str());
    BLEClient*  pClient  = BLEDevice::createClient();
    Serial.println(" - Created client");
    pClient->setClientCallbacks(new MyClientCallback());
    // former en trådløs BLE forbindelse til server.
	bool didConnect = pClient->connect(masterAddress);
    if (!didConnect) {;  // if you pass BLEAdvertisedDevice instead of address, it will be recognized type of peer device address (public or private)
		Serial.println("Couldn't connect to server!");
		return false;
	}
    Serial.println(" - Connected to server");
    // Får en reference til den service vi er efter til den trådløse BLE server.
    BLERemoteService* pRemoteService = pClient->getService(serviceUUID);
    if (pRemoteService == nullptr) { // Dette if statment benyttes hvis den ikke får forbindelse med servicen
		Serial.print("Failed to find service UUID: ");
		Serial.println(serviceUUID.toString().c_str());
		pClient->disconnect();
		return false;
    }
    Serial.println(" - Found service");
	// Find serverDataOut characteristic.
    pRemoteCharacteristic_ServerDataOut = pRemoteService->getCharacteristic(serverDataOut_charUUID);
    if (pRemoteCharacteristic_ServerDataOut == nullptr) {
		Serial.print("Failed to find serverDataOut characteristic UUID: ");
		Serial.println(serverDataOut_charUUID.toString().c_str());
		pClient->disconnect();
		return false;
    }
	Serial.println(" - Found serverDataOut characteristic");
    if(pRemoteCharacteristic_ServerDataOut->canNotify()) {
		Serial.println("Characteristic has notify property");
		pRemoteCharacteristic_ServerDataOut->registerForNotify(notifyCallback);
    }
	// Find serverDataIn characteristic.
    pRemoteCharacteristic_ServerDataIn = pRemoteService->getCharacteristic(serverDataIn_charUUID);
    if (pRemoteCharacteristic_ServerDataIn == nullptr) {
		Serial.print("Failed to find serverDataIn characteristic UUID: ");
		Serial.println(serverDataIn_charUUID.toString().c_str());
		pClient->disconnect();
		return false;
    }
    Serial.println(" - Found serverDataIn characteristic");
    connected = true;
    return true;
}

void writeToServer (String message){
	if (connected) {
		pRemoteCharacteristic_ServerDataIn->writeValue(message.c_str(), message.length());
	} else {
		if (connectToServer()) {
			Serial.println("We are now connected to the BLE Server.");
		}else {
			Serial.println("We have failed to connect to the server; there is nothin more we will do.");
		}
	}
}
#endif