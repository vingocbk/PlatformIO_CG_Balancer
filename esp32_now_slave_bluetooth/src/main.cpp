/*
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp-now-esp32-arduino-ide/
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.
  
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*/
#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
// #include "BluetoothSerial.h"

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#include "HX711.h"

HX711 scale;

static QueueHandle_t msg_queue;

BLEServer *pServer = NULL;
BLECharacteristic * pTxCharacteristic;
bool deviceConnected = false;
bool oldDeviceConnected = false;
float fDataLoadCell[3] = {23.5345, 45.1, 32.2};

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" // UART service UUID
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"


// HX711 circuit wiring
const int LOADCELL_DOUT_PIN = 2;
const int LOADCELL_SCK_PIN = 3;

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      Serial.println("onConnect");
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      Serial.println("onDisconnect");
      deviceConnected = false;
    }
};

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string rxValue = pCharacteristic->getValue();

      if (rxValue.length() > 0) {
        Serial.println("*********");
        Serial.print("Received Value: ");
        for (int i = 0; i < rxValue.length(); i++)
          Serial.print(rxValue[i]);

        Serial.println();
        Serial.println("*********");
      }
    }
};



// Structure example to receive data
// Must match the sender structure
typedef struct struct_message {
  int number;
  float valueLoadCell;
} struct_message;

// Create a struct_message called myData
struct_message myData;

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  Serial.print("number: ");
  Serial.print(myData.number);
  Serial.print(" --- ");
  Serial.print("valueLoadCell: ");
  Serial.println(myData.valueLoadCell);
  xQueueSend(msg_queue, &myData, 0);
}



void setup() {
  // Init Serial Monitor
  Serial.begin(115200);


  // Create the BLE Device
  BLEDevice::init("Load Cell");

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pTxCharacteristic = pService->createCharacteristic(
										CHARACTERISTIC_UUID_TX,
										BLECharacteristic::PROPERTY_NOTIFY
									);
                      
  pTxCharacteristic->addDescriptor(new BLE2902());

  BLECharacteristic * pRxCharacteristic = pService->createCharacteristic(
											 CHARACTERISTIC_UUID_RX,
											BLECharacteristic::PROPERTY_WRITE
										);

  pRxCharacteristic->setCallbacks(new MyCallbacks());

  // Start the service
  pService->start();

  // Start advertising
  pServer->getAdvertising()->start();
  Serial.println("Waiting a client connection to notify...");



  msg_queue = xQueueCreate(10, sizeof(myData));
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  Serial.println(WiFi.macAddress());

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(OnDataRecv);


  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);

  pinMode(2, OUTPUT);
}
 
uint32_t count_time = 0;
void loop() {
  struct_message myDataQueue;
  if (deviceConnected && millis() >= count_time + 500) {
    count_time = millis();
    uint8_t dataSend[12] = {0};
    uint8_t array[4] = {0};
    memcpy(array,&fDataLoadCell[0],4);
    for(int i = 0; i < 4; i++){
      dataSend[i] = array[i];
    }
    memcpy(array,&fDataLoadCell[1],4);
    for(int i = 0; i < 4; i++){
      dataSend[4+i] = array[i];
    }
    memcpy(array,&fDataLoadCell[2],4);
    for(int i = 0; i < 4; i++){
      dataSend[8+i] = array[i];
    }

    pTxCharacteristic->setValue(dataSend, 12);
    pTxCharacteristic->notify();
	}

  if (xQueueReceive(msg_queue, (void *)&myDataQueue, 0) == pdTRUE) {
    if(myDataQueue.number == 1){
      fDataLoadCell[1] = myDataQueue.valueLoadCell;
    }
    else if(myDataQueue.number == 2){
      fDataLoadCell[2] = myDataQueue.valueLoadCell;
    }
	}

  // disconnecting
  if (!deviceConnected && oldDeviceConnected) {
      delay(500); // give the bluetooth stack the chance to get things ready
      pServer->startAdvertising(); // restart advertising
      Serial.println("start advertising");
      oldDeviceConnected = deviceConnected;
  }
  // connecting
  if (deviceConnected && !oldDeviceConnected) {
  // do stuff here on connecting
      oldDeviceConnected = deviceConnected;
  }
  digitalWrite(2, HIGH);
  delay(500);
  digitalWrite(2, LOW);
  delay(500);
}