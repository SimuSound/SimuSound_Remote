/*
    Video: https://www.youtube.com/watch?v=oCMOYS71NIU
    Based on Neil Kolban example for IDF: https://github.com/nkolban/esp32-snippets/blob/master/cpp_utils/tests/BLE%20Tests/SampleNotify.cpp
    Ported to Arduino ESP32 by Evandro Copercini

   Create a BLE server that, once we receive a connection, will send periodic notifications.
   The service advertises itself as: 6E400001-B5A3-F393-E0A9-E50E24DCCA9E
   Has a characteristic of: 6E400002-B5A3-F393-E0A9-E50E24DCCA9E - used for receiving data with "WRITE" 
   Has a characteristic of: 6E400003-B5A3-F393-E0A9-E50E24DCCA9E - used to send data with  "NOTIFY"

   The design of creating the BLE server is:
   1. Create a BLE Server
   2. Create a BLE Service
   3. Create a BLE Characteristic on the Service
   4. Create a BLE Descriptor on the characteristic
   5. Start the service.
   6. Start advertising.

   In this example rxValue is the data received (only accessible inside that function).
   And txValue is the data to be sent, in this example just a byte incremented every second. 
*/
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

BLEServer *pServer = NULL;
BLECharacteristic * pTxCharacteristic;
bool deviceConnected = false;
bool oldDeviceConnected = false;
uint8_t txValue = 0;

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" // UART service UUID
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"


const int Motor = 12; // These are GPIO #s
const int MotorPWM = 13;
const int Button1 = 15;
const int Button2 = 23;
const int Button3 = 14;
const int Button4 = 27;
const int Button5 = 26;

const int Freq = 20;
const int Resolution = 8;

#define MESSAGE_MAX_LEN 10
char message[MESSAGE_MAX_LEN] = {0};
uint8_t message_len = 0;
bool isNewMessage = false;
SemaphoreHandle_t xMutex;

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};


void handle_message() {
  char local_message[MESSAGE_MAX_LEN] = {0};
  uint8_t local_message_len;
  bool local_isNewMessage;
  char message_val;  // only one char sent by rpi for now


  xSemaphoreTake( xMutex, portMAX_DELAY );
  local_isNewMessage = isNewMessage;
  local_message_len = message_len;
  if (local_isNewMessage) {
    memcpy(local_message, message, local_message_len);
  }
  isNewMessage = false;
  xSemaphoreGive( xMutex );

  if (!local_isNewMessage) {
    return;
  }

  message_val = local_message[local_message_len-1];
  Serial.print("message_val: ");
  Serial.println((uint8_t)message_val);

  // cancel out button press when we are about to vibrate
  txValue = 0;
  pTxCharacteristic->setValue(&txValue, 1);
  // pTxCharacteristic->notify();
  // delay(10); // bluetooth stack will go into congestion, if too many packets are sent


  if (message_val == '0'){
//     digitalWrite(Motor, HIGH);
//     delay(1000);
//     digitalWrite(Motor, LOW);
  } else if (message_val == '1'){
    ledcWrite(0,160);
    delay(1000);
    ledcWrite(0,0);
  } else if (message_val == '2'){
    ledcWrite(0,180);
    delay(1000);
    ledcWrite(0,0);
  } else if (message_val == '3'){
    ledcWrite(0,200);
    delay(1000);
    ledcWrite(0,0);
  } else if (message_val == '4'){
    ledcWrite(0,220);
    delay(1000);
    ledcWrite(0,0);
  } else if (message_val == '5'){
    ledcWrite(0,240);
    delay(1000);
    ledcWrite(0,0);
  }
}


// setup and loop is on core 1, bluetooth etc. is on core 0
class MyCallbacks: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    std::string rxValue = pCharacteristic->getValue();
    // looks like rxValue.length() may be greater, but always ends with null terminators
    
    if (rxValue.length() > 0) {
      Serial.println("*********");
      Serial.print("Received Value: ");
      for (int i = 0; i < rxValue.length(); i++){
        Serial.print("'");
        Serial.print(rxValue[i]);
        Serial.print("'(");
        Serial.print((int)rxValue[i]);
        Serial.print(")");
        Serial.print(", ");
      }
      Serial.println();
      Serial.print("Length: ");
      Serial.println(rxValue.length());
      Serial.println("*********");

      xSemaphoreTake( xMutex, portMAX_DELAY );
      memset(message, 0, MESSAGE_MAX_LEN);
      message_len = strnlen(rxValue.c_str(), MESSAGE_MAX_LEN-1);
      memcpy(message, rxValue.c_str(), message_len);
      isNewMessage = true;
      xSemaphoreGive( xMutex );

      Serial.print("cstring Length: ");
      Serial.println(message_len);
      Serial.print("message[message_len-1]: ");
      Serial.println(message[message_len-1]);
    }
  }
};

void setup() {
  xMutex = xSemaphoreCreateMutex();

  pinMode(Motor, OUTPUT);
  // these are active low now
  pinMode(Button1, INPUT_PULLUP);
  pinMode(Button2, INPUT_PULLUP);
  pinMode(Button3, INPUT_PULLUP);
  pinMode(Button4, INPUT_PULLUP);
  pinMode(Button5, INPUT_PULLUP);
  
  ledcSetup(0,Freq,Resolution);
  ledcAttachPin(MotorPWM,0);
  
  Serial.begin(115200);

  // Create the BLE Device
  BLEDevice::init("ESP32");
  
  // Create the BLE Server
  // https://www.arduino.cc/en/Reference/ArduinoBLE
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pTxCharacteristic = pService->createCharacteristic(
										CHARACTERISTIC_UUID_TX,
                    BLECharacteristic::PROPERTY_READ |
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

}

void loop() {

  int Button1State = digitalRead(Button1);
  int Button2State = digitalRead(Button2);
  int Button3State = digitalRead(Button3);
  int Button4State = digitalRead(Button4);
  int Button5State = digitalRead(Button5);
  
  if (deviceConnected) {
      if(Button1State == LOW){
        txValue = 1; // can use txValue as button number
      } else if (Button2State == LOW) {
        txValue = 2; // can use txValue as button number
      } else if (Button3State == LOW) {
        txValue = 3; // can use txValue as button number
      } else if (Button4State == LOW) {
        txValue = 4; // can use txValue as button number
      } else if (Button5State == LOW) {
        txValue = 5; // can use txValue as button number
      } else {
          txValue = 0;  
      }

      if (txValue != 0) {
        Serial.print("Button being pressed: ");
        Serial.println(txValue);
      }

      pTxCharacteristic->setValue(&txValue, 1);
      // pTxCharacteristic->notify();
      delay(10); // bluetooth stack will go into congestion, if too many packets are sent
      txValue = 0;
  }

  handle_message();

  // disconnecting
  if (!deviceConnected && oldDeviceConnected) {
      delay(500); // give the bluetooth stack the chance to get things ready
      pServer->startAdvertising(); // restart advertising
      Serial.println("disconnected, start advertising");
      oldDeviceConnected = deviceConnected;
  }
  // connecting
  if (deviceConnected && !oldDeviceConnected) {
  // do stuff here on connecting 
      Serial.println("connecting");
      oldDeviceConnected = deviceConnected;
  }
}
