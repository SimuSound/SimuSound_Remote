#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run 'make menuconfig' to enable it
#endif

BluetoothSerial SerialBT;

const int Motor = 12; // These are GPIO #s
const int MotorPWM = 13;
const int Button = 15;
const int Switch = 23;

const int Freq = 20;
const int Resolution = 8;

String message = ""; // Theses are for reading bluetooth messages from incoming device
String cmd = ""; // complete command
char inputChar = 0;

int ButtonState;
int SwitchState;

void setup() {
  // put your setup code here, to run once:
  pinMode(Motor, OUTPUT);
  pinMode(Button, INPUT);
  pinMode(Switch, INPUT);
  ledcSetup(0,Freq,Resolution);
  ledcAttachPin(MotorPWM,0);
  Serial.begin(115200);
  SerialBT.begin("ESP32Remote");
  Serial.println("Device started, pair with bluetooth.");
}

void loop() {
  // put your main code here, to run repeatedly:
  ButtonState = digitalRead(Button);
  SwitchState = digitalRead(Switch);

  //read incoming bluetooth serial message
  if (SerialBT.available()) {
    while (SerialBT.available()) {
      inputChar = SerialBT.read();
      if (inputChar == '\n') {
        break;  // handle one command at a time
      }
      Serial.print(inputChar);
      Serial.print(" ");
      Serial.print(inputChar, HEX);
      Serial.println();
      if (inputChar >= 0x20 && inputChar <= 0x7f) {  // in printable character range
        message += String(inputChar);
      }
    }

    if (inputChar == '\n') {
      cmd = message;
      message = "";
      Serial.println("-");
      Serial.println(cmd + ".");
      Serial.println(cmd.length());
      Serial.println("_");
    }
  }

  if (SwitchState == HIGH || cmd.endsWith("0vibrate"))
  {
    Serial.println("Button = 0vibrate");
    digitalWrite(Motor, HIGH);
    delay(1000);
  }
  else if (cmd.endsWith("1vibrate"))
  {
    ledcWrite(0,160);
    Serial.println("Button = 1vibrate");
    delay(1000);
  }
  else if (cmd.endsWith("2vibrate"))
  {
    ledcWrite(0,180);
    Serial.println("Button = 2vibrate");
    delay(1000);
  }
  else if (ButtonState == HIGH || cmd.endsWith("3vibrate"))
  {
    ledcWrite(0,200);
    Serial.println("Button = 3vibrate");
    delay(1000);
  }
  else
  {
    ledcWrite(0,0);
    digitalWrite(Motor, LOW);
    delay(50);
  }

  cmd = "";
  
}
