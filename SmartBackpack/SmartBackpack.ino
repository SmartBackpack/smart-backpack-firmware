/*
   Smart backpack project's firmware for Arduino based sensors management


   Sensors used:

   Created at 4th of March 2017
   by Vilius Kraujutis, Justas Riabovas, Povilas Brazys, Mindaugas Varkalys, Gražina Bočkutė et al.

   # Library dependencies

   # HC-05 BLUETOOTH

    If you want to use AT COMMANDS to configure HC-05 bluetooth module,
    you need to change BLUETOOTH_USE_AT_COMMANDS constant to true.

    You need to bring HC-05 into AT commands mode like this:
      - power off arduino
      - press and hold little button on HC-05 module
      - power on arduino (so it will power on BT module too)
      - wait until red LED on BT module will start flashing every 2s

    Serial should be in 9600 baud rate (although BTSerial will operate in 38400 baud rate).

      Most useful AT commands are
        AT : Ceck the connection.
        AT+NAME : See default name
        AT+ADDR : see default address
        AT+VERSION : See version
        AT+UART : See baudrate
        AT+ROLE: See role of bt module(1=master/0=slave)
        AT+RESET : Reset and exit AT mode
        AT+ORGL : Restore factory settings
        AT+PSWD: see default password

   # CONNECTION Scheme

   See PINS section below to see what actual constants mean

   ## BLUETOOTH
      - Connect the HC-05 TX to Arduino pin PIN_BLUETOOTH_RX.
      - Connect the HC-05 RX to Arduino pin PIN_BLUETOOTH_TX through a voltage divider.
*/



// PINS

#define BLUETOOTH_USE_AT_COMMANDS false
#define PIN_BLUETOOTH_RX 2
#define PIN_BLUETOOTH_TX 3
#define PIN_HEART_BEAT_LED 13
#define PIN_BUTTON    9

#define TILT1 4
#define TILT2 5
#define REED1 6
#define REED2 7
#define BUZZ 8
#define LED1 9
#define LED2 10
#define LIGHT A0

// INCLUDES

#include <SoftwareSerial.h>

// CONSTANTS

#define LED_BLINK_TIME 100

// VARIABLE INITIALIZATION

SoftwareSerial BTserial(PIN_BLUETOOTH_RX, PIN_BLUETOOTH_TX); // RX | TX // would be good to move to hardware RX/TX pins for better performance

// PRIMITIVE VARIABLES

boolean blinkLED = true;
boolean isButtonPressed = true;

//#define TILTSTAT ((Tilt1Stat == TILTFAIL) || (Tilt2Stat == TILTFAIL))
#define TILT_ALARM_CNT  5 //Counter of invalid values to start alarm
bool Tilt1Stat, Tilt2Stat, Reed1Stat, Reed2Stat;
int TiltCnt = 0;

//Serial
String SerRx, BtRx;

//Light sensor
volatile int LSens = 0;
bool LStat;
#define DARK_THRESH 50 //light sensor
#define BRIGHT 1
#define DARK  0

// SETUP

void setup() {
  Serial.begin(9600);
  Serial.println();
  Serial.println("Arduino is ready");
  Serial.println("Remember to select Both NL & CR in the serial monitor");

  // BLUETOOTH - HC-05 default serial speed for AT mode is 38400
  //BTserial.begin(38400); // for AT commands
  BTserial.begin(9600); // for AT commands

  // PIN MODES
  pinMode(TILT1, INPUT_PULLUP);
  pinMode(TILT2, INPUT_PULLUP);
  pinMode(REED1, INPUT_PULLUP);
  pinMode(REED2, INPUT_PULLUP);
  pinMode(PIN_HEART_BEAT_LED, OUTPUT);
  pinMode(PIN_BUTTON, INPUT_PULLUP);

  pinMode(BUZZ, OUTPUT);
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  digitalWrite(BUZZ, LOW);
  digitalWrite(LED1, LOW);
  digitalWrite(LED2, LOW);

  pinMode(LIGHT, INPUT);
}

// PROGRAM LOOP

void loop() {
  if (BLUETOOTH_USE_AT_COMMANDS) {
    readFromBTserial();
    writeToBTserial();
    return;
  }

  //readButtonValue();
  heartBeatLED(true);

  //******************************************
  //LIGHT sensor
  //******************************************
  LSens = analogRead(LIGHT);
  if (LSens > 50)LStat = DARK;
  else LStat = BRIGHT;

  //******************************************
  //Tilt sensors
  //******************************************
  Tilt1Stat = digitalRead(TILT1);
  Tilt2Stat = digitalRead(TILT2);
  Reed1Stat = digitalRead(REED1);
  Reed2Stat = digitalRead(REED2);

  //******************************************
  // Control LEDS an BUZZ
  //******************************************

  if (Tilt1Stat ^ Tilt2Stat) { //sensors status not equal
    If (TiltCnt >= TILT_ALARM_CNT) {
      BuzzBlink(100);
      LEDSOn();
    } else TiltCnt++;
  } else { //Sensor status equal
    TiltCnt = 0; //reset
    if (TiltStat1 == TILTFAIL) {
    }
  }

  if (TILTSTAT == TILTFAIL) {
    LEDSOn();
  } else {
    if (LStat == DARK) LEDSBlink(100);
    else LEDSOff();
  }

  if ((TILTSTAT == TILTFAIL) && (Tilt1Stat != Tilt2Stat)) {
    BuzzBlink(100);
  } else {
    BuzzOff();
  }

  //******************************************
  //Terminal
  //******************************************

  //Print status to BT
  BTserial.print("STAT:");
  BTserial.print(Tilt1Stat, DEC); BTserial.print(";");
  BTserial.print(Tilt2Stat, DEC); BTserial.print(";");
  BTserial.print(TILTSTAT, DEC); BTserial.print(";");
  BTserial.print(LSens, DEC); BTserial.println();


  //Print status to terminal
  Serial.print("STAT:");
  Serial.print(Tilt1Stat, DEC); Serial.print(";");
  Serial.print(Tilt2Stat, DEC); Serial.print(";");
  Serial.print(TILTSTAT, DEC); Serial.print(";");
  Serial.print(LSens, DEC); Serial.println();

  //Received data
  SerRx.remove(SerRx.indexOf("\n"));
  SerRx.remove(SerRx.indexOf("\r"));
  if (SerRx == "") {
    SerRx = BTserial.readString();
    SerRx.remove(SerRx.indexOf("\n"));
    SerRx.remove(SerRx.indexOf("\r"));
  }

  //Process data
  if (SerRx == "BUZZ:ON") digitalWrite(BUZZ, HIGH);
  if (SerRx == "BUZZ:OFF") digitalWrite(BUZZ, LOW);
  if (SerRx == "LED1:ON") digitalWrite(LED1, HIGH);
  if (SerRx == "LED1:OFF") digitalWrite(LED1, LOW);
  if (SerRx == "LED2:ON") digitalWrite(LED2, HIGH);
  if (SerRx == "LED2:OFF") digitalWrite(LED2, LOW);

  heartBeatLED(false);
  //Serial.println(LSens, DEC);
  //recvOneChar();
  //showNewData();
}

char receivedChar;
boolean newData = false;

void LEDSOn() {
  digitalWrite(LED1, HIGH);
  digitalWrite(LED2, HIGH);
}

void LEDSOff() {
  digitalWrite(LED1, LOW);
  digitalWrite(LED2, LOW);
}

void LEDSBlink(int del) {
  digitalWrite(LED1, HIGH);
  delay(del);
  digitalWrite(LED1, LOW);
  digitalWrite(LED2, HIGH);
  delay(del);
  digitalWrite(LED2, LOW);
}


void BuzzOn() {
  digitalWrite(BUZZ, HIGH);
}

void BuzzOff() {
  digitalWrite(BUZZ, LOW);
}
void BuzzBlink(int del) {
  digitalWrite(BUZZ, HIGH);
  delay(del);
  digitalWrite(BUZZ, LOW);
}

void recvOneChar() {
  if (BTserial.available() > 0) {
    receivedChar = BTserial.read();
    newData = true;
  }
}

void showNewData() {
  if (newData == true) {
    Serial.print("This just in ... ");
    Serial.println(receivedChar);
    newData = false;
  }
}


// MAIN METHODS

void readButtonValue() {
  if (digitalRead(PIN_BUTTON) == LOW) {
    isButtonPressed = true;
  } else {
    if (isButtonPressed) {
      blinkLED = !blinkLED;
    }
    isButtonPressed = false;
  }
}

void heartBeatLED(boolean light) {
  if (light && !blinkLED || isButtonPressed) {
    digitalWrite(PIN_HEART_BEAT_LED, HIGH);
  } else {
    digitalWrite(PIN_HEART_BEAT_LED, LOW);
  }
  delay(LED_BLINK_TIME);
}

String readFromBTserial() {
  String rx;
  // Keep reading from HC-05 and send to Arduino Serial Monitor
  if (BTserial.available()) {
    rx = BTserial.read();
    return rx;
  }
}

void writeToBTserial() {
  // Keep reading from Arduino Serial Monitor and send to HC-05
  if (Serial.available()) {
    BTserial.write(Serial.read());
  }
}

String toPercentage(float value, float maxValue) {
  float p = value / maxValue;
  return String(p * 100) + "%";
}


