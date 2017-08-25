#include <LiquidCrystal.h>
#include <Canbus.h>
#include <mcp2515.h>
#include <mcp2515_defs.h>
#include <defaults.h>
#include <global.h>

#define UP     A1
#define DOWN   A3
#define LEFT   A2
#define RIGHT  A5
#define CLICK  A4

#define PIN_RS 8
#define PIN_EN 7
#define PIN_D4 6
#define PIN_D5 5
#define PIN_D6 4
#define PIN_D7 3

#define NUM_PAGES 5
#define DEBOUNCE_TIME 1

int pageNo = 0;
int prevInTime = 0;

unsigned int inputRPM = 0;
unsigned int inputTEMP = 0;
unsigned int inputGEAR = 0;
float inputSPD = 0;
float inputSWA = 0;

bool canNotReady = true;

LiquidCrystal lcd(PIN_RS, PIN_EN, PIN_D4, PIN_D5, PIN_D6, PIN_D7);

void readCAN() {
  tCAN message;
  while (mcp2515_check_message()) { //read until buffer empty
    if (mcp2515_get_message(&message)) {
      if (canNotReady) {
        lcd.clear();
      }

      canNotReady = false;  //CAN Message received, switch to active display
      decodeCAN(message);
    }
  }
}

void decodeCAN(tCAN& msg) {
  switch (msg.id) {
    case 0x17c:
      //ENGINE_RPM = {16,1.000000,16,false,false,0.000000} rpm
      inputRPM = (unsigned int)msg.data[3] + ((unsigned int)msg.data[2]) * 256;
      break;
    case 0x1d0:
      //SPEED_FL = {0,0.005,15,false,false,0.000000} kph
      inputSPD = (unsigned int)((msg.data[1] & 0xFE) >> 1) + ((unsigned int)msg.data[0]) * 256;
      inputSPD *= 0.005; //scale
      break;
    case 0x191:
      //GEAR = {36,1.000000,4,false,false,0.000000} gear
      inputGEAR = (unsigned int)((msg.data[0] & 0xF0) >> 4);
      break;
    case 0x324:
      //ENGINE_TEMPERATURE = {0,1.000000,8,false,false,0.000000} degC
      inputTEMP = (unsigned int)msg.data[0];
      break;
    case 0x156:
      //STEER_ANGLE = {0,-0.100000,16,true,false,0.000000} deg
      inputSWA = (((unsigned int)msg.data[0]) * 256 + (unsigned int)msg.data[1]) * 0.1;
      if ((msg.data[0] & 0x80) >> 7) { //sign bit
        inputSWA -= 6553.5;  //2s compliment equivalent
      }
      break;
    default:
      break;
  }
}

void displayRPM(int line) {
  lcd.setCursor(0, line);
  lcd.print("RPM: ");
  lcd.setCursor(5, line);
  lcd.print(inputRPM);
  if (inputRPM < 1000) {
    lcd.print("        "); //fill line with blank
  }
  //  Serial.print("RPM: ");
  //  Serial.println(inputRPM);
}

void displayTEMP(int line) {
  lcd.setCursor(0, line);
  lcd.print("Temp: ");
  lcd.setCursor(6, line);
  lcd.print(inputTEMP);
  lcd.setCursor(9, line);
  lcd.print("C");
  //  Serial.print("Temp: ");
  //  Serial.print(inputTEMP);
  //  Serial.println(" C");
}

void displayGEAR(int line) {
  lcd.setCursor(0, line);
  lcd.print("Gear: ");
  lcd.setCursor(6, line);
  lcd.print(inputGEAR);
  //  Serial.print("Gear: ");
  //  Serial.println(inputGEAR);
}

void displaySPD(int line) {
  lcd.setCursor(0, line);
  lcd.print("Spd: ");
  lcd.setCursor(5, line);
  lcd.print(inputSPD);
  lcd.setCursor(10, line);
  lcd.print("kph  ");
  //  Serial.print("Spd: ");
  //  Serial.print(inputSPD);
  //  Serial.println(" kph");
}

void displaySWA(int line) {
  lcd.setCursor(0, line);
  lcd.print("SWA: ");
  lcd.setCursor(5, line);
  lcd.print(inputSWA);
  lcd.setCursor(10, line);
  lcd.print("deg  ");
  //  Serial.print("SWA: ");
  //  Serial.print(inputSWA);
  //  Serial.println(" deg");
}

void wrapPage() {
  if (pageNo < 0) {
    pageNo = NUM_PAGES - 1;
  } else if (pageNo > NUM_PAGES - 1) {
    pageNo = 0;
  }
  Serial.print("Page ");
  Serial.println(pageNo);
}

void incrementPage() {
  prevInTime = millis() / 1000;
  lcd.clear();
  pageNo++;
  wrapPage();
}

void decrementPage() {
  prevInTime = millis() / 1000;
  lcd.clear();
  pageNo--;
  wrapPage();
}

void processJoystick() {
  if (millis() / 1000 - prevInTime < DEBOUNCE_TIME) {return;}

  if (digitalRead(UP) == 0) {incrementPage();}
  if (digitalRead(DOWN) == 0) {decrementPage();}
  if (digitalRead(LEFT) == 0) {decrementPage();}
  if (digitalRead(RIGHT) == 0) {incrementPage();}
}


void setup() {
  
  pinMode(UP, INPUT);
  pinMode(DOWN, INPUT);
  pinMode(LEFT, INPUT);
  pinMode(RIGHT, INPUT);
  pinMode(CLICK, INPUT);

  //Pull analog pins high to enable reading of joystick movements
  digitalWrite(UP, HIGH);
  digitalWrite(DOWN, HIGH);
  digitalWrite(LEFT, HIGH);
  digitalWrite(RIGHT, HIGH);

  //Setup 16col x 2row
  lcd.begin(16, 2);

  // Print startup message
  lcd.setCursor(0, 0);
  lcd.print("Digital Cluster");
  lcd.setCursor(0, 1);
  lcd.print("    v0.1.2     ");

  delay(2000);    //Wait 2000ms
  
  //Enable Serial
  Serial.begin(9600);
  Serial.println("Serial output for debugging");

  if (Canbus.init(CANSPEED_500)) {
    Serial.println("CAN init OK");
  }
  else {
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("CAN init failed");
    lcd.setCursor(0,1);
    lcd.println("Reset to restart");
    
    Serial.println("CAN init failed");
    delay(100000);
  }

  lcd.clear();
}

void loop() {
  processJoystick();

  readCAN();

  if (!canNotReady) {
    switch (pageNo % NUM_PAGES) {
      case 0:
        displayRPM(0);
        displayTEMP(1);
        break;
      case 1:
        displayTEMP(0);
        displayGEAR(1);
        break;
      case 2:
        displayGEAR(0);
        displaySPD(1);
        break;
      case 3:
        displaySPD(0);
        displaySWA(1);
        break;
      case 4:
        displaySWA(0);
        displayRPM(1);
        break;
      default:
        lcd.setCursor(0, 1);
        lcd.print(millis() / 1000);
        break;
    }
  }
  else {
    lcd.setCursor(0, 0);
    lcd.print("Waiting for CAN");
  }

  delay(5); // [TODO] Verify if msg buffering is still causing issue
}
