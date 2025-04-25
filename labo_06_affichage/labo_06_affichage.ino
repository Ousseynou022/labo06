#include <LCD_I2C.h>
#include <Wire.h>
#include <AccelStepper.h>
#include <LedControl.h>

#define MOTOR_INTERFACE_TYPE 4

#define IN_1 31
#define IN_2 33
#define IN_3 35
#define IN_4 37

#define TRIGGER_PIN 9
#define ECHO_PIN 10

#define RED_PIN 3
#define BLUE_PIN 2
#define BUZZER 4

#define DIN_PIN 34
#define CLK_PIN 30
#define CS_PIN 32

LCD_I2C lcd(0x27, 16, 2);
AccelStepper myStepper(MOTOR_INTERFACE_TYPE, IN_1, IN_3, IN_2, IN_4);
LedControl lc = LedControl(DIN_PIN, CLK_PIN, CS_PIN, 1); // Un seul module MAX7219

#define STEPS_PER_REVOLUTION 200
#define MIN_ANGLE 10
#define MAX_ANGLE 170
#define QUART_CIRCLE_STEPS_OPEN (STEPS_PER_REVOLUTION * (MAX_ANGLE - MIN_ANGLE) / 360.0)

unsigned long duration;
unsigned long distance;
String doorStatus = "Ferme";
unsigned long lastDistanceMeasurement = 0;

unsigned long currentTime = 0;
unsigned long lastAlarmTime = 0;
unsigned long lastLedSwitchTime = 0;
bool ledState = false;
bool alarmActive = false;

int alarmLimit = 12;
int limInf = 10, limSup = 100;
bool isOpening = false;
bool isClosing = false;
int currentDegree = MIN_ANGLE;

bool stopCmdFlag = false;
bool resetCmdFlag = false;
bool countCmdFlag = false;
int startValue = 0;
int endValue = 99;

enum AppState { STOP, COUNTING, RESET };
AppState appState = STOP;

void setup() {
  lcd.begin();
  lcd.backlight();
  Serial.begin(9600);

  pinMode(RED_PIN, OUTPUT);
  pinMode(BLUE_PIN, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  myStepper.setMaxSpeed(1000);
  myStepper.setAcceleration(500);
  myStepper.setSpeed(200);

  lc.shutdown(0, false);
  lc.setIntensity(0, 8);
  lc.clearDisplay(0);

  lcd.setCursor(0, 0); lcd.print("2419796");
  lcd.setCursor(0, 1); lcd.print("Labo 6");
  delay(2000);
  lcd.clear();
  lcd.print("Distance:");
}

unsigned long measureDistance() {
  digitalWrite(TRIGGER_PIN, LOW); delayMicroseconds(2);
  digitalWrite(TRIGGER_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN, LOW);

  duration = pulseIn(ECHO_PIN, HIGH, 30000);
  if (duration == 0) return 999;
  return duration * 0.034 / 2;
}

void updateDisplay() {
  lcd.setCursor(0, 0);
  lcd.print("Dist : ");
  lcd.print(distance);
  lcd.print(" cm   ");
  lcd.setCursor(0, 1);
  lcd.print("Porte : ");
  lcd.print(doorStatus);
}

void showCheck() {
  byte check[8] = {
    B00000000,
    B00000001,
    B00000010,
    B10000100,
    B01001000,
    B00110000,
    B00000000,
    B00000000
  };
  lc.clearDisplay(0);
  for (int i = 0; i < 8; i++) lc.setRow(0, i, check[i]);
  delay(3000);
  lc.clearDisplay(0);
}

void showError() {
  byte error[8] = {
    B10000001,
    B01000010,
    B00100100,
    B00011000,
    B00011000,
    B00100100,
    B01000010,
    B10000001
  };
  lc.clearDisplay(0);
  for (int i = 0; i < 8; i++) lc.setRow(0, i, error[i]);
  delay(3000);
  lc.clearDisplay(0);
}

void showPannError() {
  byte interdit[8] = {
    B00111100, 
    B11000100,  
    B10001001,  
    B10010001,  
    B10100001,  
    B11000001,  
    B11000010,  
    B00111100   
  };
  lc.clearDisplay(0);
  for (int i = 0; i < 8; i++) lc.setRow(0, i, interdit[i]);
  delay(3000);
  lc.clearDisplay(0);
}


void handleAlarmEffects() {
  currentTime = millis();
  if (distance <= alarmLimit) {
    digitalWrite(BUZZER, HIGH);
    if (currentTime - lastLedSwitchTime >= 500) {
      lastLedSwitchTime = currentTime;
      ledState = !ledState;
      digitalWrite(RED_PIN, ledState);
      digitalWrite(BLUE_PIN, !ledState);
    }
    alarmActive = true;
    lastAlarmTime = currentTime;
  } else {
    if (alarmActive && currentTime - lastAlarmTime >= 3000) {
      digitalWrite(BUZZER, LOW);
      digitalWrite(RED_PIN, LOW);
      digitalWrite(BLUE_PIN, LOW);
      alarmActive = false;
    }
  }
}

void openDoorNonBlocking() {
  if (!isOpening && doorStatus == "Ferme") {
    isOpening = true;
    currentDegree = MIN_ANGLE;
    doorStatus = "Ouverture";
  }
  if (isOpening) {
    if (currentDegree <= MAX_ANGLE) {
      int steps = map(currentDegree, MIN_ANGLE, MAX_ANGLE, 0, QUART_CIRCLE_STEPS_OPEN);
      myStepper.moveTo(steps);
      myStepper.run();
      if (myStepper.distanceToGo() == 0) currentDegree++;
    } else {
      isOpening = false;
      doorStatus = "Ouverte";
    }
  }
}

void closeDoorNonBlocking() {
  if (!isClosing && doorStatus == "Ouverte") {
    isClosing = true;
    currentDegree = MAX_ANGLE;
    doorStatus = "Fermeture";
  }
  if (isClosing) {
    if (currentDegree >= MIN_ANGLE) {
      int steps = map(currentDegree, MIN_ANGLE, MAX_ANGLE, 0, QUART_CIRCLE_STEPS_OPEN);
      myStepper.moveTo(steps);
      myStepper.run();
      if (myStepper.distanceToGo() == 0) currentDegree--;
    } else {
      isClosing = false;
      doorStatus = "Ferme";
    }
  }
}

void analyserCommande(const String& tampon, String& commande, String& arg1, String& arg2) {
  int firstSep = tampon.indexOf(';');
  int secondSep = tampon.indexOf(';', firstSep + 1);
  if (firstSep == -1) {
    commande = tampon;
    return;
  }
  commande = tampon.substring(0, firstSep);
  if (secondSep != -1) {
    arg1 = tampon.substring(firstSep + 1, secondSep);
    arg2 = tampon.substring(secondSep + 1);
  } else {
    arg1 = tampon.substring(firstSep + 1);
  }
}

void serialEvent() {
  String tampon = Serial.readStringUntil('\n');
  tampon.trim();
  String commande, arg1, arg2;
  analyserCommande(tampon, commande, arg1, arg2);

  if (commande == "g_dist") {
    Serial.println(distance);
    showCheck();
  } else if (commande == "cfg") {
    if (arg1 == "alm") {
      alarmLimit = arg2.toInt();
      Serial.println("Configure la distance de l’alarme à " + String(alarmLimit) + " cm");
      showCheck();
    } else if (arg1 == "lim_inf") {
      int val = arg2.toInt();
      if (val >= limSup) {
        Serial.println("Erreur – Limite inférieure >= limite supérieure");
        showPannError();
      } else {
        limInf = val;
        Serial.println("Limite inférieure configurée à " + String(limInf));
        showCheck();
      }
    } else if (arg1 == "lim_sup") {
      int val = arg2.toInt();
      if (val <= limInf) {
        Serial.println("Erreur – Limite supérieure <= limite inférieure");
        showPannError();
      } else {
        limSup = val;
        Serial.println("Limite supérieure configurée à " + String(limSup));
        showCheck();
      }
    }
  } else {
    Serial.println("Commande inconnue");
    showError();
  }
}

void loop() {
  currentTime = millis();
  if (millis() - lastDistanceMeasurement >= 100) {
    lastDistanceMeasurement = millis();
    distance = measureDistance();
    updateDisplay();
  }

  handleAlarmEffects();

  if (distance < 60 && doorStatus == "Ferme" && !isOpening) openDoorNonBlocking();
  else if (distance > 60 && doorStatus == "Ouverte" && !isClosing) closeDoorNonBlocking();

  if (isOpening) openDoorNonBlocking();
  if (isClosing) closeDoorNonBlocking();
}
