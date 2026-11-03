#include <Arduino.h>
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

#define NTC_PIN A0
#define HEATER_PIN 9
#define FILA_PIN 8
#define STEP_PIN 2
#define DIR_PIN 3
#define BUZZ_PIN A1
#define MAX_TEMP 250

#include <EncButton.h>
EncButton encBut(7, 6, 5, INPUT, INPUT);  //s1, s2, key

#include <AccelStepper.h>
AccelStepper stepper(1, STEP_PIN, DIR_PIN);

bool motorOn = false;
int motorSpeed = 2;

LiquidCrystal_I2C lcd(0x27,16,2);  // set the LCD address to 0x27 for a 16 chars and 2 line display

class NTC100K {
  private:
    int analogPin;
    float nominalResistance; // 100000 Ω
    float bCoefficient;      // 3950 K
    float seriesResistance;  // 100000 Ω
    float nominalTemp;       // 25 °C
    float calibrationOffset;
    
  public:
    NTC100K(int pin, float calibration = 0) {
      analogPin = pin;
      nominalResistance = 100000.0;
      bCoefficient = 3950.0;
      seriesResistance = 100000.0;
      nominalTemp = 25.0;
      calibrationOffset = calibration;
    }
    
    void setCalibration(float offset) {
      calibrationOffset = offset;
    }
    
    float readTemperature() {
      // int adcValue = readAveragedADC(16);
      // int adcValue = readAveragedADC(1);
      int adcValue = motorOn?readAveragedADC(1):readAveragedADC(16);
      
      // Защита от крайних значений
      if (adcValue <= 0) adcValue = 1;
      if (adcValue >= 1023) adcValue = 1022;
      
      double voltage = adcValue * (5.0 / 1023.0);
      double resistance = seriesResistance * (voltage / (5.0 - voltage));
      
      // Проверка на корректность сопротивления
      if (resistance <= 0) return -100;
      
      // Формула Стейнхарта-Харта
      // double tempK = 1.0 / (1.0/(nominalTemp + 273.15) + 1.0/bCoefficient * log(resistance/nominalResistance));
      float tempK = 1.0 / (log(resistance / nominalResistance) / bCoefficient + 1.0 / 298.15);
      return (tempK - 273.15) + calibrationOffset;
    }

    float readTemperature2() {
      // int adcValue = readAveragedADC(16);
      // int adcValue = readAveragedADC(1);
      int adcValue = motorOn?readAveragedADC(1):readAveragedADC(16);
      
      // Защита от крайних значений
      if (adcValue <= 0) adcValue = 1;
      if (adcValue >= 1023) adcValue = 1022;

      float tr = 1023.0 / adcValue - 1;
      tr = seriesResistance / tr;
      // Serial.print("R=");
      // Serial.print(tr);
      // Serial.print(", t=");

      float steinhart;
      steinhart = tr / nominalResistance; // (R/Ro)
      steinhart = log(steinhart); // ln(R/Ro)
      steinhart /= bCoefficient; // 1/B * ln(R/Ro)
      steinhart += 1.0 / (nominalTemp + 273.15); // + (1/To)
      steinhart = 1.0 / steinhart; // Invert
      steinhart -= 273.15; 
      Serial.println(steinhart);

      return steinhart;
    }



  private:
    int readAveragedADC(int samples) {
      long sum = 0;
      for (int i = 0; i < samples; i++) {
        if(i>0)
          delay(1);
        sum += analogRead(analogPin);
      }
      return sum / samples;
    }
};

NTC100K ntc(NTC_PIN);

enum MenuState { DISPLAY_TEMP, DISPLAY_MOTOR };
MenuState menuState = DISPLAY_TEMP;
unsigned long showTempAt = 0;


float curTemp = 0;
double curPower = 0;

bool haveFila = false;
bool prevHaveFila = false;

unsigned long buzzUntil = 0;
int buzzV = 200;
void buzz(int ms){
  buzzUntil = millis()+ms;
}
void buzzTick(unsigned long ms){
  if(buzzUntil==0)
    return;
  if(buzzUntil>ms){
    analogWrite(BUZZ_PIN,buzzV);
  }else{
    analogWrite(BUZZ_PIN,0);
    buzzUntil = 0;
  }
}

class PIDController {
  private:
    double kp, ki, kd;
    double setpoint;
    byte setPointFrom;
    double integral, prevError;
    unsigned long prevTime;
    double outMin, outMax;
    
  public:
    PIDController(double p, double i, double d, double sp) {
      kp = p; ki = i; kd = d; setpoint = sp;
      setPointFrom = 0;
      integral = 0; prevError = 0;
      prevTime = millis();
      outMin = 0; outMax = 255;
    }
    
    void setTunings(double p, double i, double d) {
      kp = p; ki = i; kd = d;
    }
    
    void setSetpoint(double sp) { 
      setpoint = sp;
      setPointFrom = curTemp<sp?-1:1;
    }
    double getSetpoint(){return setpoint;}
    void setOutputLimits(double min, double max) { outMin = min; outMax = max; }

    void checkSetpointReached(float curTemp){
      if(setPointFrom==0)
        return;
      if( (setPointFrom==-1 && curTemp>=setpoint)
        || (setPointFrom==1 && curTemp<=setpoint)){
        setPointFrom=0;
        buzz(300);
      }else{
        // Serial.print("setPointFrom=");Serial.println(setPointFrom);
        // Serial.print("curTemp=");Serial.println(curTemp);
        // Serial.print("setpoint=");Serial.println(setpoint);
      }
    }
    
    double compute(double input) {
      unsigned long now = millis();
      double dt = (now - prevTime) / 1000.0;
      
      if (dt < 1.0) return -1; // Слишком часто
      
      double error = setpoint - input;
      
      // Пропорциональная
      double proportional = kp * error;
      
      // Интегральная
      integral += error * dt;
      double integralTerm = ki * integral;
      
      // Дифференциальная
      double derivative = (error - prevError) / dt;
      double derivativeTerm = kd * derivative;
      
      // Суммирование
      double output = proportional + integralTerm + derivativeTerm;
      
      // Ограничение и anti-windup
      if (output > outMax) {
        output = outMax;
        integral -= error * dt; // Anti-windup
      } else if (output < outMin) {
        output = outMin;
        integral -= error * dt; // Anti-windup
      }
      
      prevError = error;
      prevTime = now;

      Serial.print(">Temp:");Serial.println(input);
      Serial.print(">Power:");Serial.println(output);

      return output;
    }
};

PIDController pid(10, 0.3, 20, 160.0);
unsigned long ranMin = 0;

void updateDisplay() {
  lcd.clear();
  
  if (menuState == DISPLAY_TEMP) {
    lcd.print("T:");
    lcd.print(curTemp, 1);
    lcd.print("C S:");
    lcd.print(pid.getSetpoint(), 1);
    lcd.print("C");
    
    lcd.setCursor(0, 1);
    lcd.print("P:");
    lcd.print((curPower/255)*100, 0);
    lcd.print("%");
    lcd.print(haveFila?" ON":" OFF");
    lcd.print(" R:");
    lcd.print(ranMin);
  } else if(menuState==DISPLAY_MOTOR){
    lcd.setCursor(0, 0);
    lcd.print("Motor is ");
    lcd.print(motorOn?"ON":"OFF");
    lcd.setCursor(0, 1);
    lcd.print("Motor speed: ");
    lcd.print(motorSpeed);
  }
}

void showMotor(){
  menuState = DISPLAY_MOTOR;
  showTempAt = millis()+3000;
  updateDisplay();
}

void setup() {
  Serial.begin(9600);
  lcd.init();                      // initialize the lcd 
  lcd.backlight();
  
  pinMode(HEATER_PIN, OUTPUT);
  pinMode(FILA_PIN, INPUT_PULLUP);
  pinMode(BUZZ_PIN, OUTPUT);
  
  pid.setOutputLimits(0, 255);

  encBut.setFastTimeout(100);
  
  lcd.print("Filament Maker");
  lcd.setCursor(0,1);
  lcd.print("Machine");
  delay(1000);

  stepper.setMaxSpeed(1500);
  stepper.setSpeed(0);
  // stepper.setCurrentPosition(0);
  // stepper.setMaxSpeed(1000); // Set maximum speed value for the stepper
  // stepper.setAcceleration(500); // Set acceleration value for the stepper
  buzz(100);
}

unsigned long nextTempCheck = 0;
unsigned long motorStarted = 0;

void loop() {

  unsigned long mil = millis();

  buzzTick(mil);

  haveFila = !digitalRead(FILA_PIN);

  if(showTempAt>0 && mil>showTempAt){
    menuState = DISPLAY_TEMP;
    showTempAt = 0;
  }


  encBut.tick();

  if(encBut.click()){
    Serial.println("clicked");
    motorOn = !motorOn;
    showMotor();
    if(motorOn){
      motorStarted = mil;
      stepper.setSpeed(motorSpeed*100);
    }
  }else if(encBut.turn()){
    
    Serial.print("turn: dir ");
    Serial.print(encBut.dir());
    Serial.print(", fast ");
    Serial.print(encBut.fast());
    Serial.print(", hold ");
    Serial.print(encBut.pressing());
    Serial.print(", counter ");
    Serial.print(encBut.counter);
    Serial.print(", clicks ");
    Serial.println(encBut.getClicks());

    if(encBut.pressing()){
      // motorSpeed += (encBut.fast()?10:1) * encBut.dir();
      motorSpeed += encBut.dir();
      motorSpeed = constrain(motorSpeed,-10,10);
      stepper.setSpeed(motorSpeed*100);
      showMotor();
    }else{
      pid.setSetpoint(pid.getSetpoint() + (encBut.fast()?10:1) * encBut.dir());
      updateDisplay();
    }
  }

  if(motorOn)
    ranMin = (mil - motorStarted)/60000;

  if(motorOn && haveFila){
    stepper.runSpeed();
  }

  if(mil>nextTempCheck){
    nextTempCheck+=1000;

    curTemp = ntc.readTemperature2();

    pid.checkSetpointReached(curTemp);

    curPower = pid.compute(curTemp);

    if(haveFila == false || curTemp>=MAX_TEMP){
      analogWrite(HEATER_PIN, 0);
    }
    else if (curPower >= 0) {
      analogWrite(HEATER_PIN, (int)curPower);
    }
    if(!motorOn)
      updateDisplay();
  }


  if (Serial.available()) {
    delay(100);
    String s = Serial.readString();
    double sp = s.toDouble();
    pid.setSetpoint(sp);
  }


  if(haveFila==false && prevHaveFila==true){
    buzz(300);
    motorOn = false;
    showMotor();

  }
  prevHaveFila = haveFila;
  if(!haveFila){
    lcd.noBacklight();
    delay(500);
    lcd.backlight();
    delay(500);
  }
}

