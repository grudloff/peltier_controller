/*
  Temperature Control System with Peltier Devices and PID Controllers

  This Arduino code implements a temperature control system using Peltier devices
  (thermoelectric coolers and heaters) controlled by PID (Proportional-Integral-Derivative)
  controllers. The system is designed to regulate the temperature of a controlled environment
  based on user-defined setpoints.

  Author: Gabriel Rudloff Barison
  Date: 27/09/2023

  Libraries Used:
  - OneWire: For 1-Wire temperature sensor communication.
  - DallasTemperature: For reading temperature values from DS18B20 sensors.
  - L298NX2: For controlling dual H-bridge motor drivers.
  - PID_v1: For implementing PID control.
  - SoftwareReset.hpp: For performing Arduino software resets.
*/

#include <OneWire.h>
#include <DallasTemperature.h>
#include <L298NX2.h>
#include <PID_v1.h>
#include <SoftwareReset.hpp>

// Verbose ?
#define VERBOSE true

// TUNE ?
#define TUNE true

// Define period [ms] between increase in reference temp
#define WAIT_PERIOD 30000  //20 secs

// Define temp change per step
#define TEMP_CHANGE 0.1
#define MAX_TEMP 65
#define MIN_TEMP 5

// Pins H bridge
const unsigned int EN_B = 6;
const unsigned int IN1_B = 11;
const unsigned int IN2_B = 10;

const unsigned int IN1_A = 9;
const unsigned int IN2_A = 8;
const unsigned int EN_A = 5;

// PID values
double coolerA_p = 20;  //
double coolerA_i = 10;  //
double coolerA_d = 20;  //
double heaterA_p = 10;
double heaterA_i = 1.1;
double heaterA_d = 1;
double coolerB_p = 20;  //
double coolerB_i = 10;  //
double coolerB_d = 7;   //
double heaterB_p = 10;
double heaterB_i = 1.1;
double heaterB_d = 1;

const float coolerA_min = 0;
const float coolerA_max = 255;
const float heaterA_min = 0;
const float heaterA_max = 255;
const float coolerB_min = 0;
const float coolerB_max = 255;
const float heaterB_min = 0;
const float heaterB_max = 255;

// Pin temp sensor
const int pinDQ = 13;
const int resolution = 12;

// Buzzer
const int pinBuzzer = 3;
int buzz_flag = 0;
int danger_flag = 0;

OneWire oneWireObjt(pinDQ);
DallasTemperature sensorTemp(&oneWireObjt);
L298NX2 peltiers(EN_A, IN1_A, IN2_A, EN_B, IN1_B, IN2_B);

// Temperatures
double tempA, tempB, TA, TB, TAa, TBa;

// Peltiers
double powerA = 0, powerB = 0;
bool polarityA, polarityB;

// Controllers
PID controlA(&tempA, &powerA, &TA, coolerA_p, coolerA_i, coolerA_d, REVERSE);
PID controlB(&tempB, &powerB, &TB, coolerB_p, coolerB_i, coolerB_d, REVERSE);

// time for temp change
unsigned long time, prev_time = 0;
// time for sensor convertion
unsigned long request_start;

// Temperature ramp start param
int s = 0;

// tuning alternative
int tuning;
String tuningStr;

// delta for histeresis
const float delta = 1;

void setup() {
  Serial.begin(9600);
  Serial.setTimeout(30000);  //30 secs

  // 1-Wire coms start
  sensorTemp.begin();
  sensorTemp.setResolution(resolution);
  sensorTemp.setCheckForConversion(true);  //Use async mode
  sensorTemp.requestTemperatures();
  request_start = millis();

  // Start PID
  TA = -1;
  TB = -1;
  controlA.SetMode(AUTOMATIC);
  controlB.SetMode(AUTOMATIC);

  // Interface
  Serial.println("Send char 'c' to set PID controller to tune.");
  Serial.println("Send char 'x' to set PID params or char 't' to set target temps.");
  Serial.println("Send char 's' to start temperature ramp.");
  delay(1000);
  Serial.print(".");
  delay(1000);
  Serial.print(".");
  delay(1000);
  Serial.print(".");
  delay(1000);
  Serial.print(".");
  delay(1000);
  Serial.println(".");
  analogWrite(pinBuzzer, 127);
  delay(100);
  Serial.println("Start!");
  analogWrite(pinBuzzer, 0);
}

void change_temp() {
  """ Change the target temperature according to the ramp mode. """	
  if (s > 0) {
    // TODO: Move to a timer
    time = millis() % WAIT_PERIOD;
    if (time < prev_time) {
      switch (s) {
        case 1:
          if(TA < MAX_TEMP) TA += TEMP_CHANGE;
          break;
        case 2:
          if(TA > MIN_TEMP) TA -= TEMP_CHANGE;
          break;
        case 3:
          if(TB < MAX_TEMP) TB += TEMP_CHANGE;
          break;
        case 4:
          if(TB > MIN_TEMP) TB -= TEMP_CHANGE;
          break;
      }
    }
    prev_time = time;
  }
}

void temp_sensing() {
  """ Get temperature from sensors. """
  // Wait for conversion to complete
  // NOTE: The conversion takes 750ms in 12bit resolution 
  while (millis() - request_start < 750);
  // Optionally block until conversion is complete, which could be less than 750ms
  // but may lead to some issues in practice
  //sensorTemp.blockTillConversionComplete(resolution, request_start); // wait for async conversion to complete
  tempA = sensorTemp.getTempCByIndex(0);
  tempB = sensorTemp.getTempCByIndex(1);
  if (VERBOSE) {
    Serial.print("tempA:");
    Serial.print(tempA);
    Serial.print("C\t");
    Serial.print("tempB:");
    Serial.print(tempB);
    Serial.print("C\t");
    Serial.print("TA:");
    Serial.print(TA);
    Serial.print("C\t");
    Serial.print("TB:");
    Serial.print(TB);
    Serial.print("C\t");
    Serial.print("TAa:");
    Serial.print(TAa);
    Serial.print("C\t");
    Serial.print("TBa:");
    Serial.print(TBa);
    Serial.println("C");
    Serial.print("deltaA:");
    Serial.print(TA - tempA);
    Serial.print("C\t");
    Serial.print("deltaB:");
    Serial.print(TB - tempB);
    Serial.println("C");
  }
  sensorTemp.requestTemperatures();  //request next measurement
  request_start = millis();
}

void peltier_control() {
  """ Compute a step of the PID controller and update the power and polarity values. """

  // Set PID mode (heater or cooler)
  if (polarityA) {//heater
    controlA.SetControllerDirection(DIRECT);
    controlA.SetOutputLimits(heaterA_min, heaterA_max);
    controlA.SetTunings(heaterA_p, heaterA_i, heaterA_d);
  } else {//cooler
    controlA.SetControllerDirection(REVERSE);
    controlA.SetOutputLimits(coolerA_min, coolerA_max);
    controlA.SetTunings(coolerA_p, coolerA_i, coolerA_d);
  }

  if (polarityB) {//heater
    controlB.SetControllerDirection(DIRECT);
    controlB.SetOutputLimits(heaterB_min, heaterB_max);
    controlB.SetTunings(heaterB_p, heaterB_i, heaterB_d);
  } else {//cooler
    controlB.SetControllerDirection(REVERSE);
    controlB.SetOutputLimits(coolerB_min, coolerB_max);
    controlB.SetTunings(coolerB_p, coolerB_i, coolerB_d);
  }

  // Run PID
  controlA.Compute();
  controlB.Compute();

  if (VERBOSE) {
    Serial.print("powerA:");
    Serial.print(powerA);
    Serial.print(", powerB:");
    Serial.println(powerB);
  }
}

void set_peltiers() {
  """ Set the power and polarity of the peltiers. """

  // int pwm value 0->255
  peltiers.setSpeedA(powerA);
  if (polarityA) {
    peltiers.forwardA();  //heater
  } else {
      peltiers.backwardA();  //cooler
  }
  
  peltiers.setSpeedB(powerB);
  if (polarityB) {
      peltiers.forwardB();  //heater
  } else {
      peltiers.backwardB();  //cooler
  }

}

void beep(int t) {
  """ Make a beep with a period of t ms. """	
  analogWrite(pinBuzzer, 127);
  delay(t);
  analogWrite(pinBuzzer, 0);
  delay(t);
}

void multi_beep(int n, int t) {
  """ Make n beeps with a period of t ms. """	
  for (int i = 0; i < n; i++) {
    beep(t);
  }
}

void check_temp() {
  """ Check if temperature and power are sound.
  
  If temperature is out of bounds, notify via the buzzer with a constant buzz.
  If the power is too high, notify via the buzzer with a series of two beeps. If the
  temperature is changing too fast, notify via the buzzer with a series of three beeps.

  The danger_flag is used to keep track of the number of times the temperature
  has been out of bounds. If it is too high, the system is reset.

  The buzz_flag is used to keep track of the number of times the temperature
  has been changing too fast. After a certain number of beeps, the buzzer is
  turned off.

  """	
  if (danger_flag > 50) {
    softwareReset::standard();
  }
  if (TA == -1 || TB == -1){
    return;
  }
  if (tempA < MIN_TEMP - 5 || tempB < MIN_TEMP - 5 || tempA > MAX_TEMP + 5 || tempB > MAX_TEMP + 5) {
    analogWrite(pinBuzzer, 127);
    danger_flag = danger_flag + 5;
  } else if (powerA == 255 || powerB == 255) {
    multi_beep(3, 60);
    danger_flag = danger_flag + 1;
  } else if (abs(tempA - TA) > TEMP_CHANGE * 3 || abs(tempB - TB) > TEMP_CHANGE * 3) {
    if ((buzz_flag < 20) && (buzz_flag % 5 == 0)) {
      multi_beep(2, 100);
    }
    buzz_flag = buzz_flag + 1;
  } else {
    analogWrite(pinBuzzer, 0);
    buzz_flag = 0;
    danger_flag = 0;
  }
}

void loop() {
  check_temp();
  change_temp();
  temp_sensing();

  if (TA != -1 && TB != -1) {
    // Get polarity
    polarityA = TA > TAa;
    polarityB = TB > TBa;
    
    peltier_control();

    if (powerA > 200 && powerB > 200) {
      int aux = powerA + powerB;
      powerA = powerA / aux * 2 * 200;
      powerB = powerB / aux * 2 * 200;
    }
  }
  if (TA == -1){
    powerA = 0;    
  }
  if (TB == -1){
    powerB = 0;    
  }

  set_peltiers();
}

void serialEvent() {
  """ Used to handle incoming serial commands from a user or external controller.
  It allows users to:
    - Set PID parameters for different controllers (x command).
    - Set target temperatures (t command).
    - Start temperature ramping (s command).
    - Select which controller to tune (c command).
  """


  if (!TUNE){
    return;
  }

  char c = Serial.read();
  Serial.print("Recieved char ");
  Serial.println(c);

  switch(c){
    case 'x':
      set_pid_params();
      break;
    case 't':
      set_temp();
      break;
    case 's':
      set_ramp_mode();
      break;
    case 'c':
      select_pid_to_tune();
      break;
    default:
      Serial.println("No command selected!");
  }
}

void set_pid_params(){
  """ Set pid parameters for the previously selected PID. """
  Serial.println("PID param setting selected.");
  Serial.print("Tuning ");
  Serial.println(tuningStr);

  double *controlP, *controlI, *controlD;
  switch (tuning) {
    case 1:
      controlP = &heaterA_p;
      controlI = &heaterA_i;
      controlD = &heaterA_d;
      break;
    case 2:
      controlP = &coolerA_p;
      controlI = &coolerA_i;
      controlD = &coolerA_d;
      break;
    case 3:
      controlP = &heaterB_p;
      controlI = &heaterB_i;
      controlD = &heaterB_d;
      break;
    case 4:
      controlP = &heaterB_p;
      controlI = &heaterB_i;
      controlD = &heaterB_d;
      break;
  break;
  }

  Serial.println("Current values:");
  Serial.print("kp:");
  Serial.println(*controlP);
  Serial.print("ki:");
  Serial.println(*controlI);
  Serial.print("kd:");
  Serial.println(*controlD);

  Serial.print("kp:");
  float kp = Serial.parseFloat();
  Serial.println(kp);
  *controlP = kp;
  Serial.print("ki:");
  float ki = Serial.parseFloat();
  Serial.println(ki);
  *controlI = ki;
  Serial.print("kd:");
  float kd = Serial.parseFloat();
  Serial.println(kd);
  *controlD = kd;
}

void set_temp(){
  """ Set target temperatures. """	
  if (TA == -1 || TB == -1) {
    if (TA == -1) TAa = tempA;
    if (TB == -1) TBa = tempB;
    Serial.print("References set to TAa=");
    Serial.print(TAa);
    Serial.print("C,");
    Serial.print("TBa=");
    Serial.print(TBa);
    Serial.println("C");
    s = 0;
    Serial.println("Ramp Stoped!");
  }
  // set temperature
  Serial.println("Temperature setting selected.");
  Serial.print("Current values are TA=");
  Serial.print(TA);
  Serial.print(" and TB=");
  Serial.println(TB);
  Serial.print("New TA:");
  TA = Serial.parseFloat();
  Serial.println(TA);
  Serial.print("New TB:");
  TB = Serial.parseFloat();
  Serial.println(TB);
}

void set_ramp_mode(){
  """ Set ramp mode. Avaliable modes are:
    OFF: No ramping.
    TX+: Ramp up the temperature of controller X.
    TX-: Ramp down the temperature of controller X.
    Where X is A or B.
  """
  Serial.println("Ramp modes: 0 OFF, 1 TA+, 2 TA-, 3 TB+, 4 TB-");
  Serial.print("Select ramp mode:");
  s = Serial.parseInt();
  Serial.println(s);
  Serial.println("Temperature ramp started!");
}

void select_pid_to_tune(){
  """ Select which pid to tune. The parameters for heating and cooling are
  different, so the mode and controller must be selected.
  """	
  Serial.println("Tuning: heaterA 1, coolerA 2, heaterB 3, coolerB 4");
  Serial.println("Input int in [1,4]:");
  tuning = Serial.parseInt();
  switch (tuning) {
    case 1:
      tuningStr = "heaterA";
      break;
    case 2:
      tuningStr = "coolerA";
      break;
    case 3:
      tuningStr = "heaterB";
      break;
    case 4:
      tuningStr = "coolerB";
      break;
    default:
      tuningStr = "No controller";
      Serial.println("No tuning selected!");
  }
  Serial.print(tuningStr);
  Serial.println(" selected!");
}