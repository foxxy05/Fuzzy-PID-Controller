// Teensy / Arduino
#include <math.h>
#define TEENSY 13

// I2C Communication
#include <Wire.h>
int8_t slaveAddr = 5;

// BTS Setup
#include <BTS7960.h>
#define maxPWM 50
// Add proper pins for PWM and enable according to board of choice
// Directly add the PWM and enable pins when declaring the object
BTS7960 FW(4, 5);
BTS7960 LW(0, 1);
BTS7960 RW(2, 3);

// Global Velocity Variables
#define sqrt3by2 0.8660254038
#define minus1by2 -0.5000
#define constVector 1
#define arraySize 4  // LX  LY  L2  R2
int8_t buffer = 10;
int8_t receivedData[arraySize] = { 0 };

// Navigation Variables
int16_t wFW = 0, wLW = 0, wRW = 0;
int16_t Vx = 0, Vy = 0;
int16_t VxG = 0, VyG = 0;
int16_t omega = 0;

// BNO
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#define PI 3.1415962

double targetAngle = 0;
double currentAngle = 0;

Adafruit_BNO055 bno = Adafruit_BNO055();

// //PID
double currentTime = 0;
double previousTime = 0;
double error = 0;
double previousError = 0;
double derivative = 0;
double Kp = 8.0;
double Kd = 68;
double Ki = 0;
double PID = 0;

// // FUZZY-PID-CONTROLLER
double fuzzyCurrentTime = 0;
double fuzzyPreviousTime = 0;
double fuzzyError = 0;
double fuzzyPreviousError = 0;

enum Sets{
   NB = 0,        // Negative-Big
   NM = 1,        // Negative-Medium
   NS = 2,        // Negative-Small
   ZO = 3,        // Zero
   PS = 4,        // Positive-Small
   PM = 5,        // Positive-Medium
   PB = 6,        // Positive-Big
   NUM_SETS = 7   // Total count
};

// Constant Data
static const double KP_MIN = 0.0;
static const double KP_MAX = 25.0; // Example values, tune these!
static const double KD_MIN = 0.0;
static const double KD_MAX = 10.0;
static const double MAX_ERROR = 180; // NAX BNO-Orientation Error
static const double MAX_DELTA_ERROR = 1200;

const double error_mf_points[NUM_SETS][3] = {
    {-180.0,   -180.0,      -90.0},       // NB
    {-180.0,    -90.0,        0.0},       // NM
    { -90.0,      0.0,       90.0},       // NS
    { -20.0,      0.0,       20.0},       // ZO
    {   0.0,     20.0,       40.0},       // PS
    {  20.0,     70.0,      140.0},       // PM
    {  90.0,    180.0,      180.0}        // PB
};

const double delta_error_mf_points[NUM_SETS][3] = {
    {-1200.0,     -1200.0,    -680.0},    // NB
    { -950.0,      -400.0,       0.0},    // NM
    { -240.0,      -100.0,       0.0},    // NS
    {  -80.0,         0.0,      80.0},    // ZO
    {    0.0,       100.0,     240.0},    // PS
    {    0.0,       400.0,     950.0},    // PM
    {  680.0,      1200.0,    1200.0}     // PB
};

const int KP_Rule_Base[NUM_SETS][NUM_SETS] = {
    //        ERROR
    //                      _
    {3, 4, 5, 6, 6, 6, 5}, //
    {2, 3, 4, 6, 5, 5, 4}, //
    {1, 2, 3, 5, 4, 4, 1}, //
    {2, 1, 1, 2, 2, 1, 2}, //    DELTA ERROR
    {5, 4, 3, 1, 3, 2, 1}, //
    {6, 5, 4, 2, 3, 2, 1}, //
    {5, 6, 5, 2, 2, 1, 3}  //
                           //                    -
};

const int KD_Rule_Base[NUM_SETS][NUM_SETS] = {
    //        ERROR
    //                      _
    {2, 1, 0, 0, 0, 1, 2}, //
    {3, 2, 1, 0, 1, 2, 3}, //
    {4, 3, 2, 1, 2, 3, 4}, //
    {5, 4, 3, 3, 3, 4, 5}, //    DELTA ERROr
    {6, 5, 4, 1, 2, 3, 4}, //
    {6, 6, 5, 4, 4, 3, 1}, //
    {6, 6, 6, 5, 4, 4, 3}  //
                           //                    -
};

// const int Fuzzy_Logic_Controller::KI_Rule_Base[NUM_SETS][NUM_SETS] = {
//   //        ERROR
//   //                      _
//     {2, 1, 0, 0, 0, 1, 2},  //
//     {2, 1, 0, 0, 0, 1, 2},  //
//     {2, 1, 2, 4, 4, 1, 2},  //
//     {1, 2, 4, 6, 4, 2, 1},  //    DELTA ERROR
//     {2, 1, 4, 4, 2, 1, 2},  //
//     {2, 1, 0, 0, 0, 1, 2},  //
//     {2, 1, 0, 0, 0, 1, 2}   //
//     //                    -
// };

const double OUTPUT_CENTROIDS[NUM_SETS] = {
    -0.200,  // NB
    -0.150,  // NM
    -0.075,  // NS
     0.000,  // ZO
     0.075,  // PS
     0.150,  // PM
     0.200   // PB
};

// Fuzzy-PID-Controller Functions
double getMembershipValue(double x, double a, double b, double c);
void fuzzifyInput(double input, const double mf_points[NUM_SETS][3], double *membership_values);
void ruleInference(const double *error_Membership_Values, const double *delta_Error_Membership_Values, double aggregated_Membership_Values[NUM_SETS], const int Rule_Base[NUM_SETS][NUM_SETS]);
double defuzzify(const double *aggregated_Membership_Values, const double *OUTPUT_CENTROIDS);
void updateGains(double error, double delta_error, double *Kp, double *Kd);
double calculateDeltaError(double fuzzyError);

// SETUP-CODE
void setup() {
  pinMode(TEENSY, OUTPUT);
  digitalWrite(TEENSY, HIGH);

  Serial.begin(115200);
  Serial.print("Ganpati Bappa Morya!");

  // Setting-up I communication between ESP32 and Arduino
  Wire2.begin();
  Serial.println("I2C Master Ready!");

  // Setting the enable as HIGH for each BTS
  FW.setEnable(true);
  LW.setEnable(true);
  RW.setEnable(true);

  // Initiating BNO and setting extCrystal as true
  if (!bno.begin()) {
    // Serial.print("No BNO055 detected");
    bno.setExtCrystalUse(true);
    while (1)
      ;
  }
  delay(1000);
}

void loop() {
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  currentAngle = 0;
  wFW = 0;
  wLW = 0;
  wRW = 0;
  Vx = 0, Vy = 0;
  VxG = 0, VyG = 0;
  omega = 0;

  currentAngle = euler.x();
  // Serial.println(currentAngle);
  float theta = currentAngle * PI / 180.0;

  requestPS4();

  Vy = receivedData[0];  //Y-Component of the Joystick is the X component of the Chassis
  Vx = receivedData[1];
  omega = receivedData[3] + receivedData[2];
  VxG = Vx * cos(-theta) - Vy * sin(-theta);  // Local X
  VyG = Vx * sin(-theta) + Vy * cos(-theta);  // Local Y


  if (abs(omega) < 10) {
    error = currentAngle - targetAngle;
    if (error > 180) error -= 360;
    if (error < -180) error += 360;

    updateGains(error, calculateDeltaError(error), &Kp, &Kd);
    omega = PIDControl(error);

    previousError = error;
    previousTime = currentTime;
  } 
  else {
    targetAngle = currentAngle;
  }

  // Front wheel (120d)
  wFW = constrain(constVector * (VxG*(minus1by2) + VyG*(sqrt3by2) + omega), -maxPWM, maxPWM);
  // Left Wheel (240d)
  wLW = constrain(constVector * (VxG*(minus1by2) - VyG*(sqrt3by2) + omega), -maxPWM, maxPWM);
  // Right Wheel (0d)
  wRW = constrain(constVector * (VxG - VyG*(0) + omega), -maxPWM, maxPWM);

  // Sending equation's values to BTS
  FW.rotate(wFW);
  RW.rotate(wRW);
  LW.rotate(wLW);

  // targetAngle = currentAngle;
  // printEq();
  printPS();
  delay(10);
}

void requestPS4() {
  Wire2.requestFrom(slaveAddr, sizeof(receivedData));

  int i = 0;
  while (Wire2.available()) {
    uint8_t raw = Wire2.read();
    if (i == 0) receivedData[0] = map(raw, 0, 255, -127, 127);       // LX
    else if (i == 1) receivedData[1] = map(raw, 0, 255, -127, 127);  // LY
    else if (i == 2) receivedData[2] = map(raw, 0, 255, 0, -127);    // L2
    else if (i == 3) receivedData[3] = map(raw, 0, 255, 0, 127);     // R2

    i++;
  }

  if (abs(receivedData[0]) < buffer) receivedData[0] = 0;
  if (abs(receivedData[1]) < buffer) receivedData[1] = 0;
  if (abs(receivedData[2]) < buffer) receivedData[2] = 0;
  if (abs(receivedData[3]) < buffer) receivedData[3] = 0;
}


double PIDControl(int error) {
  currentTime = millis();
  int deltaT = (currentTime - previousTime);
  if (deltaT <= 0) {
    deltaT = 1;
  }
  derivative = (error - previousError) / (deltaT);
  PID = (Kp * error) + (Kd * derivative);
  previousError = error;
  previousTime = currentTime;
  PID = constrain(PID, -maxPWM, maxPWM);
  if (abs(PID) <= 1) {
    PID = 0;
  }
  return PID;
}

void printPS() {
  Serial.print("LX : ");
  Serial.print(receivedData[0]);
  Serial.print("   ||   LY : ");
  Serial.print(receivedData[1]);
  Serial.print("   ||   L2 : ");
  Serial.print(receivedData[2]);
  Serial.print("   ||   R2 : ");
  Serial.println(receivedData[3]);
}

void printEq() {
  Serial.print("ANGLE : ");
  Serial.print(currentAngle);
  Serial.print("   ||   wLW : ");
  Serial.print(wLW);
  Serial.print("   ||   wFW : ");
  Serial.print(wFW);
  Serial.print("   ||   wRW : ");
  Serial.println(wLW);
} 


// FUZZY-PID-CONTROLLER
double getMembershipValue(double x, double a, double b, double c){
   if (x >= a && x <= b){
      if (b == a)
         return 1.0;
      return (x - a) / (b - a);
   }
   else if (x > b && x <= c){
      if (c == b)
         return 1.0;
      return (c - x) / (c - b);
   }
   return 0.0;
}

void fuzzifyInput(double input, const double mf_points[NUM_SETS][3], double *membership_values){
   for (int i = 0; i < NUM_SETS; i++){
      double a = mf_points[i][0];
      double b = mf_points[i][1];
      double c = mf_points[i][2];
      membership_values[i] = getMembershipValue(input, a, b, c);
   }
}

void ruleInference(const double *error_Membership_Values, const double *delta_Error_Membership_Values, double aggregated_Membership_Values[NUM_SETS], const int Rule_Base[NUM_SETS][NUM_SETS]){
   for (int i = 0; i < NUM_SETS; i++){
      aggregated_Membership_Values[i] = 0.0;
   }

   // MIN-Operation for Firing-Strength (Alpha)
   for (int i = 0; i < NUM_SETS; i++){       // i --> Delta-Error --> Rows
      for (int j = 0; j < NUM_SETS; j++){    // j --> Error       --> Cols

         // Firing strength calculation
         double alpha = fmin(error_Membership_Values[j], delta_Error_Membership_Values[i]);
         int output_Index = Rule_Base[i][j];

         // Final aggregated membership values
         aggregated_Membership_Values[output_Index] = fmax(alpha, aggregated_Membership_Values[output_Index]);
      }
   }
}

double defuzzify(const double *aggregated_Membership_Values, const double *OUTPUT_CENTROIDS){
   double numerator = 0.0;
   double denominator = 0.0;

   for (int i = 0; i < NUM_SETS; i++){
      denominator += aggregated_Membership_Values[i];
      numerator += aggregated_Membership_Values[i] * OUTPUT_CENTROIDS[i];
   }

   if (denominator != 0)
      return numerator / denominator;
   else
      return 0.0;
}

void updateGains(double error, double delta_error, double *Kp, double *Kd){
   double Kp_Base = *Kp;
   double Kd_Base = *Kd;
   // Local arrays
   double mu_e[NUM_SETS];
   double mu_de[NUM_SETS];
   double aggregated_KP[NUM_SETS];
   double aggregated_KD[NUM_SETS];

   // Input clipping
   if (error > MAX_ERROR)
      error = MAX_ERROR;
   if (error < -MAX_ERROR)
      error = -MAX_ERROR;
   if (delta_error > MAX_DELTA_ERROR)
      delta_error = MAX_DELTA_ERROR;
   if (delta_error < -MAX_DELTA_ERROR)
      delta_error = -MAX_DELTA_ERROR;

   // Fuzzification
   fuzzifyInput(error, error_mf_points, mu_e);
   fuzzifyInput(delta_error, delta_error_mf_points, mu_de);

   // Inference Engine
   ruleInference(mu_e, mu_de, aggregated_KP, KP_Rule_Base);
   ruleInference(mu_e, mu_de, aggregated_KD, KD_Rule_Base);

   // Defuzzification
   double delta_KP_val = defuzzify(aggregated_KP, OUTPUT_CENTROIDS);
   double delta_KD_val = defuzzify(aggregated_KD, OUTPUT_CENTROIDS);

   // Update Gains
   *Kp = Kp_Base * (1 + delta_KP_val);
   *Kd = Kd_Base * (1 + delta_KD_val);

   // Constraining values
   if (*Kp < KP_MIN)
      *Kp = KP_MIN;
   if (*Kp > KP_MAX)
      *Kp = KP_MAX;
   if (*Kd < KD_MIN)
      *Kd = KD_MIN;
   if (*Kd > KD_MAX)
      *Kd = KD_MAX;
}

double calculateDeltaError(double fuzzyError){
  fuzzyCurrentTime = millis();
  int fuzzyDeltaT = (fuzzyCurrentTime - fuzzyPreviousTime);
  if (fuzzyDeltaT <= 0) {
    fuzzyDeltaT = 1;
  }
  double deltaError = (fuzzyError - fuzzyPreviousError) / (fuzzyDeltaT);
  fuzzyPreviousError = fuzzyError;
  fuzzyPreviousTime = fuzzyCurrentTime;

  return deltaError;
}