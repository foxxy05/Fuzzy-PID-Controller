// Teensy/Arduino

// BNO055 Setup
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// Fuzzy-Logic-Controller-Class
class Fuzzy_Logic_Controller{
  private:
  enum Sets{
    NB = 0;           //Negative-Big
    NM = 1;           //Negative-Medium
    NS = 2;           //Negative-Small
    ZO = 3;           //Zero
    PS = 4;           //Positive-Small
    PM = 5;           //Positive-Medium
    PB = 6;           //Positive-Big
    NUM_SETS = 7; 
  };

  static const double MAX_ERROR = 180;          // BNO-Orientation Error
  static const double MAX_DELTA_ERROR = 1200;   // Assumed Gyro Error
  
  // Membership function (MF) for Error(E)
  static const double error_mf_points[NUM_SETS][3]{
    // a        b         c
    {-180.0,  -180.0,   -90.0},      // NB (left shoulder)
    {-180.0,  -90.0,      0.0},         // NM
    {-90.0,     0.0,     90.0},         // NS
    {-20.0,     0.0,     20.0},         // ZO
    {0.0,      20.0,     40.0},         // PS
    {20.0,     70.0,     140.0},         // PM
    {90.0,    180.0,     180.0}          // PB (right shoulder)
  }

  static const double delta_error_mf_points[NUM_SETS][3]{
    // a        b         c
    {-1200.0,   -1200.0,    -680.0},          // NB (left shoulder)
    { -950.0,    -400.0,       0.0},          // NM
    { -240.0,    -100.0,       0.0},          // NS
    {  -80.0,       0.0,      80.0},          // ZO
    {    0.0,     100.0,     240.0},          // PS
    {    0.0,     400.0,     950.0},          // PM
    {  680.0,    1200.0,     1200.0}          // PB (right shoulder)
  }

  double getMembership(double x, double a, double b, double c);
  void fuzzifyInput(double error, double delta_Error);
  public:
};

double Fuzzy_Logic_Controller :: getMembership(double x, double a, double b, double c){
  if(x >= a && x <= b){
    if(b == a) return 1.0;
    return (x-a)/(b-a);
  }

  else if(x > b && x <= c){
    if(c==b) return 1.0;
    return (c-x)/(c-b);
  } 

  return 0.0;
}

double Fuzzy_Logic_Controller :: fuzzifyInput(double error, double delta_error){

}





