// Teensy/Arduino
// Maths library
#include <stdio.h>
#include <Math.h> // FIX: Corrected typo from #inlcude and capitalized Math.h

// // BNO055 Setup
// #include <Adafruit_Sensor.h>
// #include <Adafruit_BNO055.h>
// #include <utility/imumaths.h>

// Fuzzy-Logic-Controller-Class
class Fuzzy_Logic_Controller {
  public:
    // FIX: Enums use commas, not semicolons
    enum Sets {
      NB = 0,           // Negative-Big
      NM = 1,           // Negative-Medium
      NS = 2,           // Negative-Small
      ZO = 3,           // Zero
      PS = 4,           // Positive-Small
      PM = 5,           // Positive-Medium
      PB = 6,           // Positive-Big
      NUM_SETS = 7      // Total count
    };

    // Current Gains & Limits (Added these so updateGains can work)
    double Kp_ = 0.0;
    double Kd_ = 0.0;
    
    // Define constraints to prevent instability
    static const double KP_MIN = 0.0;
    static const double KP_MAX = 25.0; // Example values, tune these!
    static const double KD_MIN = 0.0;
    static const double KD_MAX = 10.0;

    static const double MAX_ERROR = 180;            // NAX BNO-Orientation Error
    static const double MAX_DELTA_ERROR = 1200;     // MAX Assumed Gyro Error

    static const double error_mf_points[NUM_SETS][3]; 
    static const double delta_error_mf_points[NUM_SETS][3];

    // Rule Bases
    static const int KP_Rule_Base[NUM_SETS][NUM_SETS];
    static const int KD_Rule_Base[NUM_SETS][NUM_SETS];
    static const int KI_Rule_Base[NUM_SETS][NUM_SETS];


    // For defuzzification
    static const double OUTPUT_CENTROIDS[NUM_SETS];

    // Functions
    double getMembershipValue(double x, double a, double b, double c);
    void fuzzifyInput(double input, const double mf_points[NUM_SETS][3], double *membership_values);
    void ruleInference(const double* error_Membership_Values, 
                       const double* delta_Error_Membership_Values, 
                       double aggregated_Membership_Values[NUM_SETS], 
                       const int Rule_Base[NUM_SETS][NUM_SETS]);
    double defuzzify(const double* aggregated_mu, 
                     const double* output_centroids);
    void updateGains(double error, double delta_error);
};

const double Fuzzy_Logic_Controller::error_mf_points[NUM_SETS][3] = {
    {-180.0,  -180.0,   -90.0},      // NB
    {-180.0,   -90.0,     0.0},      // NM
    { -90.0,     0.0,    90.0},      // NS
    { -20.0,     0.0,    20.0},      // ZO
    {   0.0,    20.0,    40.0},      // PS
    {  20.0,    70.0,   140.0},      // PM
    {  90.0,   180.0,   180.0}       // PB
};

const double Fuzzy_Logic_Controller::delta_error_mf_points[NUM_SETS][3] = {
    {-1200.0,   -1200.0,    -680.0},    // NB
    { -950.0,    -400.0,       0.0},    // NM
    { -240.0,    -100.0,       0.0},    // NS
    {  -80.0,       0.0,      80.0},    // ZO
    {    0.0,     100.0,     240.0},    // PS
    {    0.0,     400.0,     950.0},    // PM
    {  680.0,    1200.0,    1200.0}     // PB
};

const int Fuzzy_Logic_Controller::KP_Rule_Base[NUM_SETS][NUM_SETS] = {
  //        ERROR
  //                      _
    {3, 4, 5, 6, 6, 6, 5},  //
    {2, 3, 4, 6, 5, 5, 4},  //
    {1, 2, 3, 5, 4, 4, 1},  //
    {2, 1, 1, 2, 2, 1, 2},  //    DELTA ERROR
    {5, 4, 3, 1, 3, 2, 1},  //
    {6, 5, 4, 2, 3, 2, 1},  //
    {5, 6, 5, 2, 2, 1, 3}   //  
    //                    -
};

const int Fuzzy_Logic_Controller::KD_Rule_Base[NUM_SETS][NUM_SETS] = {
  //        ERROR
  //                      _
    {2, 1, 0, 0, 0, 1, 2},  // 
    {3, 2, 1, 0, 1, 2, 3},  //
    {4, 3, 2, 1, 2, 3, 4},  //
    {5, 4, 3, 3, 3, 4, 5},  //    DELTA ERROr
    {6, 5, 4, 1, 2, 3, 4},  //  
    {6, 6, 5, 4, 4, 3, 1},  //
    {6, 6, 6, 5, 4, 4, 3}   //
    //                    -
};

const int Fuzzy_Logic_Controller::KI_Rule_Base[NUM_SETS][NUM_SETS] = {
  //        ERROR
  //                      _
    {2, 1, 0, 0, 0, 1, 2},  //
    {2, 1, 0, 0, 0, 1, 2},  //
    {2, 1, 2, 4, 4, 1, 2},  //
    {1, 2, 4, 6, 4, 2, 1},  //    DELTA ERROR
    {2, 1, 4, 4, 2, 1, 2},  //
    {2, 1, 0, 0, 0, 1, 2},  //
    {2, 1, 0, 0, 0, 1, 2}   //
    //                    -
};

const double Fuzzy_Logic_Controller :: OUTPUT_CENTROIDS[NUM_SETS] = {
    -0.5,     //NB
    -0.3,     //NM
    -0.1,     //NS
     0.0,     //ZO
     0.1,     //PS
     0.3,     //PM
     0.5      //PB
};

double Fuzzy_Logic_Controller :: getMembershipValue(double x, double a, double b, double c){
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

void Fuzzy_Logic_Controller::fuzzifyInput(double input, const double mf_points[NUM_SETS][3], double *membership_values){
  for(int i = 0; i < NUM_SETS; i++){
    double a = mf_points[i][0];
    double b = mf_points[i][1];
    double c = mf_points[i][2];
    membership_values[i] = getMembershipValue(input, a, b, c);
  }
}

void Fuzzy_Logic_Controller::ruleInference(const double* error_Membership_Values, const double* delta_Error_Membership_Values, double aggregated_Membership_Values[NUM_SETS], const int Rule_Base[NUM_SETS][NUM_SETS]){
  for(int i = 0; i < NUM_SETS; i++){            
    aggregated_Membership_Values[i] = 0.0; 
  }

  // MIN-Operation for Firing-Strength (Alpha)
  for(int i = 0; i < NUM_SETS; i++){        // i --> Delta-Error --> Rows
    for(int j = 0; j < NUM_SETS; j++){      // j --> Error       --> Cols
      
      // Firing strength calculation
      double alpha = fmin(error_Membership_Values[j], delta_Error_Membership_Values[i]);
      int output_Index = Rule_Base[i][j];

      // Final aggregated membership values
      aggregated_Membership_Values[output_Index] = fmax(alpha, aggregated_Membership_Values[output_Index]);
    }
  }
}

double Fuzzy_Logic_Controller::defuzzify(const double* aggregated_Membership_Values, const double* OUTPUT_CENTROIDS){
  double numerator = 0.0;
  double denominator = 0.0;

  for(int i = 0; i < NUM_SETS; i++){
    denominator += aggregated_Membership_Values[i];
    numerator += aggregated_Membership_Values[i] * OUTPUT_CENTROIDS[i];
  }
  
  if(denominator != 0)  return numerator / denominator;
  else return 0.0;
}

void Fuzzy_Logic_Controller::updateGains(double error, double delta_error){
  // Local arrays
  double mu_e[NUM_SETS];
  double mu_de[NUM_SETS];
  double aggregated_KP[NUM_SETS];
  double aggregated_KD[NUM_SETS];

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
  Kp_ += delta_KP_val;
  Kd_ += delta_KD_val;

  // Constraining values
  Kp_ = fmax(KP_MIN, fmin(KP_MAX, Kp_));
  Kd_ = fmax(KD_MIN, fmin(KD_MAX, Kd_));
}


int main(){
  Fuzzy_Logic_Controller FuzzyController;

  double Kp = 30.0;
  double Kd = 0;

  printf("Kp value : %d ", Kp);
  printf("Kd value : %d ", Kd);

  FuzzyController.updateGains(30.0, 0.0);

  // printf("Updated Kp value : %d", Kp_);
  // printf("Updated Kd value : %d", Kd_);

  return 0;
}