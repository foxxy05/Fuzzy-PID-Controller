#include <stdio.h>
#include <math.h>

enum Sets
{
   NB = 0,      // Negative-Big
   NM = 1,      // Negative-Medium
   NS = 2,      // Negative-Small
   ZO = 3,      // Zero
   PS = 4,      // Positive-Small
   PM = 5,      // Positive-Medium
   PB = 6,      // Positive-Big
   NUM_SETS = 7 // Total count
};

double Kp_ = 0.0;
double Kd_ = 0.0;

// Constant Data
static const double KP_MIN = 0.0;
static const double KP_MAX = 25.0; // Example values, tune these!
static const double KD_MIN = 0.0;
static const double KD_MAX = 10.0;
static const double MAX_ERROR = 180; // NAX BNO-Orientation Error
static const double MAX_DELTA_ERROR = 1200;

const double error_mf_points[NUM_SETS][3] = {
    {-180.0,  -180.0,   -90.0},      // NB
    {-180.0,   -90.0,     0.0},      // NM
    { -90.0,     0.0,    90.0},      // NS
    { -20.0,     0.0,    20.0},      // ZO
    {   0.0,    20.0,    40.0},      // PS
    {  20.0,    70.0,   140.0},      // PM
    {  90.0,   180.0,   180.0}       // PB
};

const double delta_error_mf_points[NUM_SETS][3] = {
    {-1200.0,   -1200.0,    -680.0},    // NB
    { -950.0,    -400.0,       0.0},    // NM
    { -240.0,    -100.0,       0.0},    // NS
    {  -80.0,       0.0,      80.0},    // ZO
    {    0.0,     100.0,     240.0},    // PS
    {    0.0,     400.0,     950.0},    // PM
    {  680.0,    1200.0,    1200.0}     // PB
};