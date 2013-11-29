
#include "scale_model.h"
#include <iostream>
#include <stdio.h>
#include <math.h>
#include <assert.h>

double ScaleModel::AB2MN(double target, double A, double B, double M, double N)
{
  /**
   * We have the equation: (B - A)/(N - M) = (target - A)/(value - M)
   */
  double factor = (N - M) / (B - A);
  double value = factor * (target - A) + M;
  
  return value;
}

double ScaleModel::Normalize(double target, double targetMin, double targetMax)
{
  return AB2MN(target, targetMin, targetMax, 0.0, 1.0);
}

double ScaleModel::LinearTrans(double target, double A, double B)
{
  double functionValue = target;
  
  return AB2MN(functionValue, 0.0, 1.0, A, B);
}
  
double ScaleModel::FractTrans(double target, double A, double B)
{
  double functionValue = 1.0 / (target + 1.0);
  
  return AB2MN(functionValue, 0.5, 1.0, A, B);
}
  
double ScaleModel::SquareTrans(double target, double A, double B)
{
  assert(target >= 0 && target <= 1);
  
  double functionValue = pow(target, 2);
  
  double result =  AB2MN(functionValue, 0.0, 1.0, A, B);
  
  assert(result >= A && result <= B);
  
  return result;
}
  
double ScaleModel::LogTrans(double target, double A, double B)
{
  double functionValue = log(target + 1.0);
  
  return AB2MN(functionValue, 0.0, log(2.0), A, B); 
}
  
double ScaleModel::FractSquareTrans(double target, double A, double B)
{
  double functionValue = 1.0 / (pow(target, 2) + 1.0);
  
  return AB2MN(functionValue, 0.5, 1.0, A, B); 
}
  
double ScaleModel::FractLogTrans(double target, double A, double B)
{
  double functionValue = 1.0 / (log(target + 1) + 1.0);
  
  return AB2MN(functionValue, 0.5, 1.0 / (log(2.0) + 1), A, B); 
}
  
double ScaleModel::ExpTrans(double target, double A, double B)
{
  double functionValue = exp(target);

  double result =  AB2MN(functionValue, 1.0, exp(1.0), A, B); 
  
//  assert(result >= A && result <= B);
  
  return result;
}