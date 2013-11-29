#ifndef MY_SCALE_MODEL_H
#define MY_SCALE_MODEL_H

/*
 * class: CScaleModel
 * 
 * Function: To apply a function to a target value and map the result to a given domain.
 * 
 * XiangXu. August 18th, 2013.
 * 
 * Email: xiangxu@mail.bnu.edu.cn
 */

//! This class is used to map a target value to an given domain with a given function.
class ScaleModel
{
public: 
  ScaleModel(){}
  ~ScaleModel(){}
  
  //! Transfer target form domain [A, B] to [M, N]
  /*!
  \param target the target value we want to convert.
  \param A source domain lower boundary.
  \param B source domain upper boundary.
  \param M destination domain lower boundary.
  \param N destination domain upper boundary.
  \return the value mapped from [A, B] to [M, N] at target.
  */
  static double AB2MN(double target, double A, double B, double M, double N);
   
  //! Normalize target in [targetMin, targetMax] to [0, 1]
  static double Normalize(double target, double targetMin, double targetMax);
  
  //! Apply a function to a target value and map the result to a given domain.
  /*! Target should be between [0, 1]
   *  First y = x function is applied to it,
   *  Then the result is scaled into [A, B]
   */
  static double LinearTrans(double target, double A, double B);
  
  //! Apply a function to a target value and map the result to a given domain.
  /*! Target should be between [0, 1]
   *  First y = 1.0 / (x + 1.0) function is applied to it,
   *  Then the result is scaled into [A, B]
   */
  static double FractTrans(double target, double A, double B);
  
  //! Apply a function to a target value and map the result to a given domain.
  /*! Target should be between [0, 1]
   *  First y = pow(x, 2) function is applied to it,
   *  Then the result is scaled into [A, B]
   */
  static double SquareTrans(double target, double A, double B);
  
  //! Apply a function to a target value and map the result to a given domain.
  /*! Target should be between [0, 1]
   *  First y = log(x + 1.0) function is applied to it,
   *  Then the result is scaled into [A, B]
   */
  static double LogTrans(double target, double A, double B);
  
  //! Apply a function to a target value and map the result to a given domain.
  /*! Target should be between [0, 1]
   *  First y = 1.0 / (pow(x, 2) + 1.0) function is applied to it,
   *  Then the result is scaled into [A, B]
   */
  static double FractSquareTrans(double target, double A, double B);
  
  //! Apply a function to a target value and map the result to a given domain.
  /*! Target should be between [0, 1]
   *  First y = 1.0 / (log(x + 1) + 1.0) function is applied to it,
   *  Then the result is scaled into [A, B]
   */
  static double FractLogTrans(double target, double A, double B);
  
  //! Apply a function to a target value and map the result to a given domain.
  /*! Target should be between [0, 1]
   *  y = exp(x) function is applied to it,
   *  Then the result is scaled into [A, B]
   */
  static double ExpTrans(double target, double A, double B);
  
};



#endif