#ifndef FORECAST_T
#define FORECAST_T

#include "Main/headfiles.h"
#include <iostream>
#include <thread>
#include <vector>
#include <memory>
#include <eigen3/Eigen/Dense>

using mat4f4 = Eigen::Matrix<float, 4, 4>;
using mat4f2 = Eigen::Matrix<float, 4, 2>;
using mat2f4 = Eigen::Matrix<float, 2, 4>;
using mat2f2 = Eigen::Matrix<float, 2, 2>;
using vec4f  = Eigen::Matrix<float, 4, 1>;
using vec2f  = Eigen::Matrix<float, 2, 1>;

//Tool class
class Kalman_t  {
public:
    Kalman_t();
    void KalmanInit(float dt, float T_R, float T_Q);
    ~Kalman_t(){};
    friend class Forecast;
private:
    mat4f4 Diag4f;
    mat2f2 Diag2f;

    vec4f KalmanForecast();
    vec4f KalmanFilter(vec2f dat);

    vec4f X_last;
    vec4f X_mid;
    vec4f X_now;
    mat4f4 P_last;
    mat4f4 P_mid;
    mat4f4 P_now;
    mat4f2 Kg;
    mat4f4 A;
    mat4f4 Q;
    mat2f2 R;
    mat2f4 H;

   /*
   float X_last = 0.0f; //ï؟½1ï؟½7ï؟½1ï؟½7ï؟½0ï؟½5ï؟½0ï؟½2ï؟½1ï؟½7ï؟½0ï؟½9ï؟½1ï؟½7ï؟½1ï؟½7ï؟½1ï؟½7ï؟½1ï؟½7ï؟½0ï؟½5ï؟½1ï؟½7ï؟½1ï؟½7ï؟½1ï؟½7  X(k-|k-1)
   float X_mid = 0.0f;  //ï؟½1ï؟½7ï؟½1ï؟½7ï؟½0ï؟½2ï؟½0ï؟½2ï؟½1ï؟½7ï؟½0ï؟½9ï؟½1ï؟½7ï؟½0ï؟½0ï؟½1ï؟½7ï؟½1ï؟½7ï؟½1ï؟½7ï؟½1ï؟½7ï؟½1ï؟½7ï؟½1ï؟½7  X(k|k-1)
   float X_now = 0.0f;  //ï؟½1ï؟½7ï؟½1ï؟½7ï؟½0ï؟½2ï؟½0ï؟½2ï؟½1ï؟½7ï؟½0ï؟½9ï؟½1ï؟½7ï؟½1ï؟½7ï؟½1ï؟½7ï؟½1ï؟½7ï؟½0ï؟½5ï؟½1ï؟½7ï؟½1ï؟½7ï؟½1ï؟½7  X(k|k)
   float P_mid = 0.0f;  //ï؟½1ï؟½7ï؟½1ï؟½7ï؟½0ï؟½2ï؟½0ï؟½2ï؟½1ï؟½7ï؟½1ï؟½7ï؟½0ï؟½0ï؟½1ï؟½7ï؟½1ï؟½7ï؟½1ï؟½7ï؟½1ï؟½7ï؟½1ï؟½7ï؟½1ï؟½7ï؟½1ï؟½7ï؟½1ï؟½7ï؟½ï؟½ï؟½1ï؟½7ï؟½1ï؟½7ï؟½1ï؟½7ï؟½1ï؟½7  P(k|k-1)
   float P_now = 0.0f;  //ï؟½1ï؟½7ï؟½1ï؟½7ï؟½0ï؟½2ï؟½0ï؟½2ï؟½1ï؟½7ï؟½1ï؟½7ï؟½1ï؟½7ï؟½1ï؟½7ï؟½1ï؟½7ï؟½0ï؟½5ï؟½1ï؟½7ï؟½1ï؟½7ï؟½1ï؟½7ï؟½1ï؟½7ï؟½1ï؟½7ï؟½ï؟½ï؟½1ï؟½7ï؟½1ï؟½7ï؟½1ï؟½7ï؟½1ï؟½7  P(k|k)
   float P_last = 0.0f; //ï؟½1ï؟½7ï؟½1ï؟½7ï؟½0ï؟½5ï؟½0ï؟½2ï؟½1ï؟½7ï؟½1ï؟½7ï؟½1ï؟½7ï؟½1ï؟½7ï؟½1ï؟½7ï؟½0ï؟½5ï؟½1ï؟½7ï؟½1ï؟½7ï؟½1ï؟½7ï؟½1ï؟½7ï؟½1ï؟½7ï؟½ï؟½ï؟½1ï؟½7ï؟½1ï؟½7ï؟½1ï؟½7ï؟½1ï؟½7  P(k-1|k-1)
   float kg = 0.0f;     //kalmanï؟½1ï؟½7ï؟½1ï؟½7ï؟½1ï؟½7ï؟½1ï؟½7
   float A = 0.0f;      //ï؟½0ï؟½3ï؟½0ï؟½1ï؟½1ï؟½7ï؟½1ï؟½7ï؟½1ï؟½7ï؟½1ï؟½7
   float Q = 0.0f;
   float R = 0.0f;
   float H = 0.0f;
   */

};


//loop: getReal()->getNextForecast()
class Forecast{
public:
    //static const int Num = 2;
    Forecast();
    void Reset();
    Point2f& getNextForecast();
    Point2f& getReal(Point2f& observation);
    float getDtime(float fps);
private:
    Point2f ForecastValue;
    Point2f RealValue;
    Kalman_t Kalman_clr;
//    std::vector<std::unique_ptr<Kalman_t>> Kalmaners;
//    Kalman_t Kalmaner1,Kalmaner2;
    float T_Q,T_R;
    float dt;
};

#endif


