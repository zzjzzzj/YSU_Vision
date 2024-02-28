#ifndef FORECAST_T
#define FORECAST_T

#include "Main/headfiles.h"
#include <iostream>
#include <thread>
#include <vector>
#include <memory>




//loop: getReal()->getNextForecast()

class Kalman_t{
public:
    Kalman_t();
    void Reset();
    cv::Mat& getNextForecast();
    cv::Mat& getReal(cv::Mat& measurement);
private:
    float T_Q,T_R;
    float dt;
    cv::Mat ForecastValue;
    cv::Mat RealValue;
    std::unique_ptr<cv::KalmanFilter> Kalmaner;
};


class Forecast{
public:
    Forecast();
    void Reset();
    cv::Mat& getNextForecast();
    cv::Mat& getReal(float &x,float &y);
private:
    std::unique_ptr<Kalman_t> Kalman_h;
    cv::Mat lastReal;
    cv::Mat inputMeasure;
};


#endif
