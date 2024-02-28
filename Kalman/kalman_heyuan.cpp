#include<Kalman/kalman_heyuan.h>
#include <assert.h>

Kalman_t::Kalman_t():
Kalmaner(std::make_unique<KalmanFilter>(4,2,0))
{
    std::string xml_path = "../xml_path/kalman_hhy.xml";
    cv::FileStorage fr;
    fr.open(xml_path,cv::FileStorage::READ);
    while(!fr.isOpened()){
        std::cout << "kalman_hhy.xml loading failed..." << std::endl;
        fr=cv::FileStorage(xml_path, cv::FileStorage::READ);
        fr.open(xml_path, cv::FileStorage::READ);
    }
    fr["T_Q"] >> T_Q; 
    fr["T_R"] >> T_R;
    fr["dt"] >> dt;
    fr.release();

    Reset();   
}

void Kalman_t::Reset(){
    Kalmaner->transitionMatrix = (cv::Mat_<float>(4, 4) << 1, 0, dt, 0,
                                                            0, 1, 0, dt,
                                                            0, 0, 1, 0,
                                                            0, 0, 0, 1);
                                                            
    Kalmaner->statePre = (cv::Mat_<float>(2, 1) << 0, 0);

    setIdentity(Kalmaner->measurementMatrix);
    setIdentity(Kalmaner->errorCovPre,Scalar(1)) ;
    setIdentity(Kalmaner->processNoiseCov,Scalar::all(1e-3));
    setIdentity(Kalmaner->measurementNoiseCov,Scalar::all(1e-3));


    getNextForecast();
}

Mat& Kalman_t::getNextForecast(){
    ForecastValue = Kalmaner->predict(); 
    return ForecastValue;   
}

Mat& Kalman_t::getReal(Mat& measurement){
    RealValue = Kalmaner->correct(measurement);
    return RealValue;
}


Forecast::Forecast():Kalman_h(std::make_unique<Kalman_t>()),
                    lastReal(cv::Mat::eye(2,1,CV_32FC1)){

}
void Forecast::Reset(){
    Kalman_h->Reset();
    lastReal = cv::Mat::eye(2,1,CV_32FC1);
}

cv::Mat& Forecast::getNextForecast(){
    return Kalman_h->getNextForecast();
}

cv::Mat& Forecast::getReal(float &x,float &y){
    inputMeasure = (cv::Mat_<float>(2, 1) << x, y);
    lastReal = Kalman_h->getReal(inputMeasure);
    return lastReal;
}


