
#include<Kalman/kalman_liren.h>

Forecast::Forecast(){
    std::string xml_path = "../xml_path/forecast_hhy.xml";
    cv::FileStorage fr;
    fr.open(xml_path,cv::FileStorage::READ);
    while(!fr.isOpened()){
        std::cout << "armor_xml loading failed..." << std::endl;
        fr=cv::FileStorage(xml_path, cv::FileStorage::READ);
        fr.open(xml_path, cv::FileStorage::READ);
    }
    fr["TQ"] >> T_Q;
    fr["TR"] >> T_R;
    Kalman_clr.KalmanInit(this->dt, this->T_R, this->T_Q);
}

void Forecast::Reset(){
    /*
   for (auto &kalman:Kalmaners){
//      kalman->KalmanInit(T_Q,T_R);
       kalman.reset(new Kalman_t(this->dt, T_Q, T_R));
      kalman->KalmanForecast();
   }
   */
    Kalman_clr.KalmanInit(this->dt, this->T_R, this->T_Q);
}

Point2f& Forecast::getNextForecast(){
    vec4f predVec = Kalman_clr.KalmanForecast();

    ForecastValue = Point2f(predVec[0], predVec[1]);
    return ForecastValue;
}

Point2f& Forecast::getReal(Point2f& observation){
    vec2f Z_now;
    Z_now << observation.x, observation.y;

    vec4f corrVec;
    corrVec = Kalman_clr.KalmanFilter(Z_now);
    RealValue = Point2f(corrVec[0], corrVec[1]);
    return RealValue;
}

float Forecast::getDtime(float fps){
    return 1000/fps;
}

Kalman_t::Kalman_t()
{};

void Kalman_t::KalmanInit(float dt, float T_R, float T_Q){
    Diag4f << 1, 0, 0, 0,
              0, 1, 0, 0,
              0, 0, 1, 0,
              0, 0, 0, 1;

    Diag2f << 1, 0,
              0, 1;

    X_last << 0,
              0,
              0,
              0;

    X_mid  << 0,
              0,
              0,
              0;

    X_now  << 0,
              0,
              0,
              0;

    P_last << 0, 0, 0, 0,
              0, 0, 0, 0,
              0, 0, 0, 0,
              0, 0, 0, 0;

    P_mid  << 0, 0, 0, 0,
              0, 0, 0, 0,
              0, 0, 0, 0,
              0, 0, 0, 0;

    P_now  << 0, 0, 0, 0,
              0, 0, 0, 0,
              0, 0, 0, 0,
              0, 0, 0, 0;

    A      << 1, 0, dt, 0,
              0, 1, 0, dt,
              0, 0, 1, 0,
              0, 0, 0, 1;

    H      << 1, 0, 0, 0,
              0, 1, 0, 0;

    Q      << T_Q, 0, 0, 0,
              0, T_Q, 0, 0,
              0, 0, T_Q, 0,
              0, 0, 0, T_Q;

    R      << T_R, 0,
              0, T_R;
}

vec4f Kalman_t::KalmanForecast(){
    X_mid = A * X_last;                     //�ٶȶ�Ӧ��ʽ(1)    x(k|k-1) = A*X(k-1|k-1)+B*U(k)+W(K)
    P_mid = A*P_last*A.transpose() + Q;                     //�ٶȶ�Ӧ��ʽ(2)    p(k|k-1) = A*p(k-1|k-1)*A'+Q
    return X_mid;
}

vec4f Kalman_t::KalmanFilter(vec2f dat)
{
   X_now = X_mid + Kg * (dat - H*X_mid);     //�ٶȶ�Ӧ��ʽ(3)    x(k|k) = X(k|k-1)+kg(k)*(Z(k)-H*X(k|k-1))
   //P_now = (1-kg)*P_mid;                //�ٶȶ�Ӧ��ʽ(5)    p(k|k) = (I-kg(k)*H)*P(k|k-1)
   P_now = (Diag4f - Kg * H) * P_mid *(Diag4f - Kg * H).transpose() + Kg * R * Kg.transpose();
   Kg = P_mid * H.transpose() * (H * P_mid * H.transpose() + R).inverse();             //�ٶȶ�Ӧ��ʽ(4)    kg(k) = p(k|k-1)*H'/(H*p(k|k-1)*H'+R)


   P_last = P_now;                         //״̬����
   X_last = X_now;
   return X_now;							  //���Ԥ����x(k|k)
}



