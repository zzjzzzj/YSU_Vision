#ifndef ANGLESOLVER_H
#define ANGLESOLVER_H

#include "Main/headfiles.h"
#include "GafSolver/iterative_projectile_tool.h"
#include "GafSolver/gimbal_transform_tool.h"
#include "GafSolver/gaf_projectile_solver.h"
#include "Kalman/kalman_heyuan.h"

#define GRACOMP_DIS  7500 //gravity compensation velocity threshold
#define BULLETFIRE_V 1000 //bullet firing velocity
#define CAM_SUPPORT_DIS 2450 //238mm
class AngleSolver
{
public:
    AngleSolver();
    void InitAngle();
    double * SolveAngle(std::vector<cv::Point2f>& observation_obj);
    int shoot_get();

    vector<cv::Point3f> obj;
    cv::Mat rVec;
    cv::Mat tVec;
    cv:: Mat cam;
    cv:: Mat disCoeffD;
    int shoot=0;
    int loss_num = 0;
    float gra_t;
    double _xErr;
    double _yErr;
    double distance_3d;
    double p_y_err[2];
    void P4P_solver();
    void gravity_comp();
    Point2f trajectory_simulation(Point2f aim,double v0,double angle); //弹道模拟
    double gravity_compensation(Point2f aim, double v0);//重力补偿

private:
    float y_yaw;
    float x_pitch;
    std::unique_ptr<Forecast> p_forecast_;
    Point2f middle;
    vector<Point2f> object_armor_points_;

    std::shared_ptr<rmoss_projectile_motion::GafProjectileSolver> gaf_solver; // 重力补偿
    std::shared_ptr<rmoss_projectile_motion::GimbalTransformTool> projectile_tansformoss_tool; // 重力补偿

};

#endif // ANGLESOLVER_H
