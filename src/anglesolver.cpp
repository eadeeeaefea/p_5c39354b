/******************************************************************************
 Copyright© HITwh HERO-RoboMaster2020 Group

 Author: Wang Xiaoyan on 2019.9.20

  Update: Zeng Jing on 2019.12.17.

 Detail:
 *****************************************************************************/

#include "anglesolver.h"
//#define DEBUG
using namespace std;


AngleSolver::AngleSolver() {

}

AngleSolver::~AngleSolver() {

}

void AngleSolver::init() {
    yaw_offset_ = 0.0;
    pitch_offset_ = 0.0;
    g = 9.7988;//经测试得到的参数
}
void AngleSolver::run(double x, double y, double z, double v,
                      double &yaw, double &pitch, double ptz_pitch) {
#ifdef RUNNING_TIME
    Timer timer;
    timer.start();
#endif

    if (parabolaSolve(sqrt(x * x + z * z), y, v, pitch, ptz_pitch)) {
        if (z > 2.5) {
            pitch = -pitch - 0.65;
            yaw = atan(x / z) / PI * 180 - 0.6;
        } else {
            pitch = -pitch;
            yaw = atan(x / z) / PI * 180;
        }
    } else {
        pitch = 0.0;
        yaw = 0.0;
    }

#ifdef RUNNING_TIME
    cout << "solve angle time: " << timer.getTime() << "ms" << endl;
    timer.stop();
#endif
}

bool AngleSolver::parabolaSolve(double x, double y, double v, double &theta, double ptz_pitch) {
    /**
    double time;
    double res, res_1;
    static const double g = 9.7988;
    time = 2.0 * ((y * g + v * v) - sqrt(pow(g * y + v * v, 2.0)- (x * x + y * y) * g * g)) / (g * g);
    time = sqrt(time);
    res_1 = (y - 0.5 * g * time * time) / v / time;
    res = asin(res_1);
    theta = res * 180 / PI;
    if(isnan(theta) || isinf(theta)){
     theta = 0;
     return false;
    }
    return true;
    //cout << alpha << endl;
    //cout << "(" << x << "," << y << ")" << endl;
    **/
/**
    double time;
    static const double g = 9.7988;
    double res, res_1;
    double delta_angle;
    double x_bar;
    double y_bar;
    delta_angle = ptz_pitch * PI / 180;
    x_bar = x * cos(delta_angle) - y * sin(delta_angle);
    y_bar = x * sin(delta_angle) + y * cos(delta_angle);
    time = 2.0 * ((y_bar * g + v * v) - sqrt(pow(g * y_bar + v * v, 2.0) - (x_bar * x_bar + y_bar * y_bar) * g * g)) /
           (g * g);
    time = sqrt(time);
    res_1 = (y_bar - 0.5 * g * time * time) / v / time;
    res = asin(res_1);
    theta = res * 180 / PI;
    theta = theta - ptz_pitch;
    if (isnan(theta) || isinf(theta)) {
        theta = 0;
        return false;
    }
    return true;
**/
    double time;
    static const double g = 9.7988;
    double res, res_1;
    double delta_angle;
    double x_bar;
    double y_bar;
    delta_angle = ptz_pitch * PI / 180;
    x_bar = x * cos(delta_angle) + y * sin(delta_angle);
    y_bar = -x * sin(delta_angle) + y * cos(delta_angle);
    time = 2.0 * ((y_bar * g + v * v) - sqrt(pow(g * y_bar + v * v, 2.0) - (x_bar * x_bar + y_bar * y_bar) * g * g)) /
           (g * g);
    time = sqrt(time);
    res_1 = (y_bar - 0.5 * g * time * time) / v / time;
    res = asin(res_1);
    theta = res * 180 / PI;
    theta = theta + ptz_pitch;
    if (isnan(theta) || isinf(theta)) {
        theta = 0;
        return false;
    }
    return true;

}

double AngleSolver::get_yaw_offset() {
    return yaw_offset_;
}

double AngleSolver::get_pitch_offset() {
    return pitch_offset_;
}

void AngleSolver::set_yaw_offset(double yaw_offset) {
    yaw_offset_ = yaw_offset;

}

void AngleSolver::set_pitch_offset(double pitch_offset) {
    pitch_offset_ = pitch_offset;
}

//runesolver
double AngleSolver::getOriginPitch() {
    return origin_pitch;
}

double AngleSolver::getOriginYaw() {
    return origin_yaw;
}

void AngleSolver::setOriginPitch(double pitch) {
    origin_pitch = pitch;
}

void AngleSolver::setOriginYaw(double yaw) {
    origin_yaw = yaw;
}

//其他备用算法
void AngleSolver::change_dichotomy(double x, double y, double z, double v,
                                   double& yaw, double& pitch, double ptz_pitch){
    Timer timer;
    timer.start();

    //之前由图像坐标系转换得到的相机坐标系的y轴竖直向下，现在转换回来
    y = -y;

    //将云台坐标旋转回标准坐标系(ｙ轴竖直向下）
    double last_y,last_z,ptz_degree;
    last_y=y;//存储原来的y和z值
    last_z=z;
    ptz_degree=(ptz_pitch/180)*PI;//将云台绝对pitch轴转换成角度值
    y=last_y*cos(ptz_degree)+last_z*sin(ptz_degree);
    z=last_z*cos(ptz_degree)-last_y*sin(ptz_degree);
    //将原来的坐标转换成标准坐标系,坐标系向下旋转一定角度等价于点向上旋转相同角度

    yaw = (atan(x / z) / PI) * 180 + yaw_offset_;//计算yaw值

    //算出二分法的角度范围
    double pitch_min, pitch_max;
    pitch_min = atan2(y, sqrt(x * x + z * z))-0.05;
    pitch_max = pitch_min+0.6;
    //计算上限和下限角度对应落点与目标纵坐标差值（正常应异号）
    double t_min, t_max, delta_y_min, delta_y_max, differ_min, differ_max;
    //最小角度对应差值计算
    t_min = sqrt(x * x + z * z) / (v * cos(pitch_min));
    delta_y_min = v * sin(pitch_min) * t_min - 0.5 * g * t_min * t_min;
    differ_min = delta_y_min - y;
    //最大角度对应差值计算
    t_max = sqrt(x * x + z * z) / (v * cos(pitch_max));
    delta_y_max = v * sin(pitch_max) * t_max - 0.5 * g * t_max*t_max;
    differ_max = delta_y_max - y;
#ifdef DEBUG
    cout<<"differ_min ="<<differ_min<<endl;
    cout<<"differ_max ="<<differ_max<<endl;
#endif

    //提前排除不需二分的情况:
    if (fabs(differ_min) <= 0.001) {
        //最小角度符合pitch角要求：落点与目标纵坐标差值小于一定范围
        pitch = pitch_min;
    }else if (fabs(differ_max) <= 0.001) {//最大角度符合pitch角要求
        pitch = pitch_max;
    }else if (differ_min * differ_max > 0){ //两角度对应目标差值同号，出错
        return;
    }

    double mid;//存储二分法角度中间值的变量
    //进行二分计算，角度对应落点与目标纵坐标差值小于一定范围
    // 或当循环计算次数达到上限时输出pitch值
    if (differ_min * differ_max < 0) {
        int i = 0;
        double delta_y_mid = 0, t_mid= 0, differ_mid = 0;
        for (i = 0; i < 20; i++){
            //二分，计算mid角对应的落点与目标纵坐标差值
            mid = (pitch_max + pitch_min) / 2;
            t_mid = sqrt(x * x + z * z) / (v * cos(mid));
            delta_y_mid = v * sin(mid) * t_mid - 0.5 * g * t_mid * t_mid;
            differ_mid = delta_y_mid - y;
            //判断mid角度是否符合pitch角要求：落点与目标纵坐标差值小于一定范围
            //或循环次数达到上限，若符合输出mid值作为pitch值
            if (fabs(differ_mid) <= 0.001 && (i == 19)) {
                pitch = mid;
                break;
            } else if (differ_mid * differ_min > 0) {
                //若不符合要求，缩小角度范围继续二分
                pitch_min = mid;
            } else {
                pitch_max = mid;
            }
        }
    }
    //计算得pitch轴角度换算成角度，减去云台绝对角度才得到相对现有云台坐标系需调整角度，再加上偏移
    pitch=(pitch/PI)*180+pitch_offset_-ptz_pitch;
    //输出运算结果和运行时间
    cout << "change_dichotomy:  yaw=" << yaw << "  pitch=" << pitch << endl;
    cout << "change_dichotomy: " << timer.getTime() << "ms" << endl;
    timer.stop();
}

// 未加坐标旋转变换，稍微慢一些的两个算法，速度矢量三角计算快于二分法。
//二分法算法原理同上
void AngleSolver::dichotomy(double x, double y, double z, double v,
                            double& yaw, double& pitch) {
    Timer timer;
    timer.start();
    //之前由图像坐标系转换得到的相机坐标系的y轴竖直向下，现在转换回来
    y = -y;

    //计算yaw值
    yaw = (atan(x / z) / PI) * 180 + yaw_offset_;

    //算出二分法的角度范围
    double pitch_min, pitch_max;
    pitch_min = atan2(y, sqrt(x * x + z * z))-0.05;
    pitch_max = pitch_min+0.6;
    //计算上限和下限角度对应落点与目标纵坐标差值（正常应异号）
    double t_min, t_max, delta_y_min, delta_y_max, differ_min, differ_max;
    //最小角度对应差值计算
    t_min = sqrt(x * x + z * z) / (v * cos(pitch_min));
    delta_y_min = v * sin(pitch_min) * t_min - 0.5 * g * t_min * t_min;
    differ_min = delta_y_min - y;
    //最大角度对应差值计算
    t_max = sqrt(x * x + z * z) / (v * cos(pitch_max));
    delta_y_max = v * sin(pitch_max) * t_max - 0.5 * g * t_max*t_max;
    differ_max = delta_y_max - y;
#ifdef DEBUG
    cout<<"differ_min ="<<differ_min<<endl;
    cout<<"differ_max ="<<differ_max<<endl;
#endif

    //提前排除不需二分的情况:
    if (fabs(differ_min) <= 0.001) {
        //最小角度符合pitch角要求：落点与目标纵坐标差值小于一定范围
        pitch = pitch_min;
    }else if (fabs(differ_max) <= 0.001) {//最大角度符合pitch角要求
        pitch = pitch_max;
    }else if (differ_min * differ_max > 0){ //两角度对应目标差值同号，出错
        return;
    }

    double mid;//存储二分法角度中间值的变量
    //进行二分计算，角度对应落点与目标纵坐标差值小于一定范围
    // 或当循环计算次数达到上限时输出pitch值
    if (differ_min * differ_max < 0) {
        int i = 0;
        double delta_y_mid = 0, t_mid= 0, differ_mid = 0;
        for (i = 0; i < 20; i++){
            //二分，计算mid角对应的落点与目标纵坐标差值
            mid = (pitch_max + pitch_min) / 2;
            t_mid = sqrt(x * x + z * z) / (v * cos(mid));
            delta_y_mid = v * sin(mid) * t_mid - 0.5 * g * t_mid * t_mid;
            differ_mid = delta_y_mid - y;
            //判断mid角度是否符合pitch角要求：落点与目标纵坐标差值小于一定范围
            //或循环次数达到上限，若符合输出mid值作为pitch值
            if (fabs(differ_mid) <= 0.001 && (i == 19)) {
                pitch = mid;
                break;
            } else if (differ_mid * differ_min > 0) {
                //若不符合要求，缩小角度范围继续二分
                pitch_min = mid;
            } else {
                pitch_max = mid;
            }
        }
    }
    pitch = (pitch / PI) * 180 + pitch_offset_;//换算成角度

    cout << "Dichotomy:  yaw=" << yaw << "  pitch=" << pitch << endl;
    //输出结果和运行时间
    cout << "Dichotomy time: " << timer.getTime() << "ms" << endl;
    timer.stop();
}


void AngleSolver::speedtriangle(double x, double y, double z, double v,
                                double& yaw, double& pitch) {
    Timer timer;
    timer.start();
    //由于算法需要，此处不需要将y轴反向
    //计算yaw值
    yaw = (atan(x / z) / PI )* 180 + yaw_offset_;

    //由速度矢量三角形推导出的公式实现
    double speed_angle,sin_speed_angle,pitch_tan_y,pitch_tan_x,mid_value;
    sin_speed_angle=g*sqrt(x*x+z*z)/(v*sqrt(v*v+2*g*y));
    speed_angle=asin(sin_speed_angle);
    mid_value=v-cos(speed_angle)*sqrt(v*v+2*g*y);
    pitch_tan_y=v*fabs(mid_value);
    pitch_tan_x=g*sqrt(x*x+z*z);
    pitch=atan2(pitch_tan_y,pitch_tan_x);
    //由推导公式，根据中间值判断pitch角正负
    if(mid_value<0){pitch=-pitch;}

    pitch = (pitch/PI)*180 +pitch_offset_;//换算成角度

    cout<<"SpeedTriangle:  yaw="<<yaw<<"  pitch="<<pitch<<endl;
    cout << "SpeedTriangle time: " << timer.getTime() << "ms" << endl;
    timer.stop();
}
