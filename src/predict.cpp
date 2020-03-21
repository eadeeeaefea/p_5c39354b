/******************************************************************************
 Copyright© HITwh HERO-RoboMaster2020 Group

 Author:  Bruce Hou

 Detail:
 *****************************************************************************/
#include "predict.h"

Predict::Predict() {
//    writer.open("/home/stayalive/Documents/HERO/Hero2020_final/1.avi", 0, 25.0, Size(40, 40));
}

Predict::~Predict() {

}

void Predict::init() {

}

void Predict::run(double &x, double &y, double &z, double v, double &send_pitch, double &send_yaw, double readpitch,
                  double readyaw, double read_pitch, double read_yaw) {
    anglesolver.run(x, y, z, v, send_pitch, send_yaw, readpitch);
    sum_pitch = send_pitch + readpitch;
    sum_yaw = send_yaw + readyaw;
    time_for_excercise = anglesolver.get_flight_time(x, y, z, v, readpitch);
    cout << "time_for_excercise: " << time_for_excercise * 1000 << endl;
    object_motion.x = static_cast<float>(sum_pitch);
    object_motion.y = static_cast<float>(sum_yaw);
    object_motion.x += 250;
    object_motion.y += 250;
    update();
    motion_prediction();
    send_pitch = predict_object_motion.x - read_pitch;
    send_yaw = predict_object_motion.y - read_yaw;

#ifdef SHOW_IMAGE
    Mat src(500, 500, CV_8UC3, Scalar(0, 0, 0));
    for(int i = 0; i < object.size(); i++){
        circle(src, object[i], 2, Scalar(255, 0, 0), -1);
    }
//    circle(src, object_motion, 2, Scalar(255, 0, 0), -1);
    circle(src, predict_object_motion, 2, Scalar(0, 255, 0), -1);
    Point2f b;
    b.x = piline.x + predict_line[0] * 100;
    b.y = piline.y + predict_line[1] * 100;
    if (judgement()) {
//        line(src, piline, b, Scalar(255, 0, 0), 1);
    }
//    writer << src(Rect(Point(230, 230), Point(270, 270)));
//    stringstream str;
//    str << count << ".jpg";
//    if(count >= 96 && count <= 199){
//        imwrite("/home/stayalive/Documents/HERO/Hero2020_final/" + str.str(), src(Rect(Point(230, 230), Point(270, 270))));
//    }
//    count++;
        imshow("hello", src);
//    if(waitKey(1) == 27){
//        exit(0);
//    }
//    if(waitKey(1) < 0){
//        waitKey();
//    }
//    waitKey(30);
#endif
}

void Predict::update() {
    if (object.size() >= 5) {
        object.erase(object.begin());
    }
    object.emplace_back(object_motion);
}

void Predict::motion_prediction() {
    if (judgement()) {
        float multiplication;
        Point2f mulvector;
        Point2f prepoint;
        float predistance;
        float cosalpha, sinalpha;
        fitLine(object, predict_line, DIST_FAIR, 0, 0.01, 0.01);
        piline.x = predict_line[2];
        piline.y = predict_line[3];
        vect.x = predict_line[0];
        vect.y = predict_line[1];
        for (int i = (int) (d_object.size() - 1); i >= 0; i--) {
            if (fabs(d_object[i].x) <= 0.001 || fabs(d_object[i].y) <= 0.001) {
                continue;
            } else {
                mulvector = d_object[i];
            }
        }
        multiplication = mulvector.x * vect.x + mulvector.y * vect.y;
//        cout << "multiplication: " << multiplication << endl;
        if (multiplication < 0) {
            vect = -vect;
        }
        for (int j = (int) object.size(); j >= 0; j--) {
            if (object[j] == Point2f(0.0, 0.0)) {
                continue;
            } else {
                prepoint = object[j];
            }
        }
        predistance = static_cast<float>(sqrt(mulvector.x * mulvector.x + mulvector.y * mulvector.y) / 0.012 *
                                               time_for_excercise);
//        predict_object_motion = prepoint + vect * 5;
        predict_object_motion = prepoint + vect * predistance;
//        cout << "object_motion:(" << object_motion.x << ", " << object_motion.y <<")" << endl;
//        cout << "piline:(" << piline.x << ", " << piline.y << ")" << endl;
//        cout << "vect:" << vect.x << ", " << vect.y << ")" << endl;
//        cout << endl;
    } else {
        predict_object_motion = object_motion;
        cout << "can't predict motion" << endl;
    }
}


bool Predict::judgement() {
    int count = 0;
    Point2f d_point;
    d_object.clear();
    for (int i = 1; i < object.size(); i++) {
        d_point = object[i] - object[i - 1];
        d_object.emplace_back(d_point);
    }
    for (int i = 0; i < d_object.size(); i++) {
        if (fabs(d_object[i].x) < 0.08 && fabs(d_object[i].y) < 0.08) {
            count++;
        }
    }
    if (count >= 2) {
        return false;
    }
    return true;
}

void Predict::coordinate_transformation(double &x, double &y, double &z,
                                        double ptz_pitch, double ptz_yaw) {
    double absolute_x;
    double absolute_y;
    double absolute_z;
    absolute_x = x * cos(ptz_yaw) - y * sin(ptz_pitch) * sin(ptz_yaw) -
                 z * cos(ptz_pitch) * sin(ptz_yaw);
    absolute_y = y * cos(ptz_pitch) - z * sin(ptz_pitch);
    absolute_z = x * sin(ptz_yaw) + y * sin(ptz_pitch) * cos(ptz_yaw) +
                 z * cos(ptz_pitch) * cos(ptz_yaw);
    x = absolute_x;
    y = absolute_y;
    z = absolute_z;
}

void Predict::anti_coordinate_transformation(double &x, double &y, double &z,
                                             double ptz_pitch,
                                             double ptz_yaw) {
    double objective_x;
    double objective_y;
    double objective_z;
    objective_x = x * cos(ptz_yaw) + z * sin(ptz_yaw);
    objective_y = -x * sin(ptz_yaw) * sin(ptz_pitch) + y * cos(ptz_pitch) +
                  z * cos(ptz_yaw) * sin(ptz_pitch);
    objective_z = -x * sin(ptz_yaw) * cos(ptz_pitch) - y * sin(ptz_pitch) +
                  z * cos(ptz_yaw) * cos(ptz_pitch);
    x = objective_x;
    y = objective_y;
    z = objective_z;
}

double Predict::point_distance(Point3d point1, Point3d point2) {
    double distance;
    distance = sqrt(((point2.x - point1.x) * (point2.x - point1.x) +
                     (point2.y - point1.y) * (point2.y - point1.y) +
                     (point2.z - point1.z) * (point2.z - point1.z)));
    return distance;
}

void Predict::get_excercise_time(double x, double y, double v,
                                 double ptz_pitch) {
    double time_square;
    static const double g = 9.7988;
    double delta_angle;
    double x_bar;
    double y_bar;
    delta_angle = ptz_pitch * PI / 180;
    x_bar = x * cos(delta_angle) + y * sin(delta_angle);
    y_bar = -x * sin(delta_angle) + y * cos(delta_angle);
    time_square =
            2.0 *
            ((y_bar * g + v * v) - sqrt(pow(g * y_bar + v * v, 2.0) -
                                        (x_bar * x_bar + y_bar * y_bar) * g * g)) /
            (g * g);
    time_for_excercise = sqrt(time_square) + 0.05;
    if (isnan(time_for_excercise) || isinf(time_for_excercise)) {
        time_for_excercise = 0.05;
    }
}
