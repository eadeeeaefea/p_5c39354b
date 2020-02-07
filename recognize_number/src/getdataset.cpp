#include <iostream>
#include <string>
#include <fstream>
#include "armordetector.h"

#define ENERMY_COLOUR 0//0-red  1-blue

using namespace std;
using namespace cv;

#if 1//1-取机器人图片中数字ROI原图 0-取数字模板标准图（在官方数字图片的基础上加以处理）
int main() {
    //VideoCapture cap("/home/zj/HERO/task/week9/armature-new/dataset_red_light/red_light.avi");
    //if (!cap.isOpened()) {
        //cout << "Cannot open the video" << endl;
        //return -1;
    //}//读取视频中图片

    FileStorage fs("../param/param.xml", FileStorage::READ);
    ArmorDetector armordetector;
    armordetector.init(fs);

    string img_name, wt_name,data;
    int image_count ,result_count=0;
    Mat src,result;
    RotatedRect target_armor_;
    vector<Point2f> points;
    Rect armor_rect;
    int armor_rect_x, armor_rect_y, armor_rect_w, armor_rect_h;

  for(image_count=1;image_count<=9;image_count++){//计数变量范围要根据输入图片名修改
      //bool bSuccess = cap.read(src);
      //if (!bSuccess) {
          //cout << "Cannot read a frame from video stream" << endl;
          //break;
      //}//读入视频中图片
    wt_name = "../image/0" + to_string(image_count) + ".jpg";//读取图片的地址
    src=imread(wt_name);
    result = src.clone();
    imshow("src", src);
    waitKey(0);//读取并查看原图

    armordetector.run(result, ENERMY_COLOUR, target_armor_);
    armor_rect = target_armor_.boundingRect();//取装甲板灯条ROI矩形
    armor_rect = cvRect(armor_rect.tl().x+0.2*armor_rect.width,armor_rect.tl().y-0.5*armor_rect.height,0.6*armor_rect.width,armor_rect.height*2);
    rectangle(result, armor_rect, Scalar(255,0,0));//取数字ROI矩形（比装甲板灯条ROI矩形略宽）
    imshow("armor_rect", result);//检查ROI矩形
    waitKey(0);

    //防止ROI超出原图范围
    armor_rect_x = armor_rect.tl().x;
    armor_rect_y = armor_rect.tl().y;
    armor_rect_w = armor_rect.width;
    armor_rect_h = armor_rect.height;
    if (armor_rect_x < 0)  armor_rect_x = 0;
    if (armor_rect_y < 0)  armor_rect_y = 0;
    if (armor_rect_x + armor_rect_w > src.size().width)  armor_rect_w = armor_rect_w - (armor_rect_x + armor_rect_w - src.size().width);
    if (armor_rect_y + armor_rect_h > src.size().height) armor_rect_h = armor_rect_h - (armor_rect_y + armor_rect_h - src.size().height);
    armor_rect = Rect(armor_rect_x, armor_rect_y, armor_rect_w, armor_rect_h);
    //cout<<"armor rect:"<<armor_rect.area()<<endl;

      Mat num_roi = src(armor_rect);
      if(armor_rect.area()>=8500){//通过面积筛选一定程度上排除误识别
          result_count++;
          wt_name= "../number_image/" + to_string(result_count) + ".jpg";//存储图片路径
          imshow("result", num_roi);//检查图片，无误后写入
          waitKey(0);
          imwrite(wt_name, num_roi);
      }


    }

}
#else
int main() {
    FileStorage fs("../param/param.xml", FileStorage::READ);
    ArmorDetector armordetector;
    armordetector.init(fs);

    Mat src,result;
    Mat gray_image_,processed_image;
    Mat kernel_ = getStructuringElement(MORPH_RECT, Size(3,3));

    vector<Point2f> points;
    Rect number_rect;
    int number_rect_x, number_rect_y, number_rect_w, number_rect_h;

    src=imread("../models/origin_picture/8.jpeg");//读取官方数字原图
    imshow("原图",src);
    waitKey(0);

    //灰度化、二值化处理图片
    cvtColor(src, gray_image_, COLOR_BGR2GRAY);
    threshold(gray_image_, processed_image, 100, 255, THRESH_BINARY);
    morphologyEx(processed_image, processed_image, MORPH_OPEN, kernel_);
    imshow("处理图",processed_image);
    waitKey(0);

    vector<vector<Point>> contours;
    findContours(processed_image,contours,CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
    //for(int i = 0;i<contours.size();i++){
        //cout<<"contour:"<<i<<endl;
        //drawContours(src, contours, i, Scalar(0, 255, 0), 2, 8);
        //imshow("result", src);
        //waitKey(0);
    //}//遍历图中所有轮廓，找出数字的最外围轮廓的代码
    number_rect = boundingRect(contours[4]);//找出最外围轮廓后绘制包围矩形作为ROI矩形（轮廓编号需手动输入）

    //防止ROI区域超出原图范围
    number_rect_x = number_rect.tl().x;
    number_rect_y = number_rect.tl().y;
    number_rect_w = number_rect.width;
    number_rect_h = number_rect.height;
    if (number_rect_x < 0)  number_rect_x = 0;
    if (number_rect_y < 0)  number_rect_y = 0;
    if (number_rect_x + number_rect_w > processed_image.size().width)  number_rect_w = number_rect_w - (number_rect_x + number_rect_w - processed_image.size().width);
    if (number_rect_y + number_rect_h > processed_image.size().height) number_rect_h = number_rect_h - (number_rect_y + number_rect_h - processed_image.size().height);
    number_rect = Rect(number_rect_x, number_rect_y, number_rect_w, number_rect_h);

    processed_image(number_rect).copyTo(result);
    imshow("标准图",result);//对处理后图片取ROI得数字模板图，检查无误后储存
    waitKey(0);
    imwrite("8.jpeg", result);
}
#endif
