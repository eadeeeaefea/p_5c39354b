#include <string>
#include "opencv2/opencv.hpp"


#include "mvcamera.h"
#include "base.h"
#include "armordetector.h"


//#define OPEN_CAMERA//将摄像头画面接入检验的函数

using namespace std;
using namespace cv;
using namespace cv::ml;


int main() {



    FileStorage fs("../param/param.xml", FileStorage::READ);
    ArmorDetector armordetector;
    armordetector.init(fs);
    Mat current_frame_;
    //VideoWriter writer(VIDEO_SAVED_PATH, VideoWriter::fourcc('M','J','P','G'), 30, Size(1280,960));//将识别结果写入视频

    Mat standard_image[8];
    Mat origin_standard_image[8];
    vector<vector<Point>> standard_contour[8];
    int standard_image_number=0;


    //将标准图灰度化、二值化、找出数字最外围轮廓并依次储存在数组中
    for(standard_image_number=0;standard_image_number<=7;standard_image_number++){
        string standard_image_name="../models/standard_picture/"+to_string(standard_image_number+1)+".jpeg";
        standard_image[standard_image_number] = imread(standard_image_name);
        origin_standard_image[standard_image_number] = standard_image[standard_image_number].clone();
        cvtColor(standard_image[standard_image_number],standard_image[standard_image_number],CV_BGRA2GRAY);
        threshold(standard_image[standard_image_number],standard_image[standard_image_number],100,255,THRESH_BINARY);
        //imshow("标准图", standard_image[standard_image_number]);
        //waitKey(0);


        findContours(standard_image[standard_image_number], standard_contour[standard_image_number], CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);//最外层轮廓
        drawContours(origin_standard_image[standard_image_number], standard_contour[standard_image_number], -1, Scalar(0, 255, 0), 2, 8);
        string contour_name="number " + to_string(standard_image_number) + " contour";
        //imshow(contour_name, origin_standard_image[standard_image_number]);
        //waitKey(0);
    }


/*#ifdef OPEN_CAMERA
    MVCamera mv_camera;//调用摄像头
    mv_camera.open(FRAME_WIDTH, FRAME_HEIGHT, EXPOSURE_TIME);
    cout<<"Camera open successful!"<<endl;
    while(1){
        Mat image,result;
        try {
            mv_camera.getImage(image);
            //imshow("current", image);
            //waitKey(30);

        } catch (MVCameraException &e1) {
            cout << "Camera error." << endl;
            mv_camera.close();
            for (int i = 0; i < 10; ++i) {
                try {
                    mv_camera.open(FRAME_WIDTH, FRAME_HEIGHT, EXPOSURE_TIME);
                    if (mv_camera.isOpen())     break;
                } catch (MVCameraException& e2) {
                    cout << "Try to open camera error." << endl;
                }
            }
        }
#endif*/


    //查找待测试图像中的轮廓
    Mat image, gray_image_;
    int gray_thres_ = 30;
    for (int image_count = 1; image_count <= 70; image_count++) {//变量范围可随输入图片名变化
        cout<<endl;
        cout << "image number: " << image_count << endl;//输出图片编号
        string image_name = "../number_image/" + to_string(image_count) + ".jpg";//输入图片地址
        image = imread(image_name);
        resize(image,image,Size(image.cols*3,image.rows*3));//必要情况下可放大图片
        imshow("待测图", image);
        Mat showimage = image.clone();
        waitKey(0);

	//图片灰度化、二值化（待调参），进行高斯滤波和开处理（除噪音方式可改进）
        cvtColor(image, gray_image_, COLOR_BGR2GRAY);
        threshold(gray_image_, gray_image_, gray_thres_, 255, THRESH_BINARY);
        imshow("处理图1", gray_image_);
        waitKey(0);
        Mat kernel_ = getStructuringElement(MORPH_RECT, Size(5, 5));
        Size g_kernel = Size(5, 5);
        GaussianBlur(gray_image_, gray_image_, g_kernel, 0, 0, BORDER_DEFAULT);
        imshow("处理图2", gray_image_);
        waitKey(0);
        morphologyEx(gray_image_, gray_image_, MORPH_OPEN, kernel_);
        imshow("处理图3", gray_image_);
        waitKey(0);

        vector<vector<Point>> image_contours;
        findContours(gray_image_, image_contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);//查找图片中所有轮廓（并画出）
        drawContours(showimage, image_contours, -1, Scalar(255, 0, 0), 1, 8);
        imshow("image contours", showimage);
        waitKey(0);

        //矩特征提取：比较图中每个轮廓与八个模板轮廓间的相似度
        double similarity[8];
        double contour_similarity_min[30];
        double similar_number[30];
        double simiarity_min=2;
        int i_result = 0;
        for (int i = 0; i < image_contours.size(); i++)//遍历待测试图像的轮廓
        {   cout<<"matching contour: "<<i<<endl;//输出所匹配轮廓的编号
            contour_similarity_min[i]=99999;

            double image_size=gray_image_.cols*gray_image_.rows;
            double size_ratio=image_contours[i].size() /image_size;
            cout<<"contour size ratio: "<<size_ratio<<endl;

            if(size_ratio>=0.001){//通过轮廓占ROI区域面积的比大小去除干扰（待改进）（由于轮廓面积大小受距离影响很大，尝试通过面积比而不是面积筛选，待改进，待调参）
                for(int j=0;j<=7;j++){
                    similarity[j]=matchShapes(standard_contour[j][0],image_contours[i],CV_CONTOURS_MATCH_I1,0);//调用矩特征提取函数计算相似度（关键）
                    cout<<"similarity with "<<j+1<<":"<<similarity[j]<<endl;
                    //返回此轮廓与模版轮廓之间的相似度,值越小越相似

                    if(similarity[j]<contour_similarity_min[i]){
                        contour_similarity_min[i]=similarity[j];
                        similar_number[i]=j+1;
                        cout<<"contour_similarity_min:"<<contour_similarity_min[i]<<endl;
                    }//根据该轮廓与八个数字外围轮廓的最小相似度找出最相似的数字（误判几率大，待改进）
                }
                if(contour_similarity_min[i]<simiarity_min){
                    simiarity_min=contour_similarity_min[i];
                    i_result=i;//找出图片中相似度最小的轮廓，即为与模板数字最相似的数字轮廓（有一定误识别几率，需加强滤波）
                }
            }else{
                continue;
            }

        }
        drawContours(showimage, image_contours, i_result, Scalar(0, 255, 0), 2, 8);//在待测试图像上画出结果轮廓
	//输出结果轮廓的编号、面积、相近数字和相似度（debug用）
        cout<<"result contour:  "<<i_result<<endl;
        cout << "result_area:  " << image_contours[i_result].size() << endl;
        cout << "similar number:  "<< similar_number[i_result] << endl;
        cout <<" similarity:  "<< simiarity_min <<endl;

        RotatedRect R_rect = minAreaRect(image_contours[i_result]);
        Point2f vertices[4];
        R_rect.points(vertices);//外接矩形的4个顶点
        for (int i = 0; i < 4; i++)//绘制外接矩形，框出数字
            line(showimage, vertices[i], vertices[(i + 1) % 4], Scalar(0, 0, 255), 3);

        imshow("匹配结果", showimage);
        waitKey(0);

        cout<<endl;
    }


}
