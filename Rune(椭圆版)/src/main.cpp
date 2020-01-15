
#include "workspace.h"

using namespace cv;
using std::cout;
using std::endl;
using std::to_string;

int main(){
//    Timer clock;
    Workspace workspace;
    workspace.init();

    workspace.run();

    return 0;
    VideoIO videoIO("../素材/素材2.avi");
//    VideoIO videoIO("/dev/video0");
////    VideoIO videoIO;
////    MVCamera mvCamera;
//    Energy energy;
//    Mat src_image;
////    mvCamera.open();
////    int cnt = 0;
//    while (true) {
//        if (!videoIO.read(src_image))
//            break;
////        if (!mvCamera.isOpen()){
////            cout<<"摄像头打开失败！\n";
////            break;
////        }
//
////        src_image = mvCamera.getImage();
//
//        videoIO.write(energy.getSrc());
//
//        waitKey(0);
////        waitKey(1000 / 30);
//    }
////    mvCamera.close();
//    return 0;
}

