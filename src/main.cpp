/******************************************************************************
 Copyright© HITwh HERO-Robomaster2020 Group

 Author: Wang Xiaoyan on 2019.9.22

 Detail: 主函数。因多线程和线程运行函数已在workspace中实现，故这里只需对workspace中的相应
         类进行实现，相应函数进行调用即可。
 *****************************************************************************/

#include "workspace.h"


int main(int argc, char **argv) {

    Workspace workspace;
    cv::FileStorage file_storage("../param/param.xml", FileStorage::READ);

    workspace.init(UART_NAME, file_storage);

    workspace.run();

    return 0;
}
