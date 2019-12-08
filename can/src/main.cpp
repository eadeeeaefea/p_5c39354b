#include <iostream>
#include <thread>

#include "cannode.h"

using namespace std;

void sendThread(void);
void receiveThread(void);

CanNode can_node;

int main(int argc, char** argv)
{
    ReadPack read_pack;
    SendPack send_pack;
    can_node.init();
    while(true)
    {
         can_node.send(send_pack);
         can_node.receive(read_pack);
         sleep(0.999);
    }

//    thread t2(receiveThread);
//    t2.join();
//    thread t1(sendThread);
//    t1.join();

    return 0;
}

void sendThread(void)
{
    SendPack send_pack;
    while(true)
    {
        can_node.send(send_pack);
        sleep(0.999);
    }
}

void receiveThread(void)
{
    ReadPack read_pack;
    while(true)
    {
        can_node.receive(read_pack);
        sleep(0.999);
    }
}
