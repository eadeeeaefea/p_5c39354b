#include "cannode.h"

CanNode::CanNode() {

}

CanNode::~CanNode() {
    if (skt_) {
        close(skt_);
    }
}

bool CanNode::init() {
    bool status = true;

#ifdef USE_CAN
    #if USE_CAN == 0
    dev_name = "can0";
#elif USE_CAN == 1
    dev_name = "can1";
#endif
    system(DOWN);
    system(COMMAND);
    system(UP);
#endif // USE_CAN

    if ((skt_ = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
        std::cerr << dev_name << " socket error.\n";
        status = false;
    }
    strcpy(ifr_.ifr_name, dev_name.c_str());

    if (ioctl(skt_, SIOCGIFINDEX, &ifr_) < 0) {
        std::cerr << dev_name << " ioctl error.\n";
        status = false;
    }

    addr_.can_family = AF_CAN;
    addr_.can_ifindex = ifr_.ifr_ifindex;

    if (bind(skt_, (struct sockaddr *) &addr_, sizeof(addr_)) != 0)  // bind socket and device can
    {
        std::cout << dev_name << "bind error.\n";
        status = false;
    }

    for (int i = 0; i < ID_NUM; ++i) {
        cfilter_[i].can_id = id_rcv_[i];    // receive frame id
        cfilter_[i].can_mask = CAN_SFF_MASK; // succeed when (reveived_id) & mask == (can_id) & mask
    }
    if (setsockopt(skt_, SOL_CAN_RAW, CAN_RAW_FILTER, &cfilter_, sizeof(cfilter_)) != 0) {

        std::cout << dev_name << " sockopt error.\n";
        status = false;
    }
//    mode_ = 2;

    return status;
}

bool CanNode::send(const SendPack &send_pack) {
//    if (mode_ == 2) {
//        std::cout << "Sending pauses.\n";
//        return true;
//    }

    bool res = false;
    Frame frame{0};

    frame.can_dlc = 8;
    frame.can_id = id_snd_[0];

    frame.data[0] = static_cast<uint8_t>(send_pack.mode);
    int16_t* data_ptr = reinterpret_cast<int16_t*>(frame.data + 1);
    data_ptr[0] = static_cast<int16_t>(send_pack.yaw * 100);
    data_ptr[1] = static_cast<int16_t>(send_pack.pitch * 100);
//    frame.data[1] = static_cast<int16_t>(send_pack.yaw * 100) >> 8;
//    frame.data[2] = static_cast<int16_t>(send_pack.yaw * 100);
//    frame.data[3] = static_cast<int16_t>(send_pack.pitch * 100) >> 8;
//    frame.data[4] = static_cast<int16_t>(send_pack.pitch * 100);
    int nbytes = write(skt_, &frame, sizeof(Frame));
    if (nbytes < 0) {
        std::cerr << "can raw socket write.\n";
        res = false;
    } else if (nbytes < sizeof(Frame)) {
        std::cerr << "write: incomplete frame.\n";
        res = false;
    } else if (nbytes == sizeof(Frame)) {
        // std::cout << "Sending pitch: " << temp_ptr[0] << " , yaw: " << temp_ptr[1] << std::endl;
        // printf("sending: ");
        // for(int i = 0; i < 4; ++i)
        // printf("%x\t", frame.data[i]);
        // printf("\n");
        res = true;
        std::cout << "successfully" << std::endl;
    }

    return res;
}

bool CanNode::receive(ReadPack &read_pack) {
    bool res = false;

    Frame frame;

    int nbytes = read(skt_, &frame, sizeof(Frame));

    if (nbytes < 0) {
        std::cerr << "can raw socket read.\n";
        res = false;
    } else if (nbytes < sizeof(Frame)) {
        std::cerr << "read: incomplete frame.\n";
        res = false;
    } else if (nbytes == sizeof(Frame)) {
        unpack(frame, read_pack);
        res = true;
    }

    return res;
}

bool CanNode::unpack(const Frame &frame, ReadPack &read_pack) {

    read_pack.enemy_color = frame.data[0] > 10;
    mode_ = read_pack.mode = static_cast<int>(frame.data[1]);

    int16_t temp_pitch = (static_cast<int16_t>(frame.data[2]) << 8) + static_cast<int16_t>(frame.data[3]);
    read_pack.pitch = static_cast<double>(temp_pitch * 0.01);
    int16_t temp_yaw = (static_cast<int16_t>(frame.data[4]) << 8) + static_cast<int16_t>(frame.data[5]);
    read_pack.yaw = static_cast<double>(temp_yaw * 0.01);

    // printf("receiving: ");
    // for(int i = 0; i < 8; ++i)
    // printf("%x\t", frame.data[i]);
    // printf("\n");
    // std::cout << "Unpacked: " << read_pack.pitch << "," << read_pack.yaw << std::endl;

    return true;
}
