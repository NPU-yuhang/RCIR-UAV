#include <send_socketcan.h>

int main()
{
    int s, nbytes;
    struct sockaddr_can addr;
    struct ifreq ifr;
    struct can_frame frame[2] = {{0}};
    s = socket(PF_CAN, SOCK_RAW, CAN_RAW);//创建套接字
    strcpy(ifr.ifr_name, "can0" );
    ioctl(s, SIOCGIFINDEX, &ifr); //指定can0 设备
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    bind(s, (struct sockaddr *)&addr, sizeof(addr));//将套接字与can0 绑定
    //禁用过滤规则，本进程不接收报文，只负责发送
    setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0);
    //生成两个报文
    frame[0].can_id = 0x200;
    frame[0]. can_dlc = 8;
    //objects
    frame[0].data[0] = 0x88;
    frame[0].data[1] = 0x00;
    frame[0].data[2] = 0x00;
    frame[0].data[3] = 0x00;
    frame[0].data[4] = 0x08;
    frame[0].data[5] = 0x80;
    frame[0].data[6] = 0x00;
    frame[0].data[7] = 0x00;
    //clusters
//    frame[0].data[0] = 0x88;
//    frame[0].data[1] = 0x00;
//    frame[0].data[2] = 0x00;
//    frame[0].data[3] = 0x00;
//    frame[0].data[4] = 0x10;
//    frame[0].data[5] = 0x80;
//    frame[0].data[6] = 0x00;
//    frame[0].data[7] = 0x00;
    //循环发送两个报文
    while(1)
    {
        nbytes = write(s, &frame[0], sizeof(frame[0])); //发送frame[0]
        if(nbytes != sizeof(frame[0]))
        {
            printf("Send Error frame[0]\n!");
            break; //发送错误，退出
        }
        else
        {
          printf("Send ok frame[0]\n!");
        }
        sleep(1);
    }
    close(s);
    return 0;
}
