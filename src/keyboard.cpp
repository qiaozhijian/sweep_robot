//
// Created by qzj on 2020/6/24.
//
#include <termio.h>
#include <stdio.h>
#include <unistd.h>
#include "global.h"

// esc 27
int scanKeyboard() {
    int in;
    struct termios new_settings;
    struct termios stored_settings;
    tcgetattr(STDIN_FILENO, &stored_settings); //获得stdin 输入
    new_settings = stored_settings;           //
    new_settings.c_lflag &= (~ICANON);        //
    new_settings.c_cc[VTIME] = 0;
    tcgetattr(STDIN_FILENO, &stored_settings); //获得stdin 输入
    new_settings.c_cc[VMIN] = 1;
    tcsetattr(STDIN_FILENO, TCSANOW, &new_settings); //
    in = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &stored_settings);
    return in;
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "keyboard_node");
    ros::NodeHandle n;
    ros::Publisher keyboardAd = n.advertise<std_msgs::Int32>("keyboard", 10);
    ros::Rate loop_rate(200);
    ROS_INFO("mykeyboardnode init.");
    while (ros::ok()) {
        std_msgs::Int32 msg;
        //没有按键会卡死
        int key = scanKeyboard();
        msg.data = key;
        keyboardAd.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
}
