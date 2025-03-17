#include "rclcpp/rclcpp.hpp"
#include "auto_aim_interfaces/msg/serial_read_data.hpp" // 包含自定义消息头文件
#include "../controler/SerialModule/Serial.h"
#include <string>
#include <math.h>
#include <iostream>
#include <cmath>
#include <chrono>
#include <boost/asio.hpp>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <errno.h>
#include <stdlib.h>
#include <sys/time.h>
#include <time.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <mutex>
#include <queue>
using namespace rm;

class SerialReadNode : public rclcpp::Node {
public:
    SerialReadNode()
        : Node("serial_read_data_node") {
        
        this->debug = declare_parameter("debug",true);
        this->negation_read_yaw = declare_parameter("negation_read_yaw",false);
        this->negation_read_pitch = declare_parameter("negation_read_pitch",false);

        std::vector<SerialBase*> serial_datas;
        Constant* data_head = new Constant(170);
        enemy_color = new Uint_8;
        car_yaw = new Float;
        car_pitch = new Float;
        grade = new Uint_8;
        // Constant* data_tail = new Constant(165);
        CRC16* crc16 = new CRC16;

        serial_datas = {
            data_head,
            enemy_color,
            car_yaw,
            car_pitch,
            grade,
            crc16
        };

        SerialRead__ = std::make_unique<SerialRead>(serial_datas);
        publisher_ = this->create_publisher<auto_aim_interfaces::msg::SerialReadData>("serial_read_data_topic", 10);

        RCLCPP_INFO(this->get_logger(), "SerialReadNode is all right!!!");
    };
public:
    void read_data(){
        // 启动串口
        open_serial();
        // 主程序
        while(rclcpp::ok())
        {
            // 读取数据
            int bytes_read = read(fd, buffer, sizeof(buffer));
            std::vector<uint8_t> data(bytes_read);
            for(int i = 0;i < bytes_read; i++){
                int data_int = int(buffer[i]);
                if (data_int < 0)data_int = 256 + data_int;
                data[i] = static_cast<uint8_t>(data_int);
            };
            // 检查并推送
            this->check_publish(data);
        };
    };
private:
    bool debug;
    bool negation_read_yaw;
    bool negation_read_pitch;

    Uint_8* enemy_color;
    Float* car_yaw;
    Float* car_pitch;
    Uint_8* grade;
private: // 缓冲区信息和地址
    char buffer[1024];
    int fd;
private:
    void open_serial()
    {
        int fd1 = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY);
        int fd2 = open("/dev/ttyUSB1", O_RDWR | O_NOCTTY);
        if (fd1 == -1 && fd2 == -1) {
            RCLCPP_ERROR(this->get_logger(),"Can't open serial");
            exit(0);
        }
        else{
            fd = fd1 == -1 ? fd2 : fd1;
            RCLCPP_INFO(this->get_logger(),"Open serial!!!");
        };
        struct termios config;
        tcgetattr(fd, &config);
        config.c_cflag = B115200 | CS8 | CLOCAL | CREAD;
        config.c_iflag = IGNPAR;
        config.c_oflag = 0;
        config.c_lflag = 0;
        tcflush(fd, TCIFLUSH);
        tcsetattr(fd, TCSANOW, &config);
    };

    bool check_publish(const std::vector<uint8_t>& data) {

        if(SerialRead__->check(data)){
            // 发送
            auto message = auto_aim_interfaces::msg::SerialReadData();
            // 填充消息数据
            message.enemy_color = enemy_color->back_data();
            message.car_yaw = this->negation_read_yaw ? -car_yaw->back_data() : car_yaw->back_data();
            message.car_pitch = this->negation_read_pitch ? -car_pitch->back_data() : car_pitch->back_data();
            message.grade = grade->back_data();
            publisher_->publish(message);

            if(debug){
                // 发布消息
                RCLCPP_INFO(this->get_logger(), "Publishing: enemy_color='%d',car_yaw='%f',car_pitch='%f',car_yaw_speed='%f',grade='%d'", 
                    message.enemy_color,
                    message.car_yaw * 180 / 3.1415926,
                    message.car_pitch * 180 / 3.1415926,
                    message.car_yaw_speed,
                    message.grade
                ); // 这里仅作为示例打印第一个元素
            };
        }
        else{
            if(debug){
                RCLCPP_ERROR(this->get_logger(),"data is fail!!!");
                RCLCPP_INFO(this->get_logger(),"data size: %d",data.size());
                for(int i = 0,I = data.size();i < I;i++){
                    RCLCPP_INFO(this->get_logger(),"%d: %d",i, (int)data[i]);
                };
            };
        }
    }
    rclcpp::Publisher<auto_aim_interfaces::msg::SerialReadData>::SharedPtr publisher_;
    std::unique_ptr<SerialRead> SerialRead__;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SerialReadNode>();
    node->read_data();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}