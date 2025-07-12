#include "rclcpp/rclcpp.hpp"
#include "auto_aim_interfaces/msg/serial_send_data.hpp" 
#include "../controler/SerialModule/Serial.h"
using namespace rm;

class SerialSendNode : public rclcpp::Node {
public:
    SerialSendNode()
        : Node("serial_send_data_node") {
        
        this->debug = declare_parameter("debug",true);
        this->close_shoot = declare_parameter("close_shoot",false);
        this->negation_send_yaw = declare_parameter("negation_send_yaw",false);
        this->negation_send_pitch = declare_parameter("negation_send_pitch",false);

        // SerialModule

		std::vector<SerialBase*> serial_datas;
		Constant* serial_data_0 = new Constant(0x55);
		Length* serial_data_length = new Length;
		Constant* serial_data_1 = new Constant(0x00);
		Constant* serial_data_2 = new Constant(0x00);
		CRC8* crc8 = new CRC8();
		Constant* serial_data_3 = new Constant(0x01);
		Constant* serial_data_4 = new Constant(0x00);
		LengthStart* serial_data_start = new LengthStart(0xAA);
		pitch = new Float;
		yaw = new Float;
		yaw_speed = new Float;
		//rpm = new Float;
		canshoot = new Int;
		move_state = new Int;
		ifget = new Int;
		enemy_kind = new Uint_8;
		enemy_x = new Int;
		enemy_y = new Int;
		LengthEnd* serial_data_end = new LengthEnd(0xA5);
		CRC16* crc16 = new CRC16;
		serial_datas = {
			serial_data_0,
			serial_data_length,
			serial_data_1,
			serial_data_2,
			crc8,
			serial_data_3,
			serial_data_4,
			serial_data_start,
			pitch,
			yaw,
			yaw_speed,
			canshoot,
			move_state,
			ifget,
			enemy_kind,
			enemy_x,
			enemy_y,
			serial_data_end,
			crc16
		};
		SerialSend__ = new SerialSend(serial_datas);

		delay_shoot_happens_before = false;

		send_data_sub_ = this->create_subscription<auto_aim_interfaces::msg::SerialSendData>(
                "serial_send_data_topic", 10,
                std::bind(&SerialSendNode::serial_send_data_callback, this, std::placeholders::_1));

		RCLCPP_INFO(this->get_logger(), "SerialSendData callback is all right!!!");

    };
	~SerialSendNode(){
		if(SerialSend__ != NULL) delete SerialSend__;
	}
private:
    void serial_send_data_callback(
        const auto_aim_interfaces::msg::SerialSendData::SharedPtr serial_send_data_msg) {
        
        float send_pitch,send_rpm,send_yaw,send_yaw_speed,shoot_time;
        int send_move_state,send_ifget,send_enemy_kind,send_enemy_x,send_enemy_y;

        if(negation_send_yaw) { 
            send_yaw = -serial_send_data_msg->yaw;
            send_yaw_speed = -serial_send_data_msg->yaw_speed;
        }
        else{
            send_yaw = serial_send_data_msg->yaw;
            send_yaw_speed = serial_send_data_msg->yaw_speed;
        }
        send_pitch = negation_send_pitch ? -serial_send_data_msg->pitch : serial_send_data_msg->pitch;
        
        send_move_state = serial_send_data_msg->move_state;
        send_ifget = serial_send_data_msg->ifget;
		send_enemy_kind = serial_send_data_msg->enemy_kind;
		send_enemy_x = serial_send_data_msg->enemy_x;
		send_enemy_y = serial_send_data_msg->enemy_y;
		shoot_time = serial_send_data_msg->shoot_time;

        this->pitch->set_data(send_pitch);
		this->yaw->set_data(send_yaw);
		this->yaw_speed->set_data(send_yaw_speed);
		this->move_state->set_data(send_move_state);
		this->ifget->set_data(send_ifget);
		this->enemy_kind->set_data(send_enemy_kind);
		this->enemy_x->set_data(send_enemy_x);
		this->enemy_y->set_data(send_enemy_y);

        if(close_shoot){
            canshoot->set_data(0);
            SerialSend__->send();
            return;
        };

		// 如果上一帧延迟发送,它的下一帧将跳过,保证发送帧不会有逻辑冲突
		if(delay_shoot_happens_before){
			delay_shoot_happens_before = false;
			return;
		};

        if (shoot_time == -1) {

			canshoot->set_data(0);
            std::thread send_thread(&SerialSend::send, SerialSend__);
			send_thread.detach();
            if(debug) RCLCPP_INFO(this->get_logger(), "can not shoot");
		}
		else if (shoot_time == 0) {
			canshoot->set_data(1);
            std::thread send_thread(&SerialSend::send, SerialSend__);
			send_thread.detach();
            if(debug) RCLCPP_INFO(this->get_logger(),  "====================canshoot====================");
		}
		else {
			delay_shoot_happens_before = true;
            canshoot->set_data(1);
			double delay = shoot_time;
			std::thread send_thread(&SerialSend::send_delay, SerialSend__, delay);
			send_thread.detach();
            if(debug) RCLCPP_INFO(this->get_logger(),  "====================canshoot====================");
		};

    };
private:
	bool delay_shoot_happens_before; // 延迟发射上一帧发生
    bool debug;
    bool close_shoot;
    bool negation_send_yaw;
    bool negation_send_pitch;

	Float* pitch;
	Float* yaw;
	Float* yaw_speed;
	Float* rpm;
	Int* canshoot;
	Int* move_state;
	Int* ifget;
	Uint_8* enemy_kind;
	Int* enemy_x;
	Int* enemy_y;


private:
	rclcpp::Subscription<auto_aim_interfaces::msg::SerialSendData>::SharedPtr send_data_sub_;
	SerialSend* SerialSend__;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SerialSendNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}