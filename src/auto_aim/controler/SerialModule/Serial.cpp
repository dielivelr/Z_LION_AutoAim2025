#include "Serial.h"

namespace rm
{
	SerialSend::SerialSend(std::vector<SerialBase*> datas)
		:datas(datas)
	{
	};
	SerialSend::~SerialSend()
	{
		for (int i = 0, I = datas.size(); i < I; i++) {
			if (datas[i] != NULL) delete datas[i];
		};
	};
	std::vector<uint8_t> SerialSend::send()
	{
		std::vector<uint8_t> msg;
		int size = datas.size();
		int length_id = -1; // 长度位
		int start_length_id = -1; // 起始位
		int end_length_id = -1; // 结束位
		std::vector<std::pair<int, int>> crcs; // crc的datas下标和msg下标
		for (int i = 0; i < size; i++) {
			switch (datas[i]->operator_)
			{
			case 0:
			{
				add_in_msg(msg, datas[i]);
			}
			break;
			case 1:
			{
				length_id = msg.size();
				add_in_msg(msg, datas[i]);
			}
			break;
			case 2:
			{
				start_length_id = msg.size();
				add_in_msg(msg, datas[i]);
			}
			break;
			case 3:
			{
				end_length_id = msg.size();
				add_in_msg(msg, datas[i]);
			}
			break;
			case 4:
			{
				crcs.push_back(std::pair<int, int>(i, msg.size()));
				add_in_msg(msg, datas[i]);
			}
			break;
			case 5:
			{
				crcs.push_back(std::pair<int, int>(i, msg.size()));
				add_in_msg(msg, datas[i]);
			}
			break;
			default:
				break;
			};
		};

		// 如果长度位非-1,则计算长度
		if (length_id != -1 && start_length_id != -1 && end_length_id != -1)
		{
			msg[length_id] = end_length_id - start_length_id + 1;
		};

		// 最后处理CRC校验
		for (const auto& crc : crcs) {
			if (datas[crc.first]->operator_ == 4) { // CRC8
				msg[crc.second] = getCRC8(msg.data(), crc.second);
			}
			else if (datas[crc.first]->operator_ == 5) // CRC16
			{
				uint16_t data = getCRC16(msg.data(), crc.second);
				msg[crc.second] = (uint8_t)(data & 0x00ff);
				msg[crc.second + 1] = (uint8_t)((data >> 8) & 0x00ff);
			};
		};

		serial_send(msg.data(), msg.size());
		return msg;
	};
	std::vector<uint8_t> SerialSend::send_delay(double delay)
	{
		double last_time = get_now_time();
		while(1){
			double now_time = get_now_time();
			if(now_time - last_time > delay) break;
		};
		return send();
	}
	void SerialSend::show()
	{
		std::vector<uint8_t> msg;
		int size = datas.size();
		int length_id = -1; // 长度位
		int start_length_id = -1; // 起始位
		int end_length_id = -1; // 结束位
		std::vector<std::pair<int, int>> crcs; // crc的datas下标和msg下标
		for (int i = 0; i < size; i++) {
			switch (datas[i]->operator_)
			{
			case 0:
			{
				add_in_msg(msg, datas[i]);
			}
			break;
			case 1:
			{
				length_id = msg.size();
				add_in_msg(msg, datas[i]);
			}
			break;
			case 2:
			{
				start_length_id = msg.size();
				add_in_msg(msg, datas[i]);
			}
			break;
			case 3:
			{
				end_length_id = msg.size();
				add_in_msg(msg, datas[i]);
			}
			break;
			case 4:
			{
				crcs.push_back(std::pair<int, int>(i, msg.size()));
				add_in_msg(msg, datas[i]);
			}
			break;
			case 5:
			{
				crcs.push_back(std::pair<int, int>(i, msg.size()));
				add_in_msg(msg, datas[i]);
			}
			break;
			default:
				break;
			};
		};

		// 如果长度位非-1,则计算长度
		if (length_id != -1 && start_length_id != -1 && end_length_id != -1)
		{
			msg[length_id] = end_length_id - start_length_id + 1;
		};

		// 最后处理CRC校验
		for (const auto& crc : crcs) {
			if (datas[crc.first]->operator_ == 4) { // CRC8
				msg[crc.second] = getCRC8(msg.data(), crc.second);
			}
			else if (datas[crc.first]->operator_ == 5) // CRC16
			{
				uint16_t data = getCRC16(msg.data(), crc.second);
				msg[crc.second] = (uint8_t)(data & 0x00ff);
				msg[crc.second + 1] = (uint8_t)((data >> 8) & 0x00ff);
			};
		};

		show(msg);
	}

	void SerialSend::add_in_msg(std::vector<uint8_t>& msg, SerialBase* sb)
	{
		auto add_data = sb->process();
		for (const auto& x : add_data)
			msg.push_back(x);
	}
	void SerialSend::show(const std::vector<uint8_t>& msg)
	{
		std::cout << "===============show msg===============" << std::endl;
		for (int i = 0, I = msg.size(); i < I; i++) {
			std::cout << i << " : " << static_cast<int>(msg[i]) << std::endl;
		};
	}
	void SerialSend::serial_send(uint8_t* msg, int length)
	{
#ifdef Linux
		try {
			const char* serialPort1 = "/dev/ttyUSB0";
			int fd = open(serialPort1, O_RDWR | O_NOCTTY | O_NDELAY);
			if (fd == -1) {
				const char* serialPort2 = "/dev/ttyUSB1";
				fd = open(serialPort2, O_RDWR | O_NOCTTY | O_NDELAY);
				if (fd == -1) {
					std::cout << "can not use device /dev/ttyUSB 0&1 !!!" << std::endl;
					return;
				}
			}
			struct termios serialParams;
			tcgetattr(fd, &serialParams);
			cfsetispeed(&serialParams, B115200);
			cfsetospeed(&serialParams, B115200);

			serialParams.c_cflag &= ~PARENB;
			serialParams.c_cflag &= ~CSTOPB;
			serialParams.c_cflag &= ~CSIZE;
			serialParams.c_cflag |= CS8;
			tcsetattr(fd, TCSANOW, &serialParams);
			write(fd, msg, length);
			close(fd);
			// std::cout << "can  serial!!!" << std::endl;
		}
		catch (std::exception e) {
			std::cout << "can not serial" << std::endl;
		}
#endif // Linux
	}
	;

	double SerialSend::get_now_time()
	{
		auto now = std::chrono::system_clock::now();
		auto durationSinceEpoch = now.time_since_epoch();
		auto seconds = std::chrono::duration_cast<std::chrono::seconds>(durationSinceEpoch);
		auto microseconds = std::chrono::duration_cast<std::chrono::microseconds>(durationSinceEpoch - seconds);
		double fractional_seconds = microseconds.count() * 1e-6;
		return seconds.count() + fractional_seconds;
	};

	SerialRead::SerialRead(const std::vector<SerialBase*>& datas)
		:datas(datas)
	{
		// 如果存在长度数据,获取长度值
		int size = 0;
		for (auto& x : datas)
		{
			size += x->size;
		};
		this->data_size = size;
	}
	SerialRead::~SerialRead()
	{
		for (int i = 0, I = datas.size(); i < I; i++) {
			if (datas[i] != NULL) delete datas[i];
		};
	}
	bool SerialRead::check()
	{
		if (!open_read()) return false;
		std::vector<uint8_t> data = read_data(this->data_size);
		if (data.size() != this->data_size) return false;
		// 一一校验
		size_t t__ = 0;
		for (auto& x : datas)
		{
			std::vector<uint8_t> check_data;
			if (x->operator_ == 4 || x->operator_ == 5) {
				check_data = std::vector<uint8_t>(data.begin(), data.begin() + t__ + 1);
			}
			else {
				check_data = std::vector<uint8_t>(data.begin() + t__, data.begin() + t__ + x->size);
			}
			t__ += x->size;

			if (!check_in_msg(check_data, x)) return false;
			if (x->variable) {
				x->_read_set_data_ = check_data;
			};
		};
		return true;
	}
	bool SerialRead::check(const std::vector<uint8_t>& data)
	{
		if (data.size() != this->data_size) return false;
		// 一一校验
		size_t t__ = 0;
		for (auto& x : datas)
		{
			std::vector<uint8_t> check_data;
			if (x->operator_ == 4 || x->operator_ == 5) {
				//check_data = std::vector<uint8_t>(data.begin(), data.begin() + t__ + 2);
				check_data = std::vector<uint8_t>(data.begin(), data.begin() + t__ + x->size);
			}
			else {
				check_data = std::vector<uint8_t>(data.begin() + t__, data.begin() + t__ + x->size);
			}
			t__ += x->size;

			if (!check_in_msg(check_data, x)) return false;
			if (x->variable) {
				// x->_read_set_data_ = std::vector<uint8_t>(check_data.size()); // 确保 vec2 有足够的空间

				//for (size_t i = 0; i < check_data.size(); ++i) {
				//	x->_read_set_data_[i] = check_data[check_data.size() - 1 - i];
				//};

				x->_read_set_data_ = std::vector<uint8_t>(check_data);
			};
		};
		return true;
	};

	std::vector<uint8_t> SerialRead::read_data(int data_size)
	{
#ifdef Linux
		int bytesRead = read(fd, buffer, sizeof(buffer));
		if (data_size == bytesRead) {
			std::vector<uint8_t> vec(reinterpret_cast<uint8_t*>(buffer),
				reinterpret_cast<uint8_t*>(buffer) + bytesRead);
			return vec;
		}
		else {
			return std::vector<uint8_t>();
		}
#else
		return std::vector<uint8_t>();
#endif // Linux
	}

	bool SerialRead::check_in_msg(std::vector<uint8_t> msg, SerialBase* sb)
	{

		// msg直接对应需要的位数
		if (sb->variable) return true; // 变量,无需校验
		else if (
			sb->operator_ == 0 || sb->operator_ == 1 ||
			sb->operator_ == 2 || sb->operator_ == 3
			) { // 直接校验
			return sb->process()[0] == msg[0];
		}
		else if (sb->operator_ == 4) {
			uint8_t crc8_check = msg[msg.size() - 1];
			msg = std::vector<uint8_t>(msg.begin(), msg.begin() + msg.size() - 1);
			uint8_t crc8 = getCRC8(msg.data(), msg.size());
			return crc8 == crc8_check;
		}
		else if (sb->operator_ == 5) {
			uint8_t crc16_0_check, crc16_1_check;
			crc16_0_check = msg[msg.size() - 2];
			crc16_1_check = msg[msg.size() - 1];
			msg = std::vector<uint8_t>(msg.begin(), msg.begin() + msg.size() - 2);
			uint16_t data = getCRC16(msg.data(), msg.size());
			uint8_t crc16_0 = (uint8_t)(data & 0x00ff);
			uint8_t crc16_1 = (uint8_t)((data >> 8) & 0x00ff);
			return crc16_0 == crc16_0_check && crc16_1 == crc16_1_check;
		}
		else return false;
	};

	bool SerialRead::open_read()
	{
#ifdef Linux
		// 死循环
		int fd1 = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY);
		int fd2 = open("/dev/ttyUSB1", O_RDWR | O_NOCTTY);
		if (fd1 == -1 && fd2 == -1) {
			std::cerr << "无法打开串口" << std::endl;
		}
		else {
			fd = fd1 == -1 ? fd2 : fd1;
			std::cout << "==== 串口启动成功!!! ====" << std::endl;
			return false;
		};
		struct termios config;
		tcgetattr(fd, &config);
		config.c_cflag = B115200 | CS8 | CLOCAL | CREAD;
		config.c_iflag = IGNPAR;
		config.c_oflag = 0;
		config.c_lflag = 0;
		tcflush(fd, TCIFLUSH);
		tcsetattr(fd, TCSANOW, &config);
		return true;
#endif // Linux
	};

}
