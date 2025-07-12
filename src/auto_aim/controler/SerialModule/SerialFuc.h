#ifndef SERIALFUC_H
#define SERIALFUC_H

/*************************************************/
//
// 串口通讯关节成员
//
/*************************************************/
#define Linux
#include <cmath>
#include <chrono>
#include <string>
#include <fcntl.h>
#include <errno.h>
#include <stdlib.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <iostream>
#include <vector>
#include <cstring>

namespace rm
{
	// 将一个float按照60为节,转化为其整数部分和小数部分
	static void split_60(uint8_t& integer, uint8_t& decimal, float input)
	{
		input = -input; //... 调试用
		if (input <= 0) {
			integer = -ceil(input);
			decimal = floor(abs(input - ceil(input)) * 100);
		}
		else {
			integer = floor(input) + 60;
			decimal = floor((input - floor(input)) * 100);
		}
	};

	// 将一个float按照30为节,转化为其整数部分和小数部分
	static void split_30(uint8_t& integer, uint8_t& decimal, float input)
	{
		if (input < 0) {
			integer = -ceil(input) + 30;
			decimal = floor(abs(input - ceil(input)) * 100);
		}
		else {
			integer = floor(input);
			decimal = floor((input - floor(input)) * 100);
		};
	};

	inline float Hex_To_Decimal(unsigned char* Byte)
	{
		return *((float*)Byte);
	};


	/* 串口基类 */
	class SerialBase
	{
	public:
		// 操作符
		// 0: 常量符
		// 1: 长度符
		// 2: 长度起始符
		// 3: 长度停止符
		// 4: CRC8校验符
		// 5: CRC16校验符
		int operator_;
		int size; // 所占字符数
		bool variable = false; // 是否为变量,变量无需进行对应位校验
		virtual std::vector<uint8_t> process() = 0; // 处理
		std::vector<uint8_t> _read_set_data_; // 变量放入的数据
	};

	/* 纯常量 */
	class Constant : public SerialBase
	{
	public:
		Constant(const uint8_t& data) :data(data)
		{
			operator_ = 0;
			size = 1;
		};
		std::vector<uint8_t> process() override
		{
			return { data };
		};
	private:
		const uint8_t data;
	};

	/* uint_8类型 */
	class Uint_8 : public SerialBase
	{
	public:
		Uint_8()
		{
			operator_ = 0;
			size = 1;
			variable = true;
		};
		void set_data(uint8_t data) {
			this->data = data;
		};
		int back_data() {
			return (int)this->_read_set_data_[0];
		};
		std::vector<uint8_t> process() override
		{
			return { data };
		};
	private:
		uint8_t data = 0;
	};

	/* Float30类型 */
	class Float30 : public SerialBase
	{
	public:
		Float30()
		{
			operator_ = 0;
			size = 2;
			variable = true;
		};
		void set_data(float input) {
			uint8_t integer, decimal;
			split_30(integer, decimal, input);
			this->data = { integer, decimal };
		};
		std::vector<uint8_t> process() override
		{
			return data;
		};
	private:
		std::vector<uint8_t> data = { 0,0 };
	};

	/* Float60类型 */
	class Float60 : public SerialBase
	{
	public:
		Float60()
		{
			operator_ = 0;
			size = 2;
			variable = true;
		};
		void set_data(float input) {
			uint8_t integer, decimal;
			split_60(integer, decimal, input);
			this->data = { integer, decimal };
		};
		std::vector<uint8_t> process() override
		{
			return data;
		};
	private:
		std::vector<uint8_t> data = { 0,0 };
	};

	/* 纯Float类型 */
	class Float : public SerialBase
	{
	public:
		Float()
		{
			operator_ = 0;
			size = 4;
			variable = true;
		};
		void set_data(float input) {
			uint8_t byte[4];
			std::memcpy(byte, &input, sizeof(input));
			this->data = { byte[0], byte[1] , byte[2] ,byte[3] };
			//const uint8_t* pre__ = reinterpret_cast<const uint8_t*>(&data);
			//this->data = { pre__[0], pre__[1] , pre__[2] ,pre__[3] };
		};
		std::vector<uint8_t> process() override
		{
			return data;
		};
		float back_data() {
			auto byte = _read_set_data_.data();
			float value;
			std::memcpy(&value, byte, sizeof(value));
			return value;
		};

	private:
		std::vector<uint8_t> data = { 0,0,0,0 };
	};

	/* 纯int类型 */
	class Int : public SerialBase
	{
	public:
		Int()
		{
			operator_ = 0;
			size = 2;
			variable = true;
		};
		void set_data(int input) {
			uint8_t byte[2];
			std::memcpy(byte, &input, sizeof(input));
			this->data = { byte[0], byte[1] };
			//const uint8_t* pre__ = reinterpret_cast<const uint8_t*>(&data);
			//this->data = { pre__[0], pre__[1] , pre__[2] ,pre__[3] };
		};
		std::vector<uint8_t> process() override
		{
			return data;
		};
		int back_data() {
			auto byte = _read_set_data_.data();
			int value;
			std::memcpy(&value, byte, sizeof(value));
			return value;
		};

	private:
		std::vector<uint8_t> data = { 0,0,0,0 };
	};

	/* 长度符类型 */
	class Length : public SerialBase
	{
	public:
		Length()
		{
			operator_ = 1;
			size = 1;
		};
		std::vector<uint8_t> process() override
		{
			return { 0 };
		};
	};

	/* 长度起始符 */
	class LengthStart : public Constant
	{
	public:
		LengthStart(const uint8_t& data)
			:Constant(data)
		{
			operator_ = 2;
			size = 1;
		};
	};

	/* 长度停止符 */
	class LengthEnd : public Constant
	{
	public:
		LengthEnd(const uint8_t& data)
			:Constant(data)
		{
			operator_ = 3;
			size = 1;
		};
	};

	/* CRC8校验符 */
	class CRC8 : public SerialBase
	{
	public:
		CRC8()
		{
			operator_ = 4;
			size = 1;
		};
		std::vector<uint8_t> process() override
		{
			return { 0 };
		};
	};

	/* CRC16校验符 */
	class CRC16 : public SerialBase
	{
	public:
		CRC16()
		{
			operator_ = 5;
			size = 2;
		};
		std::vector<uint8_t> process() override
		{
			return { 0,0 };
		};
	};
};


#endif // !SERIALFUC_H
