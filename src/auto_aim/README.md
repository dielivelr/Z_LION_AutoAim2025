# Visual Studio 调试注意点

1. 将调试模式设置为release模式,否则编译会报错

2. 更改sample/debug.h更改宏定义设定运行模式:
	- PROCESS_TEST 运行解算层
	- DETECT_TEST 运行识别层