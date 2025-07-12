#ifndef _TRANSCONTROLER_H_
#define _TRANSCONTROLER_H_

namespace rm
{
	class TransControler
	{
	public:
		TransControler() 
			:pitch(0), yaw(0), yaw_speed(0), shoot_time(-1), move_state(1), ifget(0)
		{
		};
		TransControler(float shoot_time)
			:pitch(0), yaw(0), yaw_speed(0), shoot_time(shoot_time), move_state(1), ifget(0)
		{
		};
		double pitch;
		double yaw;
		double yaw_speed;
		float shoot_time;	// 通讯转化为canshoot
		int move_state;		// 切换装甲板时取2,其余情况取1
		int ifget;			// 一旦车需要受到自瞄控制, ifget = 1
		float rpm = 0;			// rpm转速
		int enemy_kind = 0;		// 敌方类型
		int enemy_x = 0;		
		int enemy_y = 0;
	};
};


#endif // !_TRANSCONTROLER_H_
