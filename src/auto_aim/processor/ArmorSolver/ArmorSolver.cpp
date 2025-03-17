#include "ArmorSolver.h"
namespace rm
{
	ArmorSolver::ArmorSolver(Coordinate* Coordinate__, FaceAngleSolver* FaceAngleSolver__)
		:Coordinate__(Coordinate__), FaceAngleSolver__(FaceAngleSolver__)
	{
	}
	bool ArmorSolver::solve_armor_pnp(const float& car_yaw, const float& car_pitch, ArmorPos& armorpos)
	{
		return Coordinate__->trans_aromr_pos(car_yaw, car_pitch, armorpos);
	}
	void ArmorSolver::solve_armor_face(ArmorPos& armorpos, const float& car_yaw, const float& car_pitch)
	{
		armorpos.face_angle = FaceAngleSolver__->absolute_angle_solve(armorpos, car_yaw, car_pitch);
		armorpos.absolute_four_point = FaceAngleSolver__->get_armor_intersection(armorpos, armorpos.face_angle, armorpos.small_armor, armor_pitch);
	}
	;
	bool ArmorSolver::solve_armor_pos(const float& car_yaw, const float& car_pitch, ArmorPos& armorpos)
	{
		if (!solve_armor_pnp(car_yaw, car_pitch, armorpos)) return false;
		//solve_armor_face(armorpos, car_yaw, car_pitch);
		return true;
	}
	cv::Mat ArmorSolver::draw_face_angle(cv::Mat src, const ArmorPos& armor_pos, const float& car_yaw, cv::Scalar color)
	{
		float angle = FaceAngleSolver__->switch_absolute_camera_angle(armor_pos.face_angle, car_yaw) * 180 / CV_PI;
		cv::putText(src, std::to_string(angle), cv::Point(armor_pos.pixel_four_point[2]), 2, 1, color, 3);
		return src;
	}
	;
}
