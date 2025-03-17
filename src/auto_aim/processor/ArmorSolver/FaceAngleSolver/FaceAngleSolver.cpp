#include "FaceAngleSolver.h"

namespace rm
{
	FaceAngleSolver::FaceAngleSolver(Coordinate* Coordinate__)
		:Coordinate__(Coordinate__)
	{
	};

	std::vector<cv::Point3f> FaceAngleSolver::rotate_armor_yaw(cv::Point3f armor_center,
		float yaw_angle, bool armor_small_size)
	{
		cv::Point3f top_left, top_right, bottom_left, bottom_right;
		if (armor_small_size) {
			top_left = cv::Point3f(-small_armor_length / 2, -small_armor_width / 2, 0);
			top_right = cv::Point3f(small_armor_length / 2, -small_armor_width / 2, 0);
			bottom_left = cv::Point3f(-small_armor_length / 2, small_armor_width / 2, 0);
			bottom_right = cv::Point3f(small_armor_length / 2, small_armor_width / 2, 0);
		}
		else {
			top_left = cv::Point3f(-big_armor_length / 2, -big_armor_width / 2, 0);
			top_right = cv::Point3f(big_armor_length / 2, -big_armor_width / 2, 0);
			bottom_left = cv::Point3f(-big_armor_length / 2, big_armor_width / 2, 0);
			bottom_right = cv::Point3f(big_armor_length / 2, big_armor_width / 2, 0);
		};

		cv::Mat rot_y = cv::Mat::eye(3, 3, CV_32F);
		rot_y.at<float>(0, 0) = cos(yaw_angle);
		rot_y.at<float>(0, 2) = -sin(yaw_angle);
		rot_y.at<float>(2, 0) = sin(yaw_angle);
		rot_y.at<float>(2, 2) = cos(yaw_angle);

		// 绕着yaw旋转
		cv::Point3f rotated_top_left = rotate_point(top_left, rot_y, cv::Point3f(0, 0, 0)) + armor_center;
		cv::Point3f rotated_top_right = rotate_point(top_right, rot_y, cv::Point3f(0, 0, 0)) + armor_center;
		cv::Point3f rotated_bottom_left = rotate_point(bottom_left, rot_y, cv::Point3f(0, 0, 0)) + armor_center;
		cv::Point3f rotated_bottom_right = rotate_point(bottom_right, rot_y, cv::Point3f(0, 0, 0)) + armor_center;

		return {
			rotated_bottom_left,rotated_top_left,rotated_top_right,rotated_bottom_right
		};
	};

	float FaceAngleSolver::switch_absolute_camera_angle(float angle, const float& car_yaw)
	{
		angle = -angle - car_yaw;
		return angle;
	}

	float FaceAngleSolver::conversion_arc_to_angle(float angle)
	{
		return angle * 180 / CV_PI;
	}

	std::vector<cv::Point3f> FaceAngleSolver::get_armor_intersection(const ArmorPos& armor_pos, float absolute_angle,
		bool armor_small_size, float armor_pitch)
	{
		return rotate_armor(armor_pos.absolute_middle_point, absolute_angle, armor_small_size, armor_pitch);
	};

	std::vector<cv::Point3f> FaceAngleSolver::rotate_armor(cv::Point3f armor_center, float yaw_angle,
		bool armor_small_size, float armor_pitch)
	{
		std::vector<cv::Point3f> rotate_points = rotate_armor_yaw(armor_center, yaw_angle, armor_small_size);
		return rotate_pitch(rotate_points, (rotate_points[0] + rotate_points[1]) / 2,
			(rotate_points[2] + rotate_points[3]) / 2, armor_pitch);
	}



	std::vector<cv::Point3f> FaceAngleSolver::rotate_pitch(const std::vector<cv::Point3f>& points,
		cv::Point3f pt1, cv::Point3f pt2, float pitch_angle) {

		// 计算旋转轴（单位向量）
		cv::Point3f direction = pt2 - pt1;
		float length = cv::sqrt(direction.x * direction.x + direction.y * direction.y + direction.z * direction.z);
		cv::Point3f unitDirection = direction / length;

		// 计算旋转矩阵中的一些常用元素
		float cosA = cos(pitch_angle);
		float sinA = sin(pitch_angle);
		float oneMinusCosA = 1.0 - cosA;
		float ux = unitDirection.x;
		float uy = unitDirection.y;
		float uz = unitDirection.z;

		// 构建旋转矩阵
		cv::Mat rotationMatrix = (cv::Mat_<float>(4, 4) <<
			cosA + ux * ux * oneMinusCosA, ux * uy * oneMinusCosA - uz * sinA, ux * uz * oneMinusCosA + uy * sinA, 0,
			uy * ux * oneMinusCosA + uz * sinA, cosA + uy * uy * oneMinusCosA, uy * uz * oneMinusCosA - ux * sinA, 0,
			uz * ux * oneMinusCosA - uy * sinA, uz * uy * oneMinusCosA + ux * sinA, cosA + uz * uz * oneMinusCosA, 0,
			0, 0, 0, 1);

		// 移动点到旋转线的起点
		cv::Mat translationMatrix = cv::Mat::eye(4, 4, CV_32F);
		translationMatrix.at<float>(0, 3) = -pt1.x;
		translationMatrix.at<float>(1, 3) = -pt1.y;
		translationMatrix.at<float>(2, 3) = -pt1.z;

		// 移回原来的位置
		cv::Mat inverseTranslationMatrix = cv::Mat::eye(4, 4, CV_32F);
		inverseTranslationMatrix.at<float>(0, 3) = pt1.x;
		inverseTranslationMatrix.at<float>(1, 3) = pt1.y;
		inverseTranslationMatrix.at<float>(2, 3) = pt1.z;

		//组合变换矩阵
		cv::Mat transformMatrix = inverseTranslationMatrix * rotationMatrix * translationMatrix;

		// 应用变换矩阵
		std::vector<cv::Point3f> rotate_points_pitch;
		for (cv::Point3f point : points) {
			cv::Mat pointMat = (cv::Mat_<float>(4, 1) << point.x, point.y, point.z, 1);
			cv::Mat rotatedPointMat = transformMatrix * pointMat;
			point.x = rotatedPointMat.at<float>(0, 0);
			point.y = rotatedPointMat.at<float>(1, 0);
			point.z = rotatedPointMat.at<float>(2, 0);
			rotate_points_pitch.push_back(point);
		};
		return rotate_points_pitch;
	};

	cv::Point3f FaceAngleSolver::rotate_point(const cv::Point3f& point,
		const cv::Mat& rot_y, const cv::Point3f& center)
	{
		// 将三维点转换为行向量形式的cv::Mat
		cv::Mat point_vec = (cv::Mat_<float>(3, 1) << point.x, point.y, point.z);
		cv::Mat center_vec = (cv::Mat_<float>(3, 1) << center.x, center.y, center.z);
		// 应用旋转矩阵
		cv::Mat rotated_vec = rot_y * (point_vec - center_vec);

		// 转换回cv::Point3f并加上中心点坐标
		cv::Point3f rotated_point(rotated_vec.at<float>(0), rotated_vec.at<float>(1),
			rotated_vec.at<float>(2));
		return rotated_point + center;
	};

	IterationAngleSolver::IterationAngleSolver(int orientationangle_iteration, Coordinate* Coordinate__,
		float increase_param_blue, float increase_param_red)
		:FaceAngleSolver(Coordinate__), orientationangle_iteration(orientationangle_iteration),
		increase_param_blue(increase_param_blue),increase_param_red(increase_param_red)
	{
	}

	float IterationAngleSolver::absolute_angle_solve(const ArmorPos& armor_pos,
		const float& car_yaw, const float& car_pitch)
	{
		float increase_param = enemy_blue ? increase_param_blue : increase_param_red;
		// 同时迭代朝向角
		float left = -car_yaw - CV_PI / 2;
		float right = -car_yaw + CV_PI / 2;
		int iteration = this->orientationangle_iteration;
		while (iteration--) {
			float m1 = left + (right - left) / 3;
			float m2 = right - (right - left) / 3;
			std::vector<cv::Point3f> points0 = rotate_armor(armor_pos.absolute_middle_point, m1, armor_pos.small_armor);
			std::vector<cv::Point3f> points1 = rotate_armor(armor_pos.absolute_middle_point, m2, armor_pos.small_armor);

			std::vector<cv::Point2f> pointspix0(4), pointspix1(4);
			for (int i = 0; i < 4; i++) {
				pointspix0[i] = Coordinate__->abs2pixel(points0[i], car_yaw, car_pitch);
				pointspix1[i] = Coordinate__->abs2pixel(points1[i], car_yaw, car_pitch);
			};
			if (cost(armor_pos.pixel_four_point, pointspix0)
				< cost(armor_pos.pixel_four_point, pointspix1)) {
				right = m2;
			}
			else {
				left = m1;
			};
		};
		float face_angle = (right + left) / 2;

		//// 同时迭代朝向角
		//float left_b = -car_yaw - CV_PI / 2;
		//float right_b = -car_yaw + CV_PI / 2;
		//iteration = this->orientationangle_iteration;
		//while (iteration--) {
		//	float m1 = left_b + (right_b - left_b) / 3;
		//	float m2 = right_b - (right_b - left_b) / 3;
		//	std::vector<cv::Point3f> points0 = rotate_armor(armor_pos.absolute_middle_point, m1, armor_pos.small_armor);
		//	std::vector<cv::Point3f> points1 = rotate_armor(armor_pos.absolute_middle_point, m2, armor_pos.small_armor);

		//	std::vector<cv::Point2f> pointspix0(4), pointspix1(4);
		//	for (int i = 0; i < 4; i++) {
		//		pointspix0[i] = Coordinate__->abs2pixel(points0[i], car_yaw, car_pitch);
		//		pointspix1[i] = Coordinate__->abs2pixel(points1[i], car_yaw, car_pitch);
		//	};
		//	if (balance_cost(armor_pos.pixel_four_point, pointspix0)
		//		< balance_cost(armor_pos.pixel_four_point, pointspix1)) {
		//		right_b = m2;
		//	}
		//	else {
		//		left_b = m1;
		//	};
		//};
		//float face_angle_b = (right_b + left_b) / 2;

		// 角度较小使用balance
		//if (abs(FaceAngleSolver::switch_absolute_camera_angle(face_angle_b,car_yaw) * 180 / CV_PI) < 10) face_angle = face_angle_b;

		face_angle = increase_angle(face_angle, car_yaw, increase_param);
		return face_angle;
	}
	void FaceAngleSolver::set_enemy_color(bool enemy_blue)
	{
		this->enemy_blue = enemy_blue;
	}
	;

	DiagonalAngleSolver::DiagonalAngleSolver(float inclined, float pixel_error,
		int orientationangle_iteration, Coordinate* Coordinate__, float increase_param_blue, float increase_param_red)
		:inclined(inclined), pixel_error(pixel_error), 
		 IterationAngleSolver(orientationangle_iteration, Coordinate__, increase_param_blue, increase_param_red)
	{
	}

	float DiagonalAngleSolver::cost(const std::vector<cv::Point2f>& armor_four_pixel_points, const std::vector<cv::Point2f>& points2f)
	{
		float cost = 0.;
		cv::Point2f M = (armor_four_pixel_points[0] + armor_four_pixel_points[1] + armor_four_pixel_points[2] + armor_four_pixel_points[3]) / 4;
		std::size_t size = armor_four_pixel_points.size();
		for (std::size_t i = 0u; i < size; ++i) {
			std::size_t p = (i + 1u) % size;
			// i - p 构成线段。过程：先移动起点，再补长度，再旋转
			cv::Point2f ref_d = armor_four_pixel_points[p] - armor_four_pixel_points[i];
			cv::Point2f pt_d = points2f[p] - points2f[i];
			float pixel_dis =
				(0.5 * (cv::norm(armor_four_pixel_points[i] - points2f[i]) + cv::norm(armor_four_pixel_points[p] - points2f[p]))
					+ std::fabs(cv::norm(ref_d) - cv::norm(pt_d))) / cv::norm(ref_d);
			float angular_dis = abs(atan((armor_four_pixel_points[i].y - armor_four_pixel_points[p].y) /
				(armor_four_pixel_points[i].x - armor_four_pixel_points[p].x)) -
				atan((points2f[i].y - points2f[p].y) / (points2f[i].x - points2f[p].x)));
			float cost_i = (pixel_dis * std::sin(inclined)) * (pixel_dis * std::sin(inclined))
				+ (angular_dis * std::cos(inclined)) * (angular_dis * std::cos(inclined))
				* pixel_error;
			// 重投影像素误差越大，越相信斜率
			cost += std::sqrt(cost_i);
		};
		return cost;
	}
	;

	float IterationAngleSolver::balance_cost(const std::vector<cv::Point2f>& armor_four_pixel_points, const std::vector<cv::Point2f>& points2f)
	{
		cv::Point2d point0 = (armor_four_pixel_points[0] + armor_four_pixel_points[1]) / 2;
		cv::Point2d point1 = (armor_four_pixel_points[2] + armor_four_pixel_points[3]) / 2;
		cv::Point2d point0_ = (points2f[0] + points2f[1]) / 2;
		cv::Point2d point1_ = (points2f[2] + points2f[3]) / 2;
		double k0 = (point0.y - point1.y) / (point0.x - point1.x);
		double k1 = (point0_.y - point1_.y) / (point0_.x - point1_.x);
		return abs(atan(k1) - atan(k0));
	}

	float IterationAngleSolver::increase_angle(float face_angle,float car_yaw,float increase_param){
		car_yaw = car_yaw * 180 / CV_PI;
		face_angle = face_angle * 180 / CV_PI;
		float face_angle_camera = -face_angle - car_yaw;
		// 一次过原点函数映射
		float diff_ratio = increase_param / (45.0 - increase_param);
		face_angle -= diff_ratio * face_angle_camera;
		face_angle = face_angle / 180 * CV_PI;
		return face_angle;
	}
	float IterationAngleSolver::correct_face_angle(float face_angle, float face_angle_compute, float car_yaw)
	{
		// 转为相机坐标系下的朝向角
		float face_angle_camera = -face_angle - car_yaw;
		float face_angle_compute_camera = -face_angle_compute - car_yaw;
		// 
		// if(abs(face_angle_compute_camera) < 10.0 / 180 * CV_PI){
		// 	face_angle_camera = face_angle_compute_camera;
		// }
		// else{
		// 	face_angle_camera = face_angle_compute_camera > 0 ? abs(face_angle_camera) : -abs(face_angle_camera);
		// }

		return -face_angle_camera - car_yaw;
	}

};