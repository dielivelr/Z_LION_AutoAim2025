#include"Coordinate.h"

namespace rm
{
	bool Coordinate::pnp(cv::Mat& Rvec, cv::Mat& Tvec, cv::Mat intrinsic, cv::Mat distcoeffs,
		const std::vector<cv::Point2f>& four_pixel_points
		, bool armor_small_size, bool SOLVEPNP_IPPE)
	{
		return cv::solvePnP(armor_small_size ? small_armor_world_points : big_armor_world_points,
			four_pixel_points, intrinsic, distcoeffs, Rvec, Tvec, false,
			SOLVEPNP_IPPE ? cv::SOLVEPNP_IPPE : cv::SOLVEPNP_ITERATIVE);
	};


	cv::Mat Coordinate::get_rot_mat_x(float x)
	{
		return cv::Mat_<float>(4, 4) <<
			1, 0, 0, 0,
			0, cos(x), -sin(x), 0,
			0, sin(x), cos(x), 0,
			0, 0, 0, 1;
	};

	cv::Mat Coordinate::get_rot_mat_y(float y)
	{
		return cv::Mat_<float>(4, 4) <<
			cos(y), 0, sin(y), 0,
			0, 1, 0, 0,
			-sin(y), 0, cos(y), 0,
			0, 0, 0, 1;
	}

	cv::Mat Coordinate::get_rot_mat_z(float z)
	{
		return cv::Mat_<float>(4, 4) <<
			cos(z), -sin(z), 0, 0,
			sin(z), cos(z), 0, 0,
			0, 0, 1, 0,
			0, 0, 0, 1;
	};

	cv::Mat Coordinate::get_rot_mat_xyz(float x, float y, float z)
	{
		return get_rot_mat_z(z) * get_rot_mat_y(y) * get_rot_mat_x(x);
	};

	cv::Mat Coordinate::get_tans_mat(float x, float y, float z)
	{
		return cv::Mat_<float>(4, 4) <<
			1, 0, 0, x,
			0, 1, 0, y,
			0, 0, 1, z,
			0, 0, 0, 1;
	};

	cv::Point3f Coordinate::cam2abs(cv::Point3f position, const float& car_yaw, const float& car_pitch,
		const float& shaft_differ_x, const float& shaft_differ_y, const float& shaft_differ_z)
	{
		cv::Mat rot_mat = get_rot_mat_xyz(car_pitch, car_yaw, 0.0);
		cv::Mat trans_mat = get_tans_mat(shaft_differ_x, shaft_differ_y, shaft_differ_z);
		cv::Mat tf_mat = rot_mat * trans_mat;
		cv::Mat target = (cv::Mat_<float>(4, 1) << position.x, position.y, position.z, 1);
		cv::Mat abs_target = tf_mat * target;
		return cv::Point3f(abs_target.at<float>(0),
			abs_target.at<float>(1),
			abs_target.at<float>(2));
	};

	cv::Point3f Coordinate::abs2cam(cv::Point3f position, const float& car_yaw, const float& car_pitch, 
		const float& shaft_differ_x, const float& shaft_differ_y, const float& shaft_differ_z)
	{
		cv::Mat rot_mat = get_rot_mat_xyz(car_pitch, car_yaw, 0.0);
		cv::Mat trans_mat = get_tans_mat(shaft_differ_x, shaft_differ_y, shaft_differ_z);
		cv::Mat tf_mat = rot_mat * trans_mat;
		cv::Mat target = (cv::Mat_<float>(4, 1) << position.x, position.y, position.z, 1);
		cv::Mat abs_target = tf_mat.inv() * target;
		return cv::Point3f(abs_target.at<float>(0),
			abs_target.at<float>(1),
			abs_target.at<float>(2));
	}
	cv::Point3f Coordinate::rotate_angle(const cv::Point3f& position, const float& rotate_yaw, const float& rotate_pitch)
	{
		cv::Mat rot_mat = get_rot_mat_xyz(rotate_pitch, rotate_yaw, 0.0);
		cv::Mat trans_mat = get_tans_mat(0, 0, 0);
		cv::Mat tf_mat = rot_mat * trans_mat;
		cv::Mat target = (cv::Mat_<float>(4, 1) << position.x, position.y, position.z, 1);
		cv::Mat abs_target = tf_mat * target;
		Eigen::VectorXf OutVector(3);
		return cv::Point3f(abs_target.at<float>(0),
			abs_target.at<float>(1),
			abs_target.at<float>(2));
	}
	cv::Point3f Coordinate::coordinate_transformation(const cv::Point3f& position, const float& car_yaw, const float& car_pitch, 
		const float& shaft_differ_x, const float& shaft_differ_y, const float& shaft_differ_z)
	{
		return cam2abs(position, car_yaw, car_pitch, shaft_differ_x, shaft_differ_y, shaft_differ_z);
	}
	cv::Point3f Coordinate::reverse_coordinate_transformation(const cv::Point3f& position, const float& car_yaw, const float& car_pitch, const float& shaft_differ_x, const float& shaft_differ_y, const float& shaft_differ_z)
	{
		return abs2cam(position, car_yaw, car_pitch, shaft_differ_x, shaft_differ_y, shaft_differ_z);
	}
	;

	cv::Point2f Coordinate::cam2pixel(cv::Point3f position,
		cv::Mat intrinsic, cv::Mat distcoeffs)
	{
		std::vector<cv::Point3f> objectPoints;
		objectPoints.push_back(position);
		std::vector<cv::Point2f> imagePoints;
		cv::projectPoints(objectPoints, cv::Vec3d(0, 0, 0), cv::Vec3d(0, 0, 0), intrinsic, distcoeffs, imagePoints);
		return imagePoints[0];
	}
	cv::Point2f Coordinate::abs2pixel(cv::Point3f position, const float& car_yaw, const float& car_pitch,
		const float& shaft_differ_x, const float& shaft_differ_y, const float& shaft_differ_z,
		cv::Mat intrinsic, cv::Mat distCoeffs)
	{
		return cam2pixel(abs2cam(position, car_yaw, car_pitch, shaft_differ_x,
			shaft_differ_y, shaft_differ_z), intrinsic, distcoeffs);
	};

	void Coordinate::pos2angles(float& yaw, float& pitch, const cv::Point3f& position)
	{
		yaw = atan(position.x / position.z);
		pitch = atan(position.y / position.z);
	};

	cv::Point3f Coordinate::abs2sot(const cv::Point3f& abs_pos, const float& car_yaw, const float& car_pitch, float yaw_speed)
	{
		cv::Point3f cam_pos = abs2cam(abs_pos, car_yaw, car_pitch); // 转相机坐标系
		cv::Point3f sot_pos = coordinate_transformation(cam_pos, sot_shifting_yaw, sot_shifting_pitch,
			sot_shaft_differ_x, sot_shaft_differ_y, sot_shaft_differ_z);
		sot_pos.x = sot_pos.x + yaw_speed * this->wheel_to_center_distance;
		return sot_pos;
	}
	cv::Point3f Coordinate::sot2abs(const cv::Point3f& sot_pos, const float& car_yaw, const float& car_pitch, float yaw_speed)
	{
		cv::Point3f sot_pos_ = sot_pos;
		sot_pos_.x = sot_pos_.x - yaw_speed * this->wheel_to_center_distance;
		cv::Point3f cam_pos = reverse_coordinate_transformation(sot_pos_, sot_shifting_yaw, sot_shifting_pitch,
			sot_shaft_differ_x, sot_shaft_differ_y, sot_shaft_differ_z); // 转相机坐标系
		return cam2abs(cam_pos, car_yaw, car_pitch);
	}
	;

	rm::Coordinate::Coordinate(cv::Mat intrinsic, cv::Mat distcoeffs,
		const float& shaft_differ_x, const float& shaft_differ_y, 
		const float& shaft_differ_z, bool SOLVEPNP_IPPE,
		const float& sot_shaft_differ_x,
		const float& sot_shaft_differ_y,
		const float& sot_shaft_differ_z,
		const float& sot_shifting_yaw,
		const float& sot_shifting_pitch,
		const float& wheel_to_center_distance)
		:intrinsic(intrinsic), distcoeffs(distcoeffs), shaft_differ_x(shaft_differ_x), 
		shaft_differ_y(shaft_differ_y), shaft_differ_z(shaft_differ_z),
		SOLVEPNP_IPPE(SOLVEPNP_IPPE), sot_shaft_differ_x(sot_shaft_differ_x),
		sot_shaft_differ_y(sot_shaft_differ_y), sot_shaft_differ_z(sot_shaft_differ_z),
		sot_shifting_yaw(sot_shifting_yaw), sot_shifting_pitch(sot_shifting_pitch),
		wheel_to_center_distance(wheel_to_center_distance)
	{
	}

	bool Coordinate::trans_aromr_pos(const float& car_yaw, const float& car_pitch, ArmorPos& armorpos)
	{
		cv::Mat Rvec, Tvec;
		if (!pnp(Rvec, Tvec, this->intrinsic, this->distcoeffs, armorpos.pixel_four_point, armorpos.small_armor, this->SOLVEPNP_IPPE)) return false;
		cv::Point3f tvec_3f = cv::Point3f(Tvec.at<double>(0), Tvec.at<double>(1), Tvec.at<double>(2)); //只能使用double强转
		
		// 距离
		//float distance = std::sqrt(tvec_3f.x * tvec_3f.x + tvec_3f.y * tvec_3f.y + tvec_3f.z * tvec_3f.z);
		//float pitch = acos(tvec_3f.z / distance);
		//// 计算yaw（方位角）
		//float yaw = atan2(tvec_3f.y, tvec_3f.x);
		//// 对测距进行函数映射
		//float x = distance * sin(pitch) * cos(yaw);
		//float y = distance * sin(pitch) * sin(yaw);
		//float z = distance * cos(pitch);
		//tvec_3f = cv::Point3f(x, y, z);
		
		armorpos.absolute_middle_point = cam2abs(tvec_3f,car_yaw,car_pitch);



		return true;
	}

	cv::Point2f Coordinate::abs2pixel(cv::Point3f position, const float& car_yaw, const float& car_pitch)
	{
		return abs2pixel(position, car_yaw, car_pitch,shaft_differ_x, shaft_differ_y, shaft_differ_z,intrinsic, distcoeffs);
	}

	cv::Point3f Coordinate::abs2cam(cv::Point3f position, const float& car_yaw, const float& car_pitch)
	{
		return abs2cam(position, car_yaw, car_pitch, shaft_differ_x, shaft_differ_y, shaft_differ_z);
	}

	cv::Point3f Coordinate::cam2abs(cv::Point3f position, const float& car_yaw, const float& car_pitch)
	{
		return cam2abs(position, car_yaw, car_pitch, shaft_differ_x, shaft_differ_y, shaft_differ_z);
	}

}
