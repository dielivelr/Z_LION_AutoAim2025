#include "Processor.h"

namespace rm
{
	ProcessorBase::ProcessorBase(ArmorSolver* ArmorSolver__, TargetPlanner* TargetPlanner__,
		float vxz_truncation_rotate_rpm, float max_detect_distance)
		:ArmorSolver__(ArmorSolver__), TargetPlanner__(TargetPlanner__),
		vxz_truncation_rotate_speed(vxz_truncation_rotate_rpm / 30 * CV_PI),
		max_detect_distance(max_detect_distance)
	{
		clear();
	}
	TransProcessor ProcessorBase::process(std::vector<ArmorPos>& armor_poses, float new_d_t, const float& car_yaw, const float& car_pitch)
	{
		// �˵�pnp�����ʹ������
		std::vector<ArmorPos> right_armor_poses, light_loss_armor_poses;
		for (int i = 0, I = armor_poses.size(); i < I; i++)
			if (armor_poses[i].kind != 0)
				if (armor_poses[i].light_loss) {
					light_loss_armor_poses.push_back(armor_poses[i]);
				}
				else if (ArmorSolver__->solve_armor_pnp(car_yaw, car_pitch, armor_poses[i]) 
					&& !armor_poses[i].light_loss)
				{
					// ��ȡĿ�곯�����Ϣ
					ArmorSolver__->solve_armor_face(armor_poses[i], car_yaw, car_pitch);
					bool ret_angle_good = abs(-armor_poses[i].face_angle - car_yaw) < 60.0 / 180 * CV_PI;
					// �˵������Զ��װ�װ�
					if(cv::norm(armor_poses[i].absolute_middle_point) < max_detect_distance && ret_angle_good)
						right_armor_poses.push_back(armor_poses[i]);
				};

		// �滮����Ŀ��
		int last_target_kind = TargetPlanner__->get_target_kind();
		right_armor_poses = TargetPlanner__->plan(right_armor_poses);

		// ���ڿ�Ŀ�����,�������
		int target_kind = TargetPlanner__->get_target_kind();

		// ��ͬ���͵�light_loss_armor����
		for (const auto& x : light_loss_armor_poses)
			if (x.kind == last_target_kind)
				right_armor_poses.push_back(x);

		// ��һ��Ŀ��ǿ��ҷ�����Ŀ���л�,�����ö�Ӧ��ת����ת�ټ�¼����
		if (last_target_kind != 0 && last_target_kind != target_kind) {
			record_rotate_speed[last_target_kind - 1] = record_last_rotate_speed;
		};

		if (target_kind == 0) { // ���װ�װ�λ��
			clear();
			return TransProcessor();
		};

		// ����Ŀ���ȡ��Ӧת��

		// Ӣ�ۿ���Ĭ�����ֵΪ0

		//float init_new_rotate_speed = 0;

		float init_new_rotate_speed;
		if (target_kind >= 2) {
			init_new_rotate_speed = record_rotate_speed[2];
		}
		else {
			init_new_rotate_speed = record_rotate_speed[target_kind - 1];
		};

		// ����Ŀ��������ȡ
		int size = right_armor_poses.size(); // 0 1 2
		// ���ݱ���߼����в���
		std::vector<Eigen::VectorXf> States;
		if (first_time) {
			// ��ʱ���ó����
			record_last_rotate_speed = init_new_rotate_speed;
			States = init(right_armor_poses, new_d_t, car_yaw, init_new_rotate_speed);
		}
		else { // ��״̬����������
			States = update(right_armor_poses, new_d_t, car_yaw);
		};
		// �ж��Ƿ�δ�ɹ���ʼ��
		first_time = States[0].size() == 0;
		// 3V3�Կ���ƽ����ս,VY����Ϊ0,VZ����Ϊ0
		if (!first_time) {

			States[0](5) = 0;

			bool can_record_v = true;
			if (abs(States[0](4)) > 150) {
				States[0](4) = record_car_vx;
				can_record_v = false;
			};
			if (abs(States[0](6)) > 150) {
				States[0](6) = record_car_vz;
				can_record_v = false;
			};
			if (abs(States[1](3)) > 150) {
				States[1](3) = record_trans_vx;
				can_record_v = false;
			};
			if (abs(States[1](4)) > 150) {
				States[1](4) = record_trans_vz;
				can_record_v = false;
			};
			


			// ��¼��ǰ��ת��
			record_last_rotate_speed = States[0](7);
			if (abs(States[0](7)) > vxz_truncation_rotate_speed) { // ת�ٹ��߽ض�
				States[0](4) = 0;
				States[0](6) = 0;
			};

			if (can_record_v) {
				record_car_vx = States[0](4);
				record_car_vz = States[0](6);
				record_trans_vx = States[1](3);
				record_trans_vz = States[1](4);
			};
		}
		armor_poses = right_armor_poses;
		return data_to_transprocessor(States, target_kind);
	}
	void ProcessorBase::clear()
	{
		record_last_rotate_speed = 0;
		buffer_state = false;
		first_time = true;
		record_car_vx = record_car_vz = record_trans_vx = record_trans_vz = 0;
	}
	;

	cv::Mat ProcessorBase::draw_car(NoneSolver* NoneSolver__, FaceAngleSolver* FaceAngleSolver__, Coordinate* Coordinate__,
		cv::Mat src, const Eigen::VectorXf& state, bool armor_small_size,
		const float& car_yaw, const float& car_pitch, cv::Scalar color, int thickness)
	{
		auto pixel_armor_corner_points = get_draw_car_points(
			NoneSolver__, FaceAngleSolver__, Coordinate__, state, armor_small_size, car_yaw, car_pitch);

		// ����ͬװ�װ��ĵ�
		for (int i = 0; i < 4; i++) {
			for (int j = 0; j < 4; j++) {
				cv::line(src, pixel_armor_corner_points[i][j], pixel_armor_corner_points[i][(j + 1) % 4], color, thickness);
			};
		};

		return src;
	}
	std::vector<std::vector<cv::Point2f>> ProcessorBase::get_draw_car_points(NoneSolver* NoneSolver__,FaceAngleSolver* FaceAngleSolver__, Coordinate* Coordinate__, const Eigen::VectorXf& state, bool armor_small_size, const float& car_yaw, const float& car_pitch)
	{
		cv::Point3f car_pos(state(0), state(1), state(2));
		float car_face_angle = state(3);

		std::vector<cv::Point3f> armor_poses = NoneSolver__->fit_four_armors(car_pos, car_face_angle);

		// ��תװ�װ�,��ȡת��ȫ����
		std::vector<std::vector<cv::Point3f>> four_armor_corner_points(4);
		for (int i = 0; i < 4; i++) {
			four_armor_corner_points[i] = FaceAngleSolver__->rotate_armor(armor_poses[i], CV_PI / 2 * i + car_face_angle,
				armor_small_size);
		};

		// ӳ��Ϊ��ά��
		std::vector<std::vector<cv::Point2f>> pixel_armor_corner_points(4);
		for (int i = 0; i < 4; i++) {
			std::vector<cv::Point2f> pixel_four_point(4);
			// ӳ��
			for (int j = 0; j < 4; j++) {
				pixel_four_point[j] = Coordinate__->abs2pixel(four_armor_corner_points[i][j], car_yaw, car_pitch);
			};
			pixel_armor_corner_points[i] = pixel_four_point;
		};
		return pixel_armor_corner_points;
	};

	Processor::Processor(CarStateBase* CarState__, ArmorSolver* ArmorSolver__, TargetPlanner* TargetPlanner__,
		NoneSolver* NoneSolver__, SingleSolver* SingleSolver__, DoubleSolver* DoubleSolver__,
		float vxz_truncation_rotate_rpm, float max_detect_distance)
		:CarState__(CarState__),NoneSolver__(NoneSolver__),SingleSolver__(SingleSolver__), DoubleSolver__(DoubleSolver__),
		ProcessorBase(ArmorSolver__,TargetPlanner__,vxz_truncation_rotate_rpm, max_detect_distance)
	{
		clear();
	};

	
	std::vector<Eigen::VectorXf> Processor::init(const std::vector<ArmorPos>& armor_poses, float new_d_t, 
		const float& car_yaw, float init_new_rotate_speed)
	{
		SingleSolver__->set_record_rotate_speed(init_new_rotate_speed);
		return run_solve_car_strategy(CarSolver::init, armor_poses, new_d_t, car_yaw);
	}
	std::vector<Eigen::VectorXf> Processor::update(const std::vector<ArmorPos>& armor_poses, float new_d_t, const float& car_yaw)
	{
		return run_solve_car_strategy(CarSolver::update, armor_poses, new_d_t, car_yaw);
	}
	;
	TransProcessor Processor::data_to_transprocessor(const std::vector<Eigen::VectorXf>& States, int target_kind)
	{
		// ��ֵ
		float high_radius, low_radius;
		CarState__->get_high_low_radius(high_radius, low_radius);
		TransProcessor trans_processor(States, high_radius, low_radius,
			CarState__->get_high_low_height_differ(), CarState__->get_high_armor_state(), target_kind);
		return trans_processor;
	}
	std::vector<Eigen::VectorXf> Processor::run_solve_car_strategy(CarSolver::Operation operation,
		const std::vector<ArmorPos>& armor_poses, float new_d_t, const float& car_yaw)
	{
		int size = armor_poses.size();
		// ��Чװ�װ���Ŀ
		int light_get_size = 0;
		for (const auto& x : armor_poses)
			if (!x.light_loss) light_get_size++;
			
		std::vector<Eigen::VectorXf> States;
		if (light_get_size == 0) {
			States = NoneSolver__->solve_car_strategy(operation, armor_poses, new_d_t, car_yaw);
		}
		else if (light_get_size == 1 && size == 1) {
			States = SingleSolver__->solve_car_strategy(operation, armor_poses, new_d_t, car_yaw);
		}
		else
		{
			if(operation == CarSolver::Operation::init){
				States = SingleSolver__->solve_car_strategy(CarSolver::Operation::init, armor_poses, new_d_t, car_yaw);
			}
			else
				States = DoubleSolver__->solve_car_strategy(operation, armor_poses, new_d_t, car_yaw);
		};
		return States;
	}
	
	EkfProcessor::EkfProcessor(TranslationStateBase* TranslationState__, ArmorSolver* ArmorSolver__, TargetPlanner* TargetPlanner__,
		const std::vector<double>& q_v, const std::vector<double>& r_v, float vxz_truncation_rotate_rpm, float max_detect_distance)
		:ProcessorBase(ArmorSolver__, TargetPlanner__, vxz_truncation_rotate_rpm, max_detect_distance),
		TranslationState__(TranslationState__)
	{
		clear();
		auto f = [this](const Eigen::VectorXf& x) {
			Eigen::VectorXf x_new = x;
			x_new(0) += x(4) * d_t;
			x_new(1) += x(5) * d_t;
			x_new(2) += x(6) * d_t;
			x_new(3) += x(7) * d_t;
			return x_new;
			};
		// J_f - Jacobian of process function
		// ��ʼ��һЩ�˲�ֵ
		auto j_f = [this](const Eigen::VectorXf&) {
			Eigen::MatrixXf f(9, 9);
			// clang-format off
			f << 1, 0, 0, 0, d_t, 0, 0, 0, 0,
				0, 1, 0, 0, 0, d_t, 0, 0, 0,
				0, 0, 1, 0, 0, 0, d_t, 0, 0,
				0, 0, 0, 1, 0, 0, 0, d_t, 0,
				0, 0, 0, 0, 1, 0, 0, 0, 0,
				0, 0, 0, 0, 0, 1, 0, 0, 0,
				0, 0, 0, 0, 0, 0, 1, 0, 0,
				0, 0, 0, 0, 0, 0, 0, 1, 0;
				0, 0, 0, 0, 0, 0, 0, 0, 1;
			// clang-format on
			return f;
			};
		// h - Observation function
		auto h = [this](const Eigen::VectorXf& x) {
			Eigen::VectorXf z(4);
			float xc = x(0), yc = x(1), yaw = CV_PI / 2 + x(3);
			float r = x(8);
			z(0) = xc - r * cos(yaw);  // xa
			z(1) = yc - r * sin(yaw);  // ya
			z(2) = x(2);               // za
			z(3) = x(3);               // yaw
			return z;
			};
		// J_h - Jacobian of observation function
		auto j_h = [this](const Eigen::VectorXf& x) {
			Eigen::MatrixXf h(4, 9);
			float yaw = CV_PI / 2 + x(3);
			float r = x(8);
			// clang-format off
			//    xc   yc   zc   yaw         vxc  vyc  vzc  vyaw r
			h << 1, 0, 0, r* sin(yaw), 0, 0, 0, 0, -cos(yaw),
				0, 1, 0, -r * cos(yaw), 0, 0, 0, 0, -sin(yaw),
				0, 0, 1, 0, 0, 0, 0, 0, 0,
				0, 0, 0, 1, 0, 0, 0, 0, 0;
			// clang-format on
			return h;
			};
		// Q - process noise covariance matrix ��������Э�������
		// 1e-2, 1e-2, 1e-2, 2e-2, 5e-2, 5e-2, 1e-4, 4e-2
		Eigen::DiagonalMatrix<float, 9> q;
		q.diagonal() << q_v[0], q_v[1], q_v[2], q_v[3], q_v[4], q_v[5], q_v[6], q_v[7], q_v[8];
		// R - measurement noise covariance matrix
		 // 1e-1, 1e-1, 1e-1, 2e-1
		Eigen::DiagonalMatrix<float, 4> r;
		r.diagonal() << r_v[0], r_v[1], r_v[2], r_v[3];
		// P - error estimate covariance matrix
		Eigen::DiagonalMatrix<float, 9> p0;
		p0.setIdentity();
		tracker__ = std::make_unique<ExtendedKalmanFilter>(f, h, j_f, j_h, q, r, p0);
	}
	std::vector<Eigen::VectorXf> EkfProcessor::init(const std::vector<ArmorPos>& armor_poses, float new_d_t, 
		const float& car_yaw, float init_new_rotate_speed)
	{
		this->d_t = new_d_t;
		Eigen::VectorXf car_state, trans_state;
		
		std::vector<ArmorPos> armor_true_poses;
		for (const auto& x : armor_poses)
			if (!x.light_loss) armor_true_poses.push_back(x);

		if (armor_true_poses.empty()) return {};
		ArmorPos armor_pos = armor_true_poses[0];

		// ��ʼ������
		last_r = 0.25;
		// ת����������ϵ
		Eigen::VectorXf state_(4);
		state_(0) = armor_pos.absolute_middle_point.x / 100;
		state_(1) = armor_pos.absolute_middle_point.z / 100;
		state_(2) = armor_pos.absolute_middle_point.y / 100;
		state_(3) = armor_pos.face_angle;
		last_z = state_(2); // ����Ϊ������
		last_yaw = state_(3);
		float yaw = CV_PI / 2 + armor_pos.face_angle;

		Eigen::VectorXf car_state_(9);
		car_state_ << state_(0) + last_r * cos(yaw), state_(1) + last_r * sin(yaw),
			state_(2), state_(3), 0, 0, 0, init_new_rotate_speed, last_r;
		tracker__->setState(car_state_);
		target_state = car_state_;

		// ��ʼ��ƽ��
		Eigen::VectorXf trans_state_(5);
		trans_state_ << armor_pos.absolute_middle_point.x, armor_pos.absolute_middle_point.y,
			armor_pos.absolute_middle_point.z, 0, 0;
		trans_state = TranslationState__->reset(trans_state_);

		// ���
		car_state = get_last_state();

		return { car_state , trans_state };
	}
	std::vector<Eigen::VectorXf> EkfProcessor::update(const std::vector<ArmorPos>& armor_poses, float new_d_t, const float& car_yaw)
	{
		this->d_t = new_d_t;
		Eigen::VectorXf car_state, trans_state;

		std::vector<ArmorPos> armor_true_poses;
		for (const auto& x : armor_poses)
			if (!x.light_loss) armor_true_poses.push_back(x);

		// TODO: 
		if (armor_true_poses.empty()) { // �Ը���
			Eigen::VectorXf car_state, trans_state;
			// �Ը���
			Eigen::VectorXf last_state = get_last_state();
			last_state(0) += last_state(4) * new_d_t;
			last_state(1) += last_state(5) * new_d_t;
			last_state(2) += last_state(6) * new_d_t;
			last_state(3) += last_state(7) * new_d_t;
			// �������³����
			last_yaw = last_state(3);

			// תΪ��������ϵ
			last_state(0) = last_state(0) / 100.0;
			last_state(1) = last_state(2) / 100.0;
			last_state(2) = last_state(1) / 100.0;
			last_state(4) = last_state(4) / 100.0;
			last_state(5) = last_state(6) / 100.0;
			last_state(6) = last_state(5) / 100.0;
			last_state(8) = last_state(8) / 100.0;

			tracker__->setState(last_state);
			target_state = last_state;

			car_state = get_last_state();

			// ����ƽ��
			Eigen::VectorXf last_trans_state = TranslationState__->get_last_state();
			last_trans_state(0) += last_trans_state(3) * new_d_t;
			last_trans_state(2) += last_trans_state(4) * new_d_t;

			Eigen::VectorXf trans_measurement(3);
			trans_measurement << last_trans_state(0), last_trans_state(1), last_trans_state(2);
			trans_state = TranslationState__->reset_maintain_speed(trans_measurement, new_d_t);

			return { car_state , trans_state };
		};

		ArmorPos armor_pos = armor_true_poses[0];
		// ת����������ϵ
		Eigen::VectorXf state_(4);
		state_(0) = armor_pos.absolute_middle_point.x / 100;
		state_(1) = armor_pos.absolute_middle_point.z / 100;
		state_(2) = armor_pos.absolute_middle_point.y / 100;
		state_(3) = armor_pos.face_angle;
		
		bool ret_angle_jump = angle_jump(armor_pos.face_angle);
		last_yaw = state_(3);
		// �Ƕ�����
		if (false) { // 
			// ����λ����init
			Eigen::VectorXf last_state = tracker__->predict();
			// ��ȡ���º��vx,vy,vz
			std::swap(target_state(8), last_r);
			std::swap(target_state(2), last_z);
			//Eigen::VectorXf last_state = target_state;
			Eigen::VectorXf car_state_(4);
			car_state_ << state_(0), state_(1), state_(2), state_(3);
			Eigen::VectorXf new_state = tracker__->update(car_state_);
			new_state(3) = state_(3);
			new_state(7) = last_state(7);
			new_state(6) = last_state(6); // ȷ��z���ٶ�һ��
			new_state(8) = last_state(8);

			tracker__->setState(new_state);
			this->target_state = new_state;
			
			// ����ƽ��
			Eigen::VectorXf trans_state_(3);
			trans_state_ << armor_pos.absolute_middle_point.x, armor_pos.absolute_middle_point.y,
				armor_pos.absolute_middle_point.z;
			trans_state = TranslationState__->reset_maintain_speed(trans_state_, new_d_t);
		}
		else {

			// ��������
			tracker__->predict();
			Eigen::VectorXf car_state_(4);
			car_state_ << state_(0), state_(1), state_(2), state_(3);
			target_state = tracker__->update(car_state_);

			// ����ƽ��
			Eigen::VectorXf trans_state_(3);
			trans_state_ << armor_pos.absolute_middle_point.x, armor_pos.absolute_middle_point.y,
				armor_pos.absolute_middle_point.z;
			trans_state = TranslationState__->update(trans_state_, new_d_t);
		};

		car_state = get_last_state();
		return { car_state, trans_state };
	}
	TransProcessor EkfProcessor::data_to_transprocessor(const std::vector<Eigen::VectorXf>& States, int target_kind)
	{
		Eigen::VectorXf car_state = get_last_state();
		// ͳһ��λ
		return TransProcessor(States, car_state(8), last_r * 100, car_state(1) - last_z * 100,true, target_kind);
	}
	;

	bool EkfProcessor::angle_jump(float face_yaw)
	{
		//std::cout << "abs(face_yaw - last_yaw)  : " << abs(face_yaw - last_yaw) << std::endl;
		bool ret = abs(face_yaw - last_yaw) > 0.4;
		return ret;
	};

	Eigen::VectorXf EkfProcessor::get_last_state() const
	{
		Eigen::VectorXf state = target_state;
		state(0) = target_state(0) * 100;
		state(1) = target_state(2) * 100;
		state(2) = target_state(1) * 100;
		state(4) = target_state(4) * 100;
		state(5) = target_state(6) * 100;
		state(6) = target_state(5) * 100;
		state(8) = target_state(8) * 100;
		return state;
	}

}
