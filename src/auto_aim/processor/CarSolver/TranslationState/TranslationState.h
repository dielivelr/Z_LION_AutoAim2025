#ifndef _TRANSLATIONSTATE_H_
#define _TRANSLATIONSTATE_H_

#include "../Ekf/Ekf.h"
#include <memory>
#include <vector>
#include <Eigen/Dense>

namespace rm
{
    class TranslationStateBase
    {
    public:
        TranslationStateBase() = default;
        virtual ~TranslationStateBase() = default;

        virtual Eigen::VectorXf get_last_state() const;
        virtual Eigen::VectorXf update(Eigen::VectorXf measurement, double new_d_t) = 0;
        virtual Eigen::VectorXf reset(Eigen::VectorXf state) = 0;
        virtual Eigen::VectorXf reset_maintain_speed(Eigen::VectorXf measurement, double new_d_t) = 0;

    protected:
        Eigen::VectorXf target_state;
    };

    class EkfTranslationState : public TranslationStateBase
    {
    public:
        EkfTranslationState() = default;
        EkfTranslationState(const std::vector<double>& q_vs, const std::vector<double>& r_vs);

        Eigen::VectorXf update(Eigen::VectorXf measurement, double new_d_t) override;
        Eigen::VectorXf reset(Eigen::VectorXf state) override;
        Eigen::VectorXf reset_maintain_speed(Eigen::VectorXf measurement, double new_d_t) override;

    private:
        float d_t = 0.01;
        std::unique_ptr<ExtendedKalmanFilter> tracker__;
    };
};
#endif // !_TRANSLATIONSTATE_H_