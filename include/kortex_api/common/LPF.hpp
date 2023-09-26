#ifndef LPF_HPP
#define LPF_HPP

#include <Eigen/Core>
using namespace std;
using namespace Eigen;

//low-pass filter for joint torque sensor.
class LPF
{
    private:
    Eigen::VectorXd  tau_J_prev_;
    double alpha_;
    public:
    LPF(double sampling_rate, double cutoff_frequency):tau_J_prev_(7)
    {

        double tau = 1/(2*M_PI*cutoff_frequency);
        alpha_ = (tau)/(tau+sampling_rate);
    }
    void init_LPF(const Eigen::VectorXd &init_tau_J)
    {
        tau_J_prev_ = init_tau_J;
    }
    Eigen::VectorXd GetFiltered_tau_J(const Eigen::VectorXd &tau_J_raw)
    {
        Eigen::VectorXd filtered_tau_J(7);
        filtered_tau_J = tau_J_prev_ * alpha_ + tau_J_raw * (1-alpha_);
        tau_J_prev_ = filtered_tau_J;

        return filtered_tau_J;
    }
};

#endif