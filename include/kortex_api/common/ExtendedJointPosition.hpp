#ifndef EXTENDEDJOINTPOSITION_HPP
#define EXTENDEDJOINTPOSITION_HPP

#include <Eigen/Core>

using namespace std;
using namespace Eigen;

//Get Extended joint position
class ExtendedJointPosition
{
private:
    unsigned int numberOfInput;
    double threshold_of_change;

    Eigen::Matrix<double, 7, 1> init_q;
    Eigen::Matrix<double, 7, 1> extended_q;
    Eigen::Matrix<double, 7, 1> previous_sensor_q;
public:
    ExtendedJointPosition(unsigned int numberOfInput_args, double threshold_of_change_args)
    {
        numberOfInput = numberOfInput_args;
        threshold_of_change = threshold_of_change_args;
    }
    ~ExtendedJointPosition()
    {

    }

    void InitializeExtendedJointPosition(const Matrix<double, 7, 1> init_q_args)
    {
        init_q = this->NormalizeJointPosition(init_q_args);
        extended_q = init_q;
        previous_sensor_q = init_q;
    }

    /*
    * Move the joint position inside the [-pi,pi).
    * args : double -> 1d case
    * args : VectorXd -> multiple dimension(=dof) case
    */
    double NormalizeJointPosition(double input)
    {
//        double output = input;
        while(input > M_PI)
        {
            input -= 2*M_PI;
        }
        while(input <= -M_PI)
        {
            input += 2*M_PI;
        }
        return input;
    }

    Eigen::VectorXd NormalizeJointPosition(const VectorXd& input)
    {
        Eigen::VectorXd output = input;
        for (int i = 0; i < numberOfInput; ++i) {
            output[i] = NormalizeJointPosition(input[i]);
        }
        return output;
    }

    void EstimateExtendedJoint(const VectorXd& current_sensor_q)
    {
        for (int i = 0; i < numberOfInput; ++i) {
            if (abs(current_sensor_q[i] - previous_sensor_q[i]) >= threshold_of_change)
            {
                extended_q[i] += NormalizeJointPosition(current_sensor_q[i]) - NormalizeJointPosition(previous_sensor_q[i]);
            }
            else
            {
                int howMuchRotate;
                if (extended_q[i] >= 0)
                {
                    howMuchRotate = static_cast<int>(extended_q[i] / (2*M_PI));
                }
                else
                {
                    howMuchRotate = static_cast<int>(extended_q[i] / (2*M_PI)) - 1;
                }
                extended_q[i] = (double)howMuchRotate * (2*M_PI) + current_sensor_q[i];
            }
        }
        previous_sensor_q = current_sensor_q;
    }

    Eigen::VectorXd GetExtendedJoint()
    {
        return extended_q;
    }

};

#endif