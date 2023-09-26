#ifndef KINOVADYNAMICS_HPP
#define KINOVADYNAMICS_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>
#include <fstream>
#include <Eigen/Dense>
#include <vector>
// #include "reading_mesh.hpp"

namespace KinovaDynamics
{
    class Body
    {
    public:
    //constant(from urdf, gc)
    Eigen::Vector3d pos_com; // position of com from {i}, expressed in {i}
    Eigen::Vector3d joint_pos;
    Eigen::Vector3d joint_axis;
    Eigen::Matrix3d joint_rot; //joint pos, axis, rot relative to {i-1}
    double mass;
    Eigen::Matrix3d inertia;
    double parent_body_index;

    //variables
    Eigen::Vector3d poscom_w; // com of body expressed in {W}
    Eigen::Matrix3d rot_b; // orientation of frame {i'} w.r.t {i}
    Eigen::Matrix3d rot_w; // orientation of body expressed in {W}
    Eigen::Vector3d pos_w; // position of body frame{i} expressed in {W}, (i frame origin)
    Eigen::Vector3d axis_w; // joint i axis of body i, expressed in {W}.
    Eigen::Matrix3d inertia_w;
    Eigen::Vector3d ang_vel_w;
    };

    inline Eigen::Matrix3d skew(Eigen::Vector3d v)
    {
        Eigen::Matrix3d out;

        out << 0,-v(2),v(1),
                v(2),0, -v(0),
                -v(1),v(0),0;

        return out;
    }

    inline Eigen::Matrix3d RPY2RotMatrix(const double &roll, const double &pitch, const double &yaw)
    {
        Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());

        Eigen::Quaternion<double> q =  yawAngle *pitchAngle * rollAngle;

        return q.matrix();
    }

    class articulated_system
    {   private :
        std::vector<Body> bodies;
        Eigen::VectorXd gc_, gv_;

        public :
        // Mesh_info mesh;
        articulated_system() : bodies(9), gc_(7), gv_(7)
        {   
            gc_.setZero();
            gv_.setZero();
            // mesh.init();

            //gen3 link0 (base link)
            bodies[0].pos_com << -0.000648, -0.000166, 0.084487;
            bodies[0].joint_pos << 0,0,0;
            bodies[0].joint_rot = Eigen::Matrix3d::Identity();
            bodies[0].mass = 1.697; 
            bodies[0].inertia << 0.004622, 9E-06, 6E-05,
                                9E-06, 0.004495, 9E-06,
                                6E-05, 9E-06, 0.002079;

            //gen3 link1 (shoulder link)
            bodies[1].pos_com << -2.3E-05, -0.010364, -0.07336;
            bodies[1].joint_pos <<0,0,0.15643;
            bodies[1].joint_rot = RPY2RotMatrix(3.1416, 2.7629E-18, -4.9305E-36);
            bodies[1].joint_axis = Eigen::Vector3d::UnitZ();
            bodies[1].mass = 1.3773;
            bodies[1].inertia << 0.00457, 1E-06, 2E-06,
                                1E-06, 0.004831, 0.000448,
                                2E-06, 0.000448, 0.001409;
            bodies[1].parent_body_index=0;

            //gen3 link2 (HalfArm1 link)
            bodies[2].pos_com << -4.4E-05, -0.09958, -0.013278;
            bodies[2].joint_pos <<  0, 0.005375, -0.12838;
            bodies[2].joint_rot = RPY2RotMatrix(1.5708, 2.1343E-17, -1.1102E-16);
            bodies[2].joint_axis = Eigen::Vector3d::UnitZ();
            bodies[2].mass = 1.1636;
            bodies[2].inertia << 0.011088, 5E-06, 0,
                                5E-06, 0.001072, -0.000691,
                                0, -0.000691, 0.011255;
            bodies[2].parent_body_index=1;

            //gen3 link3 (HalfArm2 link)
            bodies[3].pos_com << -4.4E-05, -0.006641, -0.117892;
            bodies[3].joint_pos << 0, -0.21038, -0.006375;
            bodies[3].joint_rot = RPY2RotMatrix(-1.5708, 1.2326E-32, -2.9122E-16);
            bodies[3].joint_axis = Eigen::Vector3d::UnitZ();
            bodies[3].mass = 1.1636;
            bodies[3].inertia << 0.010932, 0, -7E-06,
                                 0, 0.011127, 0.000606,
                                -7E-06, 0.000606, 0.001043;
            bodies[3].parent_body_index=2;

            //gen3 link4 (ForeArm link)
            bodies[4].pos_com << -1.8E-05, -0.075478, -0.015006;
            bodies[4].joint_pos << 0, 0.006375, -0.21038;
            bodies[4].joint_rot = RPY2RotMatrix(1.5708, -6.6954E-17, -1.6653E-16);
            bodies[4].joint_axis = Eigen::Vector3d::UnitZ();
            bodies[4].mass = 0.9302;
            bodies[4].inertia << 0.008147, -1E-06, 0,
                                -1E-06, 0.000631, -0.0005,
                                0, -0.0005, 0.008316;
            bodies[4].parent_body_index=3;

            //gen3 link5 (SphericalWrist1 link)
            bodies[5].pos_com << 1E-06, -0.009432, -0.063883;
            bodies[5].joint_pos << 0, -0.20843, -0.006375;
            bodies[5].joint_rot = RPY2RotMatrix(-1.5708, 2.2204E-16, -6.373E-17);
            bodies[5].joint_axis = Eigen::Vector3d::UnitZ();
            bodies[5].mass = 0.6781;
            bodies[5].inertia << 0.001596, 0, 0,
                                0, 0.001607, 0.000256,
                                0, 0.000256, 0.000399;
            bodies[5].parent_body_index=4;

            //gen3 link6 (SphericalWrist2 link)
            bodies[6].pos_com << 1E-06, -0.045483, -0.00965;
            bodies[6].joint_pos << 0, 0.00017505, -0.10593;
            bodies[6].joint_rot = RPY2RotMatrix(1.5708, 9.2076E-28, -8.2157E-15);
            bodies[6].joint_axis = Eigen::Vector3d::UnitZ();
            bodies[6].mass = 0.6781;
            bodies[6].inertia << 0.001641, 0, 0,
                                0, 0.00041, -0.000278,
                                0,-0.000278, 0.001641;
            bodies[6].parent_body_index=5;

            //gen3 link7 (Bracelet link)
            bodies[7].pos_com << -0.000281, -0.011402, -0.029798;
            bodies[7].joint_pos << 0, -0.10593, -0.00017505;
            bodies[7].joint_rot = RPY2RotMatrix(-1.5708, -5.5511E-17, 9.6396E-17);
            bodies[7].joint_axis = Eigen::Vector3d::UnitZ();
            bodies[7].mass = 0.5006;
            bodies[7].inertia << 0.000587, 3E-06, 3E-06,
                                3E-06, 0.000369, 0.000118,
                                3E-06, 0.000118, 0.000609;
            bodies[7].parent_body_index=6;

            //panda end-effector
            bodies[8].joint_pos<< 0,0,-0.0615250000000001;
            bodies[8].joint_rot = RPY2RotMatrix(-1.571, 0, 0);
            bodies[8].pos_com << 0,0,0;
            bodies[8].joint_axis = Eigen::Vector3d::UnitZ();
            bodies[8].mass = 0;
            bodies[8].inertia << 0,0,0,0,0,0,0,0,0;
            bodies[8].parent_body_index=7;
        }

        void UpdateState(const Eigen::VectorXd& gc, const Eigen::VectorXd& gv)
        {   
            gc_ = gc; 
            gv_ = gv; 
            bodies[0].pos_w << 0,0,0;
            bodies[0].rot_w = Eigen::Matrix3d::Identity();
            bodies[0].inertia_w = bodies[0].rot_w * bodies[0].inertia * bodies[0].rot_w.transpose();
            bodies[0].poscom_w = bodies[0].pos_w + bodies[0].rot_w*bodies[0].pos_com;
            bodies[0].ang_vel_w << 0,0,0;
            
            for(int i=1; i<bodies.size(); i++)
            {   
                if(i==bodies.size()-1)
                {
                    bodies[i].pos_w = bodies[bodies[i].parent_body_index].pos_w + bodies[bodies[i].parent_body_index].rot_w * bodies[i].joint_pos;
                    bodies[i].rot_b = Eigen::AngleAxisd(0,bodies[i].joint_axis);
                    bodies[i].rot_w = bodies[bodies[i].parent_body_index].rot_w *bodies[i].joint_rot*bodies[i].rot_b;
                    bodies[i].poscom_w = bodies[i].pos_w + bodies[i].rot_w*bodies[i].pos_com;
                    bodies[i].axis_w = bodies[i].rot_w*bodies[i].joint_axis;
                    bodies[i].inertia_w = bodies[i].rot_w * bodies[i].inertia * bodies[i].rot_w.transpose();
                    bodies[i].ang_vel_w = bodies[bodies[i].parent_body_index].ang_vel_w;
                }
                else
                {
                    bodies[i].pos_w = bodies[bodies[i].parent_body_index].pos_w + bodies[bodies[i].parent_body_index].rot_w * bodies[i].joint_pos;
                    bodies[i].rot_b = Eigen::AngleAxisd(gc[i-1],bodies[i].joint_axis);
                    bodies[i].rot_w = bodies[bodies[i].parent_body_index].rot_w *bodies[i].joint_rot*bodies[i].rot_b;
                    bodies[i].poscom_w = bodies[i].pos_w + bodies[i].rot_w*bodies[i].pos_com;
                    bodies[i].axis_w = bodies[i].rot_w*bodies[i].joint_axis;
                    bodies[i].inertia_w = bodies[i].rot_w * bodies[i].inertia * bodies[i].rot_w.transpose();
                    bodies[i].ang_vel_w = bodies[bodies[i].parent_body_index].ang_vel_w + bodies[i].axis_w*gv(i-1);
                }
                
            }
            
        }

        Eigen::VectorXd GetBaseWrench_RNEA(const Eigen::VectorXd &gacc)
        {
            //forward iteration to calculate the lin&ang acc of bodies
            std::vector<Eigen::Vector3d> lin_acc(8);
            std::vector<Eigen::Vector3d> ang_acc(8);
            //fixed base
            lin_acc[0] << 0,0,0;
            ang_acc[0] << 0,0,0;

            //panda link 1
            lin_acc[1] << 0,0,0;
            ang_acc[1] = skew(bodies[1].ang_vel_w)*bodies[1].axis_w*gv_(0)+bodies[1].axis_w*gacc(0);

            for(int i=2; i<8; i++)
            {
                lin_acc[i]=lin_acc[i-1]+skew(ang_acc[i-1])*(bodies[i].pos_w-bodies[i-1].pos_w)
                                +skew(bodies[i-1].ang_vel_w)*skew(bodies[i-1].ang_vel_w)*(bodies[i].pos_w-bodies[i-1].pos_w);

                ang_acc[i]=ang_acc[i-1] + skew(bodies[i].ang_vel_w)*bodies[i].axis_w*gv_(i-1)+bodies[i].axis_w*gacc(i-1);

            }

            //backward iteration to calculate the wrench
            std::vector<Eigen::Vector3d> force(8);
            std::vector<Eigen::Vector3d> moment(8);

            Eigen::Vector3d gravity_acc;
            gravity_acc<< 0,0,-9.81;
            
            
            force[7] = - bodies[7].mass*gravity_acc + bodies[7].mass*Eigen::Matrix3d::Identity()*lin_acc[7] 
                            -bodies[7].mass*skew(bodies[7].poscom_w - bodies[7].pos_w)*ang_acc[7]
                            +bodies[7].mass*skew(bodies[7].ang_vel_w)*skew(bodies[7].ang_vel_w)*(bodies[7].poscom_w - bodies[7].pos_w);

            moment[7] = - skew(bodies[7].poscom_w - bodies[7].pos_w)*bodies[7].mass*gravity_acc
                        +bodies[7].mass*skew(bodies[7].poscom_w-bodies[7].pos_w)*lin_acc[7]
                        +(bodies[7].inertia_w-bodies[7].mass*skew(bodies[7].poscom_w - bodies[7].pos_w)*skew(bodies[7].poscom_w - bodies[7].pos_w))*ang_acc[7]
                        +skew(bodies[7].ang_vel_w)*(bodies[7].inertia_w-bodies[7].mass*skew(bodies[7].poscom_w - bodies[7].pos_w)*skew(bodies[7].poscom_w - bodies[7].pos_w))*bodies[7].ang_vel_w;

            for(int i=6; i>0; i--)
            {
                force[i] = force[i+1] - bodies[i].mass*gravity_acc + bodies[i].mass*Eigen::Matrix3d::Identity()*lin_acc[i] 
                                -bodies[i].mass*skew(bodies[i].poscom_w - bodies[i].pos_w)*ang_acc[i]
                                +bodies[i].mass*skew(bodies[i].ang_vel_w)*skew(bodies[i].ang_vel_w)*(bodies[i].poscom_w - bodies[i].pos_w);

                moment[i] = moment[i+1] + skew(bodies[i+1].pos_w - bodies[i].pos_w)*force[i+1] - skew(bodies[i].poscom_w - bodies[i].pos_w)*bodies[i].mass*gravity_acc
                            +bodies[i].mass*skew(bodies[i].poscom_w-bodies[i].pos_w)*lin_acc[i]
                            +(bodies[i].inertia_w-bodies[i].mass*skew(bodies[i].poscom_w - bodies[i].pos_w)*skew(bodies[i].poscom_w - bodies[i].pos_w))*ang_acc[i]
                            +skew(bodies[i].ang_vel_w)*(bodies[i].inertia_w-bodies[i].mass*skew(bodies[i].poscom_w - bodies[i].pos_w)*skew(bodies[i].poscom_w - bodies[i].pos_w))*bodies[i].ang_vel_w;
            }

            force[0] = force[1] - bodies[0].mass*gravity_acc;
            moment[0] = moment[1] + skew(bodies[1].pos_w - bodies[0].pos_w)*force[1] - skew(bodies[0].poscom_w - bodies[0].pos_w)*bodies[0].mass*gravity_acc
                        +bodies[0].mass*skew(bodies[0].poscom_w-bodies[0].pos_w)*lin_acc[0]
                        +(bodies[0].inertia_w-bodies[0].mass*skew(bodies[0].poscom_w - bodies[0].pos_w)*skew(bodies[0].poscom_w - bodies[0].pos_w))*ang_acc[0];


            Eigen::VectorXd generalized_force(7), base_wrench(6);

            for(int i=0; i<7; i++)
            {
                generalized_force(i) = bodies[i+1].axis_w.transpose()*moment[i+1];
            }

            base_wrench << force[0],moment[0];

            return base_wrench;
        }
    };
}

#endif