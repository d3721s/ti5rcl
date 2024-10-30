#include "ti5rcl.hpp"
#include "ti5mcl.hpp"
#include <kdl/kdl.hpp>
#include <kdl/chain.hpp>
#include <kdl/tree.hpp>
#include <kdl/segment.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainiksolver.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/path.hpp>
#include <kdl/path_line.hpp>
#include <kdl/path_circle.hpp>
#include <kdl/path_composite.hpp>
#include <kdl/path_cyclic_closed.hpp>
#include <kdl/path_roundedcomposite.hpp>
#include <kdl/path_point.hpp>
#include <kdl/rotational_interpolation_sa.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/frames.hpp>
#include <stdio.h>
#include <iostream>
#include <kdl/trajectory.hpp>
#include <kdl/trajectory_segment.hpp>
#include <kdl/trajectory_stationary.hpp>
#include <kdl/trajectory_composite.hpp>
#include <kdl/trajectory_composite.hpp>
#include <kdl/velocityprofile_trap.hpp>
#include <kdl/path_roundedcomposite.hpp>
#include <kdl/rotational_interpolation_sa.hpp>
#include <kdl/utilities/error.h>
#include <kdl/utilities/utility.h>
#include <kdl/trajectory_composite.hpp>

#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/parsers/urdf.hpp"
using namespace pinocchio;
using namespace KDL;
using namespace ti5rcl;
using namespace std;

bool ti5Robot::linear_move(const KDL::Frame *end_pos)
{
    tlog_info << "endpos: "<< end_pos->p.x() << "," << end_pos->p.y() << "," << end_pos->p.z() << endl;
    // 获取所有关节角度
    JntArray qNow(_nrOfJoints);
    int32_t c;
    double v;
    for (int i = 0; i < _nrOfJoints; i++)
    {
        _joint[i]->quickGetCSP(&c, &v, &qNow(i));
    }
    // 求末端位姿
    try
    {
        KDL::Frame frameNow;
        ChainFkSolverPos_recursive fwdkin(_chain);
        fwdkin.JntToCart(qNow, frameNow);
        tlog_info << "frameNow: " << frameNow.p.x() << "," << frameNow.p.y() << "," << frameNow.p.z() << endl;
        Path_Line *path = new Path_Line(frameNow, *end_pos, new RotationalInterpolation_SingleAxis(), 0.2);
        VelocityProfile *velpref = new VelocityProfile_Trap(0.5, 0.1);
        velpref->SetProfile(0, path->PathLength());
        Trajectory *traject = new Trajectory_Segment(path, velpref);
        Trajectory_Composite *ctraject = new Trajectory_Composite();
        double dt = 0.1;
        std::ofstream of("./trajectory.dat");
        for (double t = 0.0; t <= traject->Duration(); t += dt)
        {
            KDL::Frame current_pose;
            current_pose = traject->Pos(t);
            for (int i = 0; i < 4; ++i)
                for (int j = 0; j < 4; ++j)
                {
                    of << current_pose(i, j) << "\t";
                    cout <<current_pose(i, j) << "\t";
                }
            of << "\n";
            cout << "\n";

        }
        of.close();
    }
    catch (Error &error)
    {

        std::cout << "I encountered this error : " << error.Description() << std::endl;
        std::cout << "with the following type " << error.GetType() << std::endl;
    }

    return true;
}

bool ti5Robot::drag_mode_enable(bool enable)
{
    Model model;
    pinocchio::urdf::buildModel(_urdfPath, model);

    // Build a data frame associated with the model
    Data data(model);

    // Sample a random joint configuration, joint velocities and accelerations
    Eigen::VectorXd q = Eigen::VectorXd::Zero(model.nv); // in rad for the UR5
    Eigen::VectorXd v = Eigen::VectorXd::Zero(model.nv); // in rad/s for the UR5
    Eigen::VectorXd a = Eigen::VectorXd::Zero(model.nv); // in rad/s² for the UR5

    // Computes the inverse dynamics (RNEA) for all the joints of the robot
    Eigen::VectorXd tau = pinocchio::rnea(model, data, q, v, a);
    double nm_a[5]= {0.096,0.096,0.089,0.089,0.05};
    for (auto x=0; x<model.nv; x++)
    {
        tlog_info << "Joint positions: " << x << ": " << tau[x] << std::endl;
        tlog_info << "Joint current: " << x << ": " << tau[x]/nm_a[x] << std::endl;

    }

    // Print out to the vector of joint torques (in N.m)
    tlog_info << "Joint torques: " << data.tau.transpose() << std::endl;

    return true;
}
