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

using namespace KDL;
using namespace ti5rcl;
using namespace std;


bool ti5Robot::linear_move(const Frame *end_pos)
{
    //获取所有关节角度
    JntArray qNow(_nrOfJoints);
    int32_t c;
    double v;
    for (int i = 0; i < _nrOfJoints; i++)
    {

        _joint[i]->quickGetCSP(&c,&v,&qNow(i));
        qNow(i)=0;
#warning
    }
    //求末端位姿
    try
    {
        Frame frameNow;
        ChainFkSolverPos_recursive fwdkin(_chain);
        fwdkin.JntToCart(qNow,frameNow);
        tlog_info << "frameNow: " << frameNow.p.x() << "," << frameNow.p.y() << "," << frameNow.p.z() << endl;



//Path
//Path_Circle
//Path_Composite
//Path_Cyclic_Closed
//Path_Line
//Path_Point
//Path_RoundedComposite


        Path_Line* path = new Path_Line(frameNow,*end_pos,new RotationalInterpolation_SingleAxis(),0.2);

        // Trajectory defines a motion of the robot along a path.
        // This defines a trapezoidal velocity profile.
        VelocityProfile* velpref = new VelocityProfile_Trap(0.5,0.1);
        velpref->SetProfile(0,path->PathLength());
        Trajectory* traject = new Trajectory_Segment(path, velpref);
        Trajectory_Composite* ctraject = new Trajectory_Composite();

        // use the trajectory
        double dt=0.1;
        std::ofstream of("./trajectory.dat");
        for (double t=0.0; t <= traject->Duration(); t+= dt)
        {
            Frame current_pose;
            current_pose = traject->Pos(t);
            for (int i=0; i<4; ++i)
                for (int j=0; j<4; ++j)
                    of << current_pose(i,j) << "\t";
            of << "\n";
            // also velocities and accelerations are available !
            //traject->Vel(t);
            //traject->Acc(t);
        }
        of.close();


    }
    catch(Error& error)
    {
        std::cout <<"I encountered this error : " << error.Description() << std::endl;
        std::cout << "with the following type " << error.GetType() << std::endl;
    }

    return true;
}


bool ti5Robot::drag_mode_enable(BOOL enable)
{
    

}