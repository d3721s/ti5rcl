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

using namespace KDL;
using namespace ti5rcl;
using namespace std;


bool ti5Robot::linear_move(const Frame *end_pos)
{
    //获取所有关节角度
    try
    {
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
    Frame frameNow;
    ChainFkSolverPos_recursive fwdkin(_chain);
    fwdkin.JntToCart(qNow,frameNow);
    tlog_info << "frameNow: " << frameNow.p.x() << "," << frameNow.p.y() << "," << frameNow.p.z() << endl;

    //开始插补
    Path_Line* path = new Path_Line(frameNow,*end_pos,new RotationalInterpolation_SingleAxis(),0.2);

    //验证插补
        for (double t = 0.0; t <= 1.0; t += 0.01) {
        Frame pos = path->Pos(t);
        tlog_info << "At time " << t << ", position is (" << pos.p.x() << ", "
                  << pos.p.y() << ", " << pos.p.z() << ") and rotation is ("
                  << pos.M.data[0] << ", " << pos.M.data[1] << ", " << pos.M.data[2]
                  << ", " << pos.M.data[3] << ")" << endl;
    }
//Path
//Path_Circle
//Path_Composite
//Path_Cyclic_Closed
//Path_Line
//Path_Point
//Path_RoundedComposite
    //依次解算

    //依次运动
    }
    catch (exception& e)
    {

    }


    return true;
}


