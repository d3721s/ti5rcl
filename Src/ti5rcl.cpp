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
#include <kdl/frames_io.hpp>
#include <kdl/frames.hpp>
#include <stdio.h>
#include <iostream>

using namespace KDL;
using namespace ti5rcl;

KDL::Frame linearInterpolation(const KDL::Frame& start, const KDL::Frame& end, double t)
{
    KDL::Frame interpolatedPose;
    interpolatedPose.p = start.p + t * (end.p - start.p); // 位置插值
    interpolatedPose.M = start.M * (1.0 - t) + end.M * t; // 方向插值（这里是简单的加权平均，可能需要更复杂的方法）

    return interpolatedPose;
}

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
    }

    //求末端位姿
    Frame frameNow;
    ChainFkSolverPos_recursive fwdkin(_chain);
    fwdkin.JntToCart(qNow,frameNow);
    tlog_info << "frameNow: " << frameNow.p.x() << "," << frameNow.p.y() << "," << frameNow.p.z() << endl;

    //开始插补
    linearInterpolation(q)
    //yici验证插补

    //依次解算

    //依次运动
    }
    catch (const Error& e)
    {

    }


    return true;
}


