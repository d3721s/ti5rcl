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

bool ti5Robot::linear_move(const Frame *end_pos)
{
    //获取所有关节角度
    JntArray q(_nrOfJoints);
    int32_t c;
    float v;

    for (int i = 0; i < 5; i++)
    {
        if (_joint[i] == nullptr)
        {
            tlog_error << "Error: _joint[" << i << "] is null." << endl;
        }
        else
        _joint[i]->quickGetCSP(&c,&v,&q(i));
    }
    //求末端位姿
    ChainFkSolverPos_recursive fwdkin(_chain);
    fwdkin.JntToCart(q,pos_goal);
    KDL::JntArray q_init(n);
    KDL::JntArray q_sol(n);
    //与目标Frame比较

    //开始插补

    //验证插补

    //依次解算

    //依次运动


    return true;
}


