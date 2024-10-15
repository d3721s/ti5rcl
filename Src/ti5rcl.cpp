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

ti5Robot R1;
bool ti5Robot::linear_move(const Frame *end_pos)
{
    //获取所有关节角度
    #warning TODO:改为类方法访问
    float joint_angle[6] = {0};
    uint32_t c;
    float v;
    for (int i = 0; i < 6; i++)
    {
    }
    //求末端位姿
    Frame current_frame;
    ChainFkSolverPos_recursive fwdkin(_chain);
//    fwdkin.JntToCart(JntArray(joint_angle),current_frame);
    //与目标Frame比较

    //开始插补

    //验证插补

    //依次解算

    //依次运动



}


