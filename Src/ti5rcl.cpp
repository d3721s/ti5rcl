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
bool linear_move(const Frame *end_pos, MoveMode move_mode, BOOL is_block,
                 double speed, double accel = 500, double tol = 0, const OptionalCond *option_cond = nullptr,
                 double ori_vel=3.14, double ori_acc=12.56)
{



}
