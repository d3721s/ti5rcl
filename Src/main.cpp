#include "ti5mcl.hpp"
#include "ti5rcl.hpp"
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
using namespace std;
using namespace ti5rcl;
using namespace ti5mcl;

int main(int argc,char** argv){
    Frame x;
//    R1._joint[0]->home();
//    R1._joint[1]->home();
ti5Robot R1{};
    R1.linear_move(&x);
    R1.drag_mode_enable(true);
    this_thread::sleep_for(chrono::seconds(1));
}
