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
using namespace std;
int main(int argc,char** argv){
    Tree my_tree;
    kdl_parser::treeFromFile("/home/runyu/urdf/arm5.urdf",my_tree);

    bool exit_value;
    Chain chain;
    exit_value = my_tree.getChain("base_link","Empty_Link5",chain);
    double eps=1E-5;
    int maxiter=500;
    double eps_joints=1E-15;
    ChainIkSolverPos_LMA iksolver = ChainIkSolverPos_LMA(chain,eps,maxiter,eps_joints);

    unsigned int nj = chain.getNrOfJoints();
    JntArray jointGuesspositions = JntArray(nj);

    for(unsigned int i=0;i<nj;i++){
        float myinput;
        printf("Enter the initial guess position of joint %i: ",i);
        scanf("%e",&myinput);
        jointGuesspositions(i)=(double)myinput;
}

    double x,y,z;
    printf("Enter the x: ");
    scanf("%lf",&x);
    printf("Enter the y: ");
    scanf("%lf",&y);
    printf("Enter the z: ");
    scanf("%lf",&z);
    Vector vector = Vector(x,y,z);

    float roll,pitch,yaw;
    printf("Enter the roll: ");
    scanf("%e",&roll);
    printf("Enter the pitch: ");
    scanf("%e",&pitch);
    printf("Enter the yaw: ");
    scanf("%e",&yaw);
    float cy = cos(yaw);
    float sy = sin(yaw);
    float cp = cos(pitch);
    float sp = sin(pitch);
    float cr = cos(roll);
    float sr = sin(roll);
    double rot0 = cy*cp;
    double rot1 = cy*sp*sr - sy*cr;
    double rot2 = cy*sp*cr + sy*sr;
    double rot3 = sy*cp;
    double rot4 = sy*sp*sr + cy*cr;
    double rot5 = sy*sp*cr - cy*sr;
    double rot6 = -sp;
    double rot7 = cp*sr;
    double rot8 = cp*cr;
    Rotation rot = Rotation(rot0,rot1,rot2,rot3,rot4,rot5,rot6,rot7,rot8);

    Frame cartpos = Frame(rot,vector);
    JntArray jointpositions = JntArray(nj);

    bool kinematics_status;
    kinematics_status = iksolver.CartToJnt(jointGuesspositions,cartpos,jointpositions);
    if(kinematics_status>=0){
        for(int i=0;i<nj;i++){
            std::cout << jointpositions(i) << std::endl;
        }
        printf("%s \n","Success, thanks KDL!");
    }
    else{
        printf("%s \n","Error:could not calculate backword kinematics : ");
    }
}
//#include <kdl/kdl.hpp>
//#include <kdl/chain.hpp>
//#include <kdl/tree.hpp>
//#include <kdl/segment.hpp>
//#include <kdl/chainfksolver.hpp>
//#include <kdl_parser/kdl_parser.hpp>
//#include <kdl/chainfksolverpos_recursive.hpp>
//#include <kdl/frames_io.hpp>
//#include <stdio.h>
//#include <iostream>
//
//using namespace KDL;
//using namespace std;
//int main(int argc,char** argv){
//    Tree my_tree;
//    kdl_parser::treeFromFile("/home/runyu/urdf/arm5.urdf",my_tree);
//
//    bool exit_value;
//    Chain chain;
//    exit_value = my_tree.getChain("base_link","Empty_Link5",chain);
//    ChainFkSolverPos_recursive fksolver = ChainFkSolverPos_recursive(chain);
//
//    unsigned int nj = chain.getNrOfJoints();
//    JntArray jointpositions = JntArray(nj);
//
//    for(unsigned int i=0;i<nj;i++){
//        float myinput;
//        printf("Enter the position of joint %i: ",i);
//        scanf("%e",&myinput);
//        jointpositions(i)=(double)myinput;
//}
//
//    Frame cartpos;
//
//    bool kinematics_status;
//    kinematics_status = fksolver.JntToCart(jointpositions,cartpos);
//    if(kinematics_status>=0){
//        std::cout << cartpos << std::endl;
//        printf("%s \n","Success, thanks KDL!");
//    }
//    else{
//        printf("%s \n","Error:could not calculate forward kinematics : ");
//    }
//}
