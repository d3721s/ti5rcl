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

//    double x,y,z;
//    printf("Enter the x: ");
//    scanf("%lf",&x);
//    printf("Enter the y: ");
//    scanf("%lf",&y);
//    printf("Enter the z: ");
//    scanf("%lf",&z);
//    Vector vector = Vector(x,y,z);
//
//    float roll,pitch,yaw;
//    printf("Enter the roll: ");
//    scanf("%e",&roll);
//    printf("Enter the pitch: ");
//    scanf("%e",&pitch);
//    printf("Enter the yaw: ");
//    scanf("%e",&yaw);
//    float cy = cos(yaw);
//    float sy = sin(yaw);
//    float cp = cos(pitch);
//    float sp = sin(pitch);
//    float cr = cos(roll);
//    float sr = sin(roll);
//
//    double rot0 = cy*cp;
//    double rot1 = cy*sp*sr - sy*cr;
//    double rot2 = cy*sp*cr + sy*sr;
//    double rot3 = sy*cp;
//    double rot4 = sy*sp*sr + cy*cr;
//    double rot5 = sy*sp*cr - cy*sr;
//    double rot6 = -sp;
//    double rot7 = cp*sr;
//    double rot8 = cp*cr;
//    Rotation rot = Rotation(rot0,rot1,rot2,rot3,rot4,rot5,rot6,rot7,rot8);
//
//    Frame cartpos = Frame(rot,vector);


    // 设置目标位置（x, y, z）
    Frame cartpos(KDL::Rotation::Identity(), KDL::Vector(-0.6026, 0, 0));
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
///**
// * \file path_example.cpp
// * An example to demonstrate the use of trajectory generation
// * functions.
// *
// * There are is a matlab/octave file in the examples directory to visualise the results
// * of this example program. (visualize_trajectory.m)
// *
// */
//
//#include <kdl/frames.hpp>
//#include <kdl/frames_io.hpp>
//#include <kdl/trajectory.hpp>
//#include <kdl/trajectory_segment.hpp>
//#include <kdl/trajectory_stationary.hpp>
//#include <kdl/trajectory_composite.hpp>
//#include <kdl/trajectory_composite.hpp>
//#include <kdl/velocityprofile_trap.hpp>
//#include <kdl/path_roundedcomposite.hpp>
//#include <kdl/rotational_interpolation_sa.hpp>
//#include <kdl/utilities/error.h>
//#include <kdl/utilities/utility.h>
//#include <kdl/trajectory_composite.hpp>
//
//int main(int argc,char* argv[]) {
//	using namespace KDL;
//	// Create the trajectory:
//    // use try/catch to catch any exceptions thrown.
//    // NOTE:  exceptions will become obsolete in a future version.
//	try {
//        // Path_RoundedComposite defines the geometric path along
//        // which the robot will move.
//		//
//		Path_RoundedComposite* path = new Path_RoundedComposite(0.2,0.01,new RotationalInterpolation_SingleAxis());
//		// The routines are now robust against segments that are parallel.
//		// When the routines are parallel, no rounding is needed, and no attempt is made
//		// add constructing a rounding arc.
//		// (It is still not possible when the segments are on top of each other)
//		// Note that you can only rotate in a deterministic way over an angle less then PI!
//		// With an angle == PI, you cannot predict over which side will be rotated.
//		// With an angle > PI, the routine will rotate over 2*PI-angle.
//		// If you need to rotate over a larger angle, you need to introduce intermediate points.
//		// So, there is a common use case for using parallel segments.
//		path->Add(Frame(Rotation::RPY(PI,0,0), Vector(-1,0,0)));
////		path->Add(Frame(Rotation::RPY(M_PI_2,0,0), Vector(-0.5,0,0)));
////		path->Add(Frame(Rotation::RPY(0,0,0), Vector(0,0,0)));
////		path->Add(Frame(Rotation::RPY(0.7,0.7,0.7), Vector(1,1,1)));
////		path->Add(Frame(Rotation::RPY(0,0.7,0), Vector(1.5,0.3,0)));
////		path->Add(Frame(Rotation::RPY(0.7,0.7,0), Vector(1,1,0)));
//
//		// always call Finish() at the end, otherwise the last segment will not be added.
//		path->Finish();
//
//        // Trajectory defines a motion of the robot along a path.
//        // This defines a trapezoidal velocity profile.
//		VelocityProfile* velpref = new VelocityProfile_Trap(0.5,0.1);
//		velpref->SetProfile(0,path->PathLength());
//		Trajectory* traject = new Trajectory_Segment(path, velpref);
//
//
//		Trajectory_Composite* ctraject = new Trajectory_Composite();
//		ctraject->Add(traject);
//		ctraject->Add(new Trajectory_Stationary(1.0,Frame(Rotation::RPY(0.7,0.7,0), Vector(1,1,0))));
//
//		// use the trajectory
//		double dt=0.1;
//		std::ofstream of("./trajectory.dat");
//		for (double t=0.0; t <= traject->Duration(); t+= dt) {
//			Frame current_pose;
//			current_pose = traject->Pos(t);
//			for (int i=0;i<4;++i)
//				for (int j=0;j<4;++j)
//					of << current_pose(i,j) << "\t";
//			of << "\n";
//			// also velocities and accelerations are available !
//			//traject->Vel(t);
//			//traject->Acc(t);
//		}
//		of.close();
//
//		// you can get some meta-info on the path:
//		for (int segmentnr=0;  segmentnr < path->GetNrOfSegments(); segmentnr++) {
//			double starts,ends;
//			Path::IdentifierType pathtype;
//			if (segmentnr==0) {
//				starts = 0.0;
//			} else {
//				starts = path->GetLengthToEndOfSegment(segmentnr-1);
//			}
//			ends = path->GetLengthToEndOfSegment(segmentnr);
//			pathtype = path->GetSegment(segmentnr)->getIdentifier();
//			std::cout << "segment " << segmentnr << " runs from s="<<starts << " to s=" <<ends;
//			switch(pathtype) {
//				case Path::ID_CIRCLE:
//					std::cout << " circle";
//					break;
//				case Path::ID_LINE:
//					std::cout << " line ";
//					break;
//				default:
//					std::cout << " unknown ";
//					break;
//			}
//			std::cout << std::endl;
//		}
//        std::cout << " trajectory written to the ./trajectory.dat file " << std::endl;
//
//        delete ctraject;
//	} catch(Error& error) {
//		std::cout <<"I encountered this error : " << error.Description() << std::endl;
//		std::cout << "with the following type " << error.GetType() << std::endl;
//	}
//
//}

