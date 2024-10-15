#ifndef TI5RCL_HPP_INCLUDED
#define TI5RCL_HPP_INCLUDED
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

#include "ti5mcl.hpp"

#ifdef _WINDOWS
#define DLLEXPORT_API __declspec(dllexport)
#else
#define DLLEXPORT_API
#endif


#define LOGLEVEL TLOG_DEBUG
#warning "LOGLEVEL-debug"

#ifndef LOGLEVEL
#define LOGLEVEL TLOG_WARN
#endif

namespace ti5rcl
{
using namespace std;
using namespace ti5mcl;
class DLLEXPORT_API ti5Robot
{
public: //机械臂基础
    ti5Robot()
    {

    }
    ~ti5Robot();
public: //机械臂运动
//	bool jog(int aj_num, MoveMode move_mode, CoordType coord_type, double vel_cmd, double pos_cmd);
//	bool jog_stop(int num);
//	bool joint_move(const JointValue *joint_pos, MoveMode move_mode, BOOL is_block, double speed, double acc = 90, double tol = 0, const OptionalCond *option_cond= nullptr);
//	bool joint_move(MoveJParam param);
//	bool linear_move(const CartesianPose *end_pos, MoveMode move_mode, BOOL is_block, double speed, double accel = 500, double tol = 0, const OptionalCond *option_cond = nullptr, double ori_vel=3.14, double ori_acc=12.56);
//	bool linear_move(MoveLParam param);
//	bool circular_move(const CartesianPose *end_pos, const CartesianPose *mid_pos, MoveMode move_mode, BOOL is_block, double speed, double accel, double tol, const OptionalCond *option_cond = nullptr, int circle_cnt = 0, int circle_mode = 0);
//	bool circular_move(MoveCParam param);
//	bool set_rapidrate(double rapid_rate);
//	bool get_rapidrate(double *rapid_rate);
//	bool set_user_frame_data(int id, const CartesianPose *user_frame, const char *name);
//	bool set_user_frame_id(const int id);
//	bool get_user_frame_id(int *id);
//	bool get_user_frame_data(int id, CartesianPose *tcp);
//	bool set_payload(const PayLoad *payload);
//	bool get_payload(PayLoad *payload);
//	bool get_tcp_position(CartesianPose *tcp_position);
//	bool get_joint_position(JointValue *joint_position);
//	bool is_in_estop(BOOL *estop);
//	bool is_on_limit(BOOL *on_limit);
//	bool is_in_pos(BOOL *in_pos);
//	bool motion_abort();
//	bool get_motion_status(MotionStatus *status);
public: //机械臂操作信息设置与获取
public: //机械臂安全状态设置与查询
public: //使用脚本
public: //机械臂轨迹复现
public: //机械臂伺服运动模式
public: //机械臂运动学
public: //扩展1



private:
    vector<ti5Motor> Joint;
public:






};


}
#endif // TI5RCL_HPP_INCLUDED
