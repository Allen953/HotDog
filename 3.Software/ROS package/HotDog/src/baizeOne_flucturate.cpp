#include <stdio.h>
#include <iostream>
#include <string>
#include <vector>
 
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "turtlesim/Pose.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2/LinearMath/Quaternion.h"
#include "kdl_parser/kdl_parser.hpp"
#include "kdl/chainiksolverpos_nr_jl.hpp"
#include "trac_ik/trac_ik.hpp"
#include "urdf/model.h"
#include <math.h>
#include <serial/serial.h>
#include "geometry_msgs/PointStamped.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "robot_msg/quadrupedrobot_jointstate.h"

#define pi 3.141592653

int main(int argc,char* argv[])
{

    setlocale(LC_ALL,"");
    ros::init(argc,argv,"kinetic_robot");
    ros::NodeHandle nh;
    ros::Publisher joint_pub=nh.advertise<sensor_msgs::JointState>("joint_states",1);
    tf2_ros::StaticTransformBroadcaster broadcaster;
 
    geometry_msgs::TransformStamped ts;
    KDL::Vector v1(1,1,1);
 
    ros::Publisher quadruped_jointpub=nh.advertise<robot_msg::quadrupedrobot_jointstate>("quadruped_joint",1);
    robot_msg::quadrupedrobot_jointstate quadruped_joint;
    KDL::Tree my_tree;
    sensor_msgs::JointState joint_state;
 
    std::string robot_desc_string;
    nh.param("robot_description", robot_desc_string, std::string());
 
    if(!kdl_parser::treeFromString(robot_desc_string, my_tree))
    //if(!kdl_parser::treeFromFile("/home/zhitong/catkin_ws3/src/LIUZU/urdf/LIUZU.urdf", my_tree))
    {
        ROS_ERROR("Failed to construct kdl tree");
    }
    else
    {
        ROS_INFO("成功生成kdl树!");
    }
 
    std::vector<std::string> joint_name = {
  "RF_JIAN_JOINT", "RF_THIGH_JOINT", "RF_SHANK_JOINT", "RF_R_JOINT","RF_P_JOINT", "RF_Y_JOINT",
"LF_JIAN_JOINT", "LF_THIGH_JOINT", "LF_SHANK_JOINT", "LF_R_JOINT","LF_P_JOINT", "LF_Y_JOINT",
"LB_JIAN_JOINT", "LB_THIGH_JOINT", "LB_SHANK_JOINT", "LB_R_JOINT","LB_P_JOINT", "LB_Y_JOINT",
"RB_JIAN_JOINT", "RB_THIGH_JOINT", "RB_SHANK_JOINT", "RB_R_JOINT","RB_P_JOINT", "RB_Y_JOINT"
};
 
    std::vector<double> joint_pos = {
0,0,0,0,0,0,
0,0,0,0,0,0,
0,0,0,0,0,0,
0,0,0,0,0,0
};
    std::string urdf_param = "/robot_description";
    double timeout = 0.005;
    double eps = 1e-5;
    std::string chain_start  = "BODY_LINK"; 
    std::string chain_rf_end = "RF_Y_LINK"; 
    std::string chain_lf_end = "LF_Y_LINK"; 
    std::string chain_lb_end = "LB_Y_LINK"; 
    std::string chain_rb_end = "RB_Y_LINK"; 

    //逆运动学求解器
    TRAC_IK::TRAC_IK tracik_rf_solver(chain_start, chain_rf_end, urdf_param, timeout, eps);
    TRAC_IK::TRAC_IK tracik_lf_solver(chain_start, chain_lf_end, urdf_param, timeout, eps);
    TRAC_IK::TRAC_IK tracik_lb_solver(chain_start, chain_lb_end, urdf_param, timeout, eps);
    TRAC_IK::TRAC_IK tracik_rb_solver(chain_start, chain_rb_end, urdf_param, timeout, eps);

    KDL::Chain chain_rf;
    KDL::Chain chain_lf;
    KDL::Chain chain_lb;
    KDL::Chain chain_rb;


    KDL::JntArray ll, ul; //关节下限, 关节上限
    bool valid = tracik_rf_solver.getKDLChain(chain_rf);
    if(!valid)
    {
        ROS_ERROR("There was no valid KDL chain found on rf link chain");
    }
    valid = tracik_rf_solver.getKDLLimits(ll, ul);
    if(!valid)
    {
        ROS_ERROR("There was no valid KDL joint limits found on rf link chain");
    }
    valid = tracik_lf_solver.getKDLChain(chain_lf);
    if(!valid)
    {
        ROS_ERROR("There was no valid KDL chain found on lf link chain");
    }
    valid = tracik_lf_solver.getKDLLimits(ll, ul);
    if(!valid)
    {
        ROS_ERROR("There was no valid KDL joint limits found on lf link chain");
    }
    valid = tracik_lb_solver.getKDLChain(chain_lb);
    if(!valid)
    {
        ROS_ERROR("There was no valid KDL chain found on lblink chain");
    }
    valid = tracik_lb_solver.getKDLLimits(ll, ul);
    if(!valid)
    {
        ROS_ERROR("There was no valid KDL joint limits found on lb link chain");
    }
    valid = tracik_rb_solver.getKDLChain(chain_rb);
    if(!valid)
    {
        ROS_ERROR("There was no valid KDL chain found on rblink chain");
    }
    valid = tracik_rb_solver.getKDLLimits(ll, ul);
    if(!valid)
    {
        ROS_ERROR("There was no valid KDL joint limits found on rb link chain");
    }

    //正运动学求解器
    KDL::ChainFkSolverPos_recursive fk_solver_rf(chain_rf);
    KDL::ChainFkSolverPos_recursive fk_solver_lf(chain_lf);
    KDL::ChainFkSolverPos_recursive fk_solver_lb(chain_lb);
    KDL::ChainFkSolverPos_recursive fk_solver_rb(chain_rb);

    ROS_INFO("rf关节数量: %d", chain_rf.getNrOfJoints());
    ROS_INFO("rf关节数量: %d", chain_lf.getNrOfJoints());
    ROS_INFO("rf关节数量: %d", chain_lb.getNrOfJoints());
    ROS_INFO("rf关节数量: %d", chain_rb.getNrOfJoints());


    KDL::JntArray nominal(24);
 
    ROS_INFO("the nominal size is:%d",nominal.data.size());
 
    for(size_t j = 0; j < 24; j ++)
    {
        nominal(j)=0.0;
        //nominal(j) = (ll(j) + ul(j))/2.0;
    }
    
     //定义初始状态末端点齐次矩阵
    KDL::Frame end_effector_pose_rfstart;//start
    KDL::Frame end_effector_pose_lfstart;//start
    KDL::Frame end_effector_pose_lbstart;//start
    KDL::Frame end_effector_pose_rbstart;//start

    KDL::Frame end_effector_pose_rf_now;//now
    KDL::Frame end_effector_pose_lf_now;//now
    KDL::Frame end_effector_pose_lb_now;//now
    KDL::Frame end_effector_pose_rb_now;//now
    //定义逆运动学解算结果存储数组
    KDL::JntArray result_rflast(6);//last time
    KDL::JntArray result_lflast(6);//last time
    KDL::JntArray result_lblast(6);//last time
    KDL::JntArray result_rblast(6);//last time

    KDL::JntArray result_rfnow(6);//now
    KDL::JntArray result_lfnow(6);//now
    KDL::JntArray result_lbnow(6);//now
    KDL::JntArray result_rbnow(6);//now


    result_rflast(0)=0;
    result_rflast(1)=-pi/4;
    result_rflast(2)=-0.2;
    result_rflast(3)=0;
    result_rflast(4)=0;
    result_rflast(5)=0;

    result_lflast(0)=0;
    result_lflast(1)=-pi/4;
    result_lflast(2)=-0.2;
    result_lflast(3)=0;
    result_lflast(4)=0;
    result_lflast(5)=0;

    result_lblast(0)=0;
    result_lblast(1)=pi/4;
    result_lblast(2)=-0.2;
    result_lblast(3)=0;
    result_lblast(4)=0;
    result_lblast(5)=0;

    result_rblast(0)=0;
    result_rblast(1)=-pi/4;
    result_rblast(2)=-0.2;
    result_rblast(3)=0;
    result_rblast(4)=0;
    result_rblast(5)=0;

    ros::Rate r(5);
 
    auto print_frame_lambda = [](KDL::Frame f)
    {
        double x, y, z, roll, pitch, yaw;
        x = f.p.x();
        y = f.p.y();
        z = f.p.z();
        f.M.GetRPY(roll, pitch, yaw);
        std::cout << "x:" << x << " y:" << y << " z:" << z << " roll:" << roll << " pitch:" << pitch << " yaw:" << yaw << std::endl;
    };
    
    //正运动学
    fk_solver_rf.JntToCart(result_rflast,end_effector_pose_rfstart);//RF
    fk_solver_lf.JntToCart(result_lflast,end_effector_pose_lfstart);//LF
    fk_solver_lb.JntToCart(result_lblast,end_effector_pose_lbstart);//LB
    fk_solver_rb.JntToCart(result_rblast,end_effector_pose_rbstart);//RB
    //逆运动学
    int rc_rf = tracik_rf_solver.CartToJnt(result_rflast, end_effector_pose_rfstart, result_rfnow);//RF
    int rc_lf = tracik_lf_solver.CartToJnt(result_lflast, end_effector_pose_lfstart, result_lfnow);//LF
    int rc_lb = tracik_lb_solver.CartToJnt(result_lblast, end_effector_pose_lbstart, result_lbnow);//LB
    int rc_rb = tracik_rb_solver.CartToJnt(result_rblast, end_effector_pose_rbstart, result_rbnow);//RB

    ROS_INFO("result_rf    1:%f,2:%f,3:%f,4:%f,5:%f,6:%f",result_rfnow(0),result_rfnow(1),result_rfnow(2),result_rfnow(3),result_rfnow(4),result_rfnow(5));
    ROS_INFO("result_lf    1:%f,2:%f,3:%f,4:%f,5:%f,6:%f",result_lfnow(0),result_lfnow(1),result_lfnow(2),result_lfnow(3),result_lfnow(4),result_lfnow(5));
    ROS_INFO("result_lb    1:%f,2:%f,3:%f,4:%f,5:%f,6:%f",result_lbnow(0),result_lbnow(1),result_lbnow(2),result_lbnow(3),result_lbnow(4),result_lbnow(5));
    ROS_INFO("result_rb    1:%f,2:%f,3:%f,4:%f,5:%f,6:%f",result_rbnow(0),result_rbnow(1),result_rbnow(2),result_rbnow(3),result_rbnow(4),result_rbnow(5));

    print_frame_lambda(end_effector_pose_rfstart);
    print_frame_lambda(end_effector_pose_lfstart);
    print_frame_lambda(end_effector_pose_lbstart);
    print_frame_lambda(end_effector_pose_rbstart);


 
    ROS_INFO("更新关节状态");
    joint_state.header.stamp = ros::Time::now();
    
    joint_state.name.resize(36);
    joint_state.position.resize(36);
 
    for(size_t i = 0; i < 24; i ++)
    {
        joint_state.name[i] = joint_name[i];
        joint_state.position[i] = 0;
    }
 
    for(size_t i = 0; i < 6; i ++)
    {
        joint_state.position[i] = result_rfnow(i);
    }
    for(size_t i = 0; i < 6; i ++)
    {
        joint_state.position[i+6] = result_lfnow(i);
    }
    for(size_t i = 0; i < 6; i ++)
    {
        joint_state.position[i+12] = result_lbnow(i);
    }
    for(size_t i = 0; i < 6; i ++)
    {
        joint_state.position[i+18] = result_rbnow(i);
    }
    
    // while(ros::ok())
    // {
    //     joint_pub.publish(joint_state);
    //     r.sleep();
    // }
    

    //正运动学
    fk_solver_rf.JntToCart(result_rflast,end_effector_pose_rf_now);//RF
     fk_solver_lf.JntToCart(result_lflast,end_effector_pose_lf_now);//RF
    fk_solver_lb.JntToCart(result_lblast,end_effector_pose_lb_now);//RF
    fk_solver_rb.JntToCart(result_rblast,end_effector_pose_rb_now);//RF


    //数据存储
    FILE *fp;

    if( (fp=fopen("/home/c123/catkin_zhitong/src/file.txt","ab"))==NULL )
    {
        printf("cannot open file");
        return 0;
    }

    while(ros::ok())
    {
    //x=0.02*(t-sint);
    //y=0.02*(1-cost);
    for(int i=1;i<=20;i++)
    { 
        double t=2*pi*i/20;
        double x=0.01*(t-sin(t));
        double z=0.01*(1-cos(t));
        //右边
        end_effector_pose_rf_now.p.data[0]=end_effector_pose_rfstart.p.data[0]-0.01*pi+x;
        end_effector_pose_rf_now.p.data[2]=end_effector_pose_rfstart.p.data[2]+z;
        int rc_rf = tracik_rf_solver.CartToJnt(result_rflast, end_effector_pose_rf_now, result_rfnow);
        print_frame_lambda(end_effector_pose_rf_now);

        end_effector_pose_lf_now.p.data[0]=end_effector_pose_lfstart.p.data[0]+0.01*pi-x;
        end_effector_pose_lf_now.p.data[2]=end_effector_pose_lfstart.p.data[2];
        int rc_lf = tracik_lf_solver.CartToJnt(result_lflast, end_effector_pose_lf_now, result_lfnow);

        end_effector_pose_lb_now.p.data[0]=end_effector_pose_lbstart.p.data[0]-0.01*pi+x;
        end_effector_pose_lb_now.p.data[2]=end_effector_pose_lbstart.p.data[2]+z;
        int rc_lb = tracik_lb_solver.CartToJnt(result_lblast, end_effector_pose_lb_now, result_lbnow);

        end_effector_pose_rb_now.p.data[0]=end_effector_pose_rbstart.p.data[0]+0.01*pi-x;;
        end_effector_pose_rb_now.p.data[2]=end_effector_pose_rbstart.p.data[2];
        int rc_rb= tracik_rb_solver.CartToJnt(result_rblast, end_effector_pose_rb_now, result_rbnow);


        joint_state.header.stamp = ros::Time::now();

        joint_state.position[0] = result_rfnow(0);
        joint_state.position[1] = result_rfnow(1);
        joint_state.position[2] = result_rfnow(2);

        joint_state.position[6] = result_lfnow(0);
        joint_state.position[7] = result_lfnow(1);
        joint_state.position[8] = result_lfnow(2);

        joint_state.position[12] = result_lbnow(0);
        joint_state.position[13] = result_lbnow(1);
        joint_state.position[14] = result_lbnow(2);

        joint_state.position[18] = result_rbnow(0);
        joint_state.position[19] = result_rbnow(1);
        joint_state.position[20] = result_rbnow(2);
	
        //将数据写入hexapod的arduino话题
        int m=0,n=0;
        // int n=0;
        for(int i=0;i<4;i++)
        {
            for(int j=0;j<3;j++)
            {
                quadruped_joint.position[m]= joint_state.position[n];
                //写入文件
                fprintf(fp,"%.2f",joint_state.position[n]*180.0/pi);
	            fputs(",",fp);
                m++;
                n++;
            }
            n=n+3;
        }
	fputs("\n",fp);
        
        joint_pub.publish(joint_state);
        quadruped_jointpub.publish(quadruped_joint);

 
        result_rflast=result_rfnow;
        result_lflast=result_lfnow;
        result_lblast=result_lbnow;
        result_rblast=result_rbnow;
        r.sleep();
    }
    
 
    for(int i=1;i<=20;i++)
    { 
        double t=2*pi*i/20;
        double x=0.01*(t-sin(t));
        double z=0.01*(1-cos(t));

        end_effector_pose_rf_now.p.data[0]=end_effector_pose_rfstart.p.data[0]+0.01*pi-x;
        end_effector_pose_rf_now.p.data[2]=end_effector_pose_rfstart.p.data[2];
        int rc_rf = tracik_rf_solver.CartToJnt(result_rflast, end_effector_pose_rf_now, result_rfnow);
        print_frame_lambda(end_effector_pose_rf_now);

        end_effector_pose_lf_now.p.data[0]=end_effector_pose_lfstart.p.data[0]-0.01*pi+x;
        end_effector_pose_lf_now.p.data[2]=end_effector_pose_lfstart.p.data[2]+z;
        int rc_lf = tracik_lf_solver.CartToJnt(result_lflast, end_effector_pose_lf_now, result_lfnow);

        end_effector_pose_lb_now.p.data[0]=end_effector_pose_lbstart.p.data[0]+0.01*pi-x;
        end_effector_pose_lb_now.p.data[2]=end_effector_pose_lbstart.p.data[2];
        int rc_lb = tracik_lb_solver.CartToJnt(result_lblast, end_effector_pose_lb_now, result_lbnow);

        end_effector_pose_rb_now.p.data[0]=end_effector_pose_rbstart.p.data[0]-0.01*pi+x;
        end_effector_pose_rb_now.p.data[2]=end_effector_pose_rbstart.p.data[2]+z;
        int rc_rb= tracik_rb_solver.CartToJnt(result_rblast, end_effector_pose_rb_now, result_rbnow);
 
        joint_state.header.stamp = ros::Time::now();

        joint_state.position[0] = result_rfnow(0);
        joint_state.position[1] = result_rfnow(1);
        joint_state.position[2] = result_rfnow(2);

        joint_state.position[6] = result_lfnow(0);
        joint_state.position[7] = result_lfnow(1);
        joint_state.position[8] = result_lfnow(2);

        joint_state.position[12] = result_lbnow(0);
        joint_state.position[13] = result_lbnow(1);
        joint_state.position[14] = result_lbnow(2);

        joint_state.position[18] = result_rbnow(0);
        joint_state.position[19] = result_rbnow(1);
        joint_state.position[20] = result_rbnow(2);

        //将数据写入hexapod的arduino话题
        int m=0,n=0;
        // int n=0;
        for(int i=0;i<4;i++)
        {
            for(int j=0;j<3;j++)
            {
                quadruped_joint.position[m]= joint_state.position[n];
                //写入文件
                fprintf(fp,"%.2f",joint_state.position[n]*180.0/pi);
	            fputs(",",fp);
                m++;
                n++;
            }
            n=n+3;
        }
	fputs("\n",fp);
        
        joint_pub.publish(joint_state);
        quadruped_jointpub.publish(quadruped_joint);
 
        result_rflast=result_rfnow;
        result_lflast=result_lfnow;
        result_lblast=result_lbnow;
        result_rblast=result_rbnow;
        r.sleep();
    }
	// fclose(fp);
	// for(int i=0;i<=1000;i++)
	// 	r.sleep();
    }
    return 0;
 
}

