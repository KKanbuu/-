//STD header
#include<vector>
#include<cmath>
#include<string>
#include<iostream>

//ROS header
#include<ros/ros.h>
#include<tf/tf.h>
#include<tf/transform_broadcaster.h>

//Message header
#include<geometry_msgs/Pose.h>
#include<geometry_msgs/PoseStamped.h>
#include<nav_msgs/Path.h>
#include<std_msgs/Float64MultiArray.h>
#include<std_msgs/Float64.h>
#include<std_msgs/Int32.h>

//Using
using std::pow;
using std::cos;
using std::sin;
using std::abs;
using std::to_string;
using std::cout;
using std::vector;

class path{
    public:
        path(){
            p_init_.header.frame_id = "/map";
            p_goal_.header.frame_id = "/map";
            
            sub_init = nh_.subscribe("/p_init",100,&path::initBack,this);
            sub_goal = nh_.subscribe("/p_goal",100,&path::goalBack,this);
            sub_mode = nh_.subscribe("/mode",100,&path::modeBack,this);

            pub_pose = nh_.advertise<geometry_msgs::PoseStamped>("/goal",100);
            pub_pose1 = nh_.advertise<geometry_msgs::PoseStamped>("/prep1",100);
            pub_pose2 = nh_.advertise<geometry_msgs::PoseStamped>("/prep2",100);
            pub_init = nh_.advertise<geometry_msgs::PoseStamped>("/init",100);
            pub_path = nh_.advertise<nav_msgs::Path>("/path",100);
            pub_lenghts0 = nh_.advertise<std_msgs::Float64>("/length0",100);
            pub_lenghts1 = nh_.advertise<std_msgs::Float64>("/length1",100);
            pub_lenghts2 = nh_.advertise<std_msgs::Float64>("/length2",100);
            pub_lenghts3 = nh_.advertise<std_msgs::Float64>("/length3",100);
            pub_lenghts4 = nh_.advertise<std_msgs::Float64>("/length4",100);
            pub_lenghts5 = nh_.advertise<std_msgs::Float64>("/length5",100);
            pub_lenghts6 = nh_.advertise<std_msgs::Float64>("/length6",100);

            nh_.param("/dubin_test/g_width_",g_width_,4.0);
            nh_.param("/dubin_test/p1_width_",p1_width_,3.0);
            nh_.param("/dubin_test/p1_height_",p1_height_,2.5);
            nh_.param("/dubin_test/linear_d_",linear_d_,3.0);
            // nh_.param("/dubin_test/egovehicleLength",egovehicleLength,3.5);

            path_.header.frame_id = "/map"; 

        }
    private:
        //Node Handler
        ros::NodeHandle nh_;
        
        // Publisher
        ros::Publisher pub_path;
        ros::Publisher pub_pose;
        ros::Publisher pub_pose1;
        ros::Publisher pub_pose2;
        ros::Publisher pub_init;

        ros::Publisher pub_lenghts0;
        ros::Publisher pub_lenghts1;
        ros::Publisher pub_lenghts2;
        ros::Publisher pub_lenghts3;
        ros::Publisher pub_lenghts4;
        ros::Publisher pub_lenghts5;
        ros::Publisher pub_lenghts6;

        //Subscriber
        ros::Subscriber sub_init;
        ros::Subscriber sub_goal;
        ros::Subscriber sub_mode;

        //TF
        tf::Quaternion q;
        tf::Transform transform;
        static tf::TransformBroadcaster br;

        //Member Variable
        geometry_msgs::PoseStamped p_goal_;
        geometry_msgs::PoseStamped p_prep1_;
        geometry_msgs::PoseStamped p_prep2_;
        geometry_msgs::PoseStamped p_init_;

        
        
        double radius_;
        double alpha;
        double PI = 3.141592;

        double g_width_;
        double p1_width_;
        double p1_height_;
        double linear_d_;
        double prep2_yaw_;
        double g_yaw;
        double init_yaw_;
        double egovehicleLength;

        std_msgs::Float64 lengths0;
        std_msgs::Float64 lengths1;
        std_msgs::Float64 lengths2;
        std_msgs::Float64 lengths3;
        std_msgs::Float64 lengths4;
        std_msgs::Float64 lengths5;
        std_msgs::Float64 lengths6; 

        bool foworback;
        bool initbackok = false;
        bool goalbackok = false;

        int pathMode = 0;

        int point_num = 100;

        //Output >> forward path, backward path 따로 만들어서 저장하기?
        nav_msgs::Path path_;
        //nav_msgs::Path initTogoal;

    public:
        int mode_{-1};
    
        void initBack(const geometry_msgs::PoseStampedConstPtr &msg){
            p_init_ = *msg;
            initbackok = true;
        }
        void goalBack(const geometry_msgs::PoseStampedConstPtr &msg){
            p_goal_ = *msg;
            goalbackok = true;
           // ROS_INFO("Goal Point is callBack!");
        }
        void modeBack(const std_msgs::Int32ConstPtr &msg){
            mode_ = msg->data;
           // ROS_INFO("Goal Point is callBack!");
        }


        bool initok(){return initbackok;}
        bool goalok(){return goalbackok;}

        void defineMode(){
            g_yaw = tf::getYaw(p_goal_.pose.orientation);
            init_yaw_ = tf::getYaw(p_init_.pose.orientation);
            cout << "init_yaw : " << init_yaw_/M_PI*180.0 << "\n";
            if(p_init_.pose.position.x >= p_goal_.pose.position.x){
                if((init_yaw_ <= 0.75*PI) && (init_yaw_ >= 0.25*PI)){
                    pathMode = 1;
                }else if((init_yaw_ >= -0.75*PI) && (init_yaw_ <= -0.25*PI)){
                    pathMode = 2;
                }else{
                    // cout << "init_yaw_ : " << init_yaw_/M_PI*180.0 << "\n";
                    pathMode = 0; // Driving 하게끔?
                }
            }else{
                if((init_yaw_ <= 0.75*PI) && (init_yaw_ >= 0.25*PI)){
                    pathMode = 3;
                }else if((init_yaw_ >= -0.75*PI) && (init_yaw_ <= -0.25*PI)){
                    pathMode = 4;
                }else{
                    // cout << "init_yaw_ : " << init_yaw_/M_PI*180.0 << "\n";
                    pathMode = 0; // Driving 하게끔?
                    cout << "wow" << "\n";
                }
            }
        }

        double sign1(int mode){
            if(mode == 1){
                return 1;
            }else if(mode == 2){
                return 1;
            }else if(mode == 3){
                return -1;
            }else if(mode == 4){
                return -1;
            }else{
                return 1;
            }
        }

        double sign2(int mode){
            if(mode == 1){
                return 1;
            }else if(mode == 2){
                return -1;
            }else if(mode == 3){
                return 1;
            }else if(mode == 4){
                return -1;
            }else{
                return 1;
            }
        }

        double sign3(int mode){
            if(mode == 1){
                return 1;
            }else if(mode == 2){
                return -1;
            }else if(mode == 3){
                return -1;
            }else if(mode == 4){
                return 1;
            }else{
                return 1;
            }
        }

        void calPath(){
            defineMode();
            calprep();
            goalToprep1();
            prep1Toprep2();
            //cout<< init_yaw_/PI << "\n" << "\n";
            prep2Toinit();
            makeLengths();
        }

        void calprep(){
            cout << "Current Mode : " << pathMode << "\n";
            p_prep1_.header.frame_id = "/map";
            p_prep1_.pose.position.x = p_goal_.pose.position.x + g_width_*sign1(pathMode);
            p_prep1_.pose.position.y = p_goal_.pose.position.y ;
            p_prep1_.pose.position.z = 0.0;
            p_prep1_.pose.orientation = p_goal_.pose.orientation;
            

            p_prep2_.header.frame_id = "/map";
            p_prep2_.pose.position.x = p_prep1_.pose.position.x + p1_width_*sign1(pathMode);
            p_prep2_.pose.position.y = p_prep1_.pose.position.y + p1_height_*sign2(pathMode);
            p_prep2_.pose.position.z = 0.0;
            prep2_yaw_ = g_yaw + PI*0.5*sign3(pathMode);
            q = tf::createQuaternionFromRPY(0.0,0.0,prep2_yaw_);
            q.normalize();
            p_prep2_.pose.orientation.x = q[0];
            p_prep2_.pose.orientation.y = q[1];
            p_prep2_.pose.orientation.z = q[2];
            p_prep2_.pose.orientation.w = q[3];
            
            if(p_prep2_.pose.position.x - p_init_.pose.position.x == 0){
                alpha = 0;
            }else{
                alpha = 0.25*PI;
            }
            radius_ = (abs(p_init_.pose.position.x - p_prep2_.pose.position.x))/(1 - cos(alpha));
            
        }

        void goalToprep1(){
            for(int i=0;i<point_num;i++){
                geometry_msgs::PoseStamped point;
                point.header.frame_id = "/map";
                point.pose.position.x = p_goal_.pose.position.x + ((g_width_/point_num) * i)*sign1(pathMode);
                point.pose.position.y = p_goal_.pose.position.y ;
                point.pose.position.z =  -0.6;
                point.pose.orientation = p_goal_.pose.orientation;
                path_.poses.push_back(point);
            }
        }

        void prep1Toprep2(){
            for(int i=0;i<point_num;i++){
                geometry_msgs::PoseStamped point;
                point.header.frame_id = "/map";
                point.pose.position.x = p_prep1_.pose.position.x + ((abs(p1_width_ - p1_height_)/point_num) * i)*sign1(pathMode); 
                point.pose.position.y = p_prep1_.pose.position.y ;
                point.pose.position.z = -0.6 ;
                point.pose.orientation = p_prep1_.pose.orientation;
                path_.poses.push_back(point);
            }
            for(int i=0;i<point_num;i++){
                geometry_msgs::PoseStamped point;
                point.header.frame_id = "/map";
                point.pose.position.x = p_prep1_.pose.position.x + abs(p1_width_- p1_height_)*sign1(pathMode) + p1_height_*sin(((PI/2)/point_num)*i)*sign1(pathMode);
                point.pose.position.y = p_prep1_.pose.position.y + p1_height_*(1 - cos(((PI/2)/point_num)*i))*sign2(pathMode);
                point.pose.position.z = -0.6;
                double yaw = g_yaw + (((PI/2)/point_num)*i)*sign3(pathMode);
                q = tf::createQuaternionFromRPY(0.0,0.0,yaw);
                q.normalize();
                point.pose.orientation.x = q[0];
                point.pose.orientation.y = q[1];
                point.pose.orientation.z = q[2];
                point.pose.orientation.w = q[3];
                path_.poses.push_back(point);
            }
            for(int i=0;i<point_num;i++){
                geometry_msgs::PoseStamped point;
                point.header.frame_id = "/map";
                point.pose.position.x = p_prep1_.pose.position.x + p1_width_*sign1(pathMode); 
                point.pose.position.y = p_prep1_.pose.position.y + (p1_height_+(linear_d_/point_num)*i)*sign2(pathMode);
                point.pose.position.z = -0.6 ;
                point.pose.orientation = p_prep1_.pose.orientation;
                path_.poses.push_back(point);
            }
        }

        void prep2Toinit(){
            //if(fowardOrbackward){
                if(p_prep2_.pose.position.x - p_init_.pose.position.x > 0){
                    for(int i=0;i<point_num;i++){
                        geometry_msgs::PoseStamped point;
                        point.header.frame_id = "/map";
                        point.pose.position.x = p_prep2_.pose.position.x ;
                        point.pose.position.y = p_prep2_.pose.position.y + linear_d_*sign2(pathMode) - (linear_d_/point_num)*i*sign2(pathMode);
                        point.pose.position.z = 0.7;// * (1/point_num) * i;
                        point.pose.orientation = p_prep2_.pose.orientation;
                        path_.poses.push_back(point);
                    }
                    for(int i=0;i<point_num;i++){
                        geometry_msgs::PoseStamped point;
                        point.header.frame_id = "/map";
                        point.pose.position.x = p_prep2_.pose.position.x ;
                        point.pose.position.y = p_prep2_.pose.position.y - (((abs(p_prep2_.pose.position.y - p_init_.pose.position.y) - radius_*sin(alpha)) /point_num) * i)*sign2(pathMode);
                        point.pose.position.z = 1.0;// * (1/point_num) * i;
                        point.pose.orientation = p_prep2_.pose.orientation;
                        path_.poses.push_back(point);
                    }
                    for(int i=0;i<point_num;i++){
                        geometry_msgs::PoseStamped point;
                        point.header.frame_id = "/map";
                        point.pose.position.x = p_prep2_.pose.position.x - radius_*(1 - cos((alpha/point_num)*i)) ;
                        point.pose.position.y = p_prep2_.pose.position.y - ((abs(p_prep2_.pose.position.y - p_init_.pose.position.y) - radius_*sin(alpha)) + radius_*sin((alpha/point_num)*i))*sign2(pathMode);
                        point.pose.position.z = 0.5;//0.25 + 0.25 * (i/point_num);
                        double yaw = prep2_yaw_ - ((alpha/point_num)*i)*sign2(pathMode);
                        q = tf::createQuaternionFromRPY(0.0,0.0,yaw);
                        point.pose.orientation.x = q[0];
                        point.pose.orientation.y = q[1];
                        point.pose.orientation.z = q[2];
                        point.pose.orientation.w = q[3];
                        path_.poses.push_back(point);
                    }
                    
                    //ROS_INFO("Nan value generated!");
                }
                else if (p_prep2_.pose.position.x - p_init_.pose.position.x < 0){
                    //alpha = -alpha;
                    for(int i=0;i<point_num;i++){
                        geometry_msgs::PoseStamped point;
                        point.header.frame_id = "/map";
                        point.pose.position.x = p_prep2_.pose.position.x ;
                        point.pose.position.y = p_prep2_.pose.position.y + linear_d_*sign2(pathMode) - (linear_d_/point_num)*i*sign2(pathMode);
                        point.pose.position.z = 0.7;// * (1/point_num) * i;
                        point.pose.orientation = p_prep2_.pose.orientation;
                        path_.poses.push_back(point);
                    }
                    for(int i=0;i<point_num;i++){
                        geometry_msgs::PoseStamped point;
                        point.header.frame_id = "/map";
                        point.pose.position.x = p_prep2_.pose.position.x ;
                        point.pose.position.y = p_prep2_.pose.position.y - (((abs(p_prep2_.pose.position.y - p_init_.pose.position.y) - radius_*sin(alpha)) /point_num) * i)*sign2(pathMode);
                        point.pose.position.z =  1.0;//* (i/point_num);
                        point.pose.orientation = p_prep2_.pose.orientation;
                        path_.poses.push_back(point);
                    }
                    for(int i=0;i<point_num;i++){
                        geometry_msgs::PoseStamped point;
                        point.header.frame_id = "/map";
                        point.pose.position.x = p_prep2_.pose.position.x + radius_*(1 - cos((alpha/point_num)*i)) ;
                        point.pose.position.y = p_prep2_.pose.position.y - ((abs(p_prep2_.pose.position.y - p_init_.pose.position.y) - radius_*sin(alpha)) + radius_*sin((alpha/point_num)*i))*sign2(pathMode);
                        point.pose.position.z = 0.5;
                        double yaw = prep2_yaw_ + ((alpha/point_num)*i)*sign2(pathMode);
                        q = tf::createQuaternionFromRPY(0.0,0.0,yaw);
                        point.pose.orientation.x = q[0];
                        point.pose.orientation.y = q[1];
                        point.pose.orientation.z = q[2];
                        point.pose.orientation.w = q[3];
                        path_.poses.push_back(point);
                    }
                }
                else{
                    for(int i=0;i<point_num;i++){
                        geometry_msgs::PoseStamped point;
                        point.header.frame_id = "/map";
                        point.pose.position.x = p_prep2_.pose.position.x ;
                        point.pose.position.y = p_prep2_.pose.position.y - (((abs(p_prep2_.pose.position.y - p_init_.pose.position.y)) /point_num) * i)*sign2(pathMode);
                        point.pose.position.z = 1.0;
                        point.pose.orientation = p_prep2_.pose.orientation;
                        path_.poses.push_back(point);
                    }
                }
                
            //}
            //else 문 만들기
        }

        void makeLengths(){
            lengths0.data = radius_ * alpha;
            lengths1.data = (abs(p_init_.pose.position.y - p_prep2_.pose.position.y)) - radius_*sin(alpha);
            lengths2.data = p1_height_ * PI * 0.5;
            lengths3.data = abs(p1_width_ - p1_height_);
            lengths4.data = g_width_;
            lengths5.data = radius_;
            lengths6.data = p1_height_;
        }

        void publishPath(){
            pub_pose.publish(p_goal_);
            pub_pose1.publish(p_prep1_);
            pub_pose2.publish(p_prep2_);
            pub_init.publish(p_init_);
            pub_path.publish(path_);
            pub_lenghts0.publish(lengths0);
            pub_lenghts1.publish(lengths1);
            pub_lenghts2.publish(lengths2);
            pub_lenghts3.publish(lengths3);
            pub_lenghts4.publish(lengths4);
            pub_lenghts5.publish(lengths5);
            pub_lenghts6.publish(lengths6);
        }
};

int main(int argc, char **argv){
    ros::init(argc,argv,"dubin_test");

    path p;
    ros::Rate rate(10);
    bool pathcheck = false;
    while(ros::ok()){
        ros::spinOnce();
        if(p.mode_ == 1){
            if(p.initok() == true && p.goalok() == true && !pathcheck){
                ROS_INFO("Path is created!");
                p.calPath();
                p.publishPath();
                pathcheck = true;
            }
            rate.sleep();
        }
    }
    return 0;
}