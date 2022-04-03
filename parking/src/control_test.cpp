//STD header
#include<vector>
#include<cmath>
#include<string>
#include<iostream>

//ROS header
#include<ros/ros.h>
#include<tf/tf.h>
//#include<tf/transform_broadcaster.h>
#include<tf/transform_listener.h>
#include<tf/transform_datatypes.h>

//Message header
#include<geometry_msgs/TwistStamped.h>
#include<geometry_msgs/PoseStamped.h>
#include<geometry_msgs/PointStamped.h>
#include<geometry_msgs/Vector3.h>
#include<geometry_msgs/TransformStamped.h>
#include<nav_msgs/Odometry.h>
#include<std_msgs/Float64MultiArray.h>
#include<std_msgs/Float64.h>
#include<std_msgs/Int32.h>
#include<nav_msgs/Path.h>
#include<tf2_geometry_msgs/tf2_geometry_msgs.h>
//#include<carla_msgs/CarlaEgoVehicleControl.h>

//TF2 Header
#include<tf2/LinearMath/Transform.h>
#include<tf2/convert.h>
#include<tf2/LinearMath/Quaternion.h>
#include<tf2_ros/transform_broadcaster.h>


//Using
using namespace std;
using std::atan2;

class control{
    public:
        control(){
            

            sub_lengths0 = nh_.subscribe("/length0",100,&control::lengthsCallBack_0,this);
            sub_lengths1 = nh_.subscribe("/length1",100,&control::lengthsCallBack_1,this);
            sub_lengths2 = nh_.subscribe("/length2",100,&control::lengthsCallBack_2,this);
            sub_lengths3 = nh_.subscribe("/length3",100,&control::lengthsCallBack_3,this);
            sub_lengths4 = nh_.subscribe("/length4",100,&control::lengthsCallBack_4,this);
            sub_lengths5 = nh_.subscribe("/length5",100,&control::lengthsCallBack_5,this);
            sub_lengths6 = nh_.subscribe("/length6",100,&control::lengthsCallBack_6,this);

            pub_closetIDX = nh_.advertise<std_msgs::Int32>("/closest_idx",100);
            pub_cmd = nh_.advertise<carla_msgs::CarlaEgoVehicleControl>("/carla/ego_vehicle/vehicle_control_cmd",100);

            sub_initPoint = nh_.subscribe("/init",100,&control::pinitCallBack,this);
            sub_refPath = nh_.subscribe("/path",100,&control::refPathCallBack,this);
            sub_vehicle = nh_.subscribe("/rear",100,&control::vehicleCallback,this);
            sub_mode = nh_.subscribe("/mode",100,&control::modeCallback,this);
            
            ros::spinOnce();

            egovehicleRef();

        }
        ~control(){}
    private:
        //Node Handle
        ros::NodeHandle nh_;
        
        //Publisher
        ros::Publisher pub_closetIDX;
        ros::Publisher pub_cmd;

        //Subscriber
        ros::Subscriber sub_lengths0;
        ros::Subscriber sub_lengths1;
        ros::Subscriber sub_lengths2;
        ros::Subscriber sub_lengths3;
        ros::Subscriber sub_lengths4;
        ros::Subscriber sub_lengths5;
        ros::Subscriber sub_lengths6;
        ros::Subscriber sub_initPoint;
        ros::Subscriber sub_refPath;
        ros::Subscriber sub_vehicle;
        ros::Subscriber sub_mode;

        //TF
        
        tf2::Transform tf;
        tf2::Quaternion q2;
        tf::TransformListener tfListen;

        //Input
        geometry_msgs::PoseStamped p_init;
        std_msgs::Float64 ref_lengths_0;
        std_msgs::Float64 ref_lengths_1;
        std_msgs::Float64 ref_lengths_2;
        std_msgs::Float64 ref_lengths_3;
        std_msgs::Float64 ref_lengths_4;
        std_msgs::Float64 ref_lengths_5;
        std_msgs::Float64 ref_lengths_6;
        nav_msgs::Path ref_path_;
        nav_msgs::Odometry egovehiclOdom;

        //Output
        nav_msgs::Odometry egovehicle_odom;
        nav_msgs::Path past_path_;
        carla_msgs::CarlaEgoVehicleControl park_cmd_;

        //Member Variable
        double PI = 3.141592;
        geometry_msgs::PoseStamped current_pose;
        double curr_steer = 0;
        double curr_accel = 0;
        double curr_speed = 0.01;
        double egovehicleLength = 2.875; //2.875
        double distance = 0;
        double ref_distance = 0;
        double x=0;
        double y=0;
        double pre_x_;
        double pre_y_;
        double yaw = 0;

        bool pinitcallback = false;
        bool lengthcallback0 = false;
        bool lengthcallback1 = false;
        bool lengthcallback2 = false;
        bool lengthcallback3 = false;
        bool lengthcallback4 = false;
        bool lengthcallback5 = false;
        bool lengthcallback6 = false;
        bool refPathcallback = false;
        bool carlaVehicleOk = false;
        bool initToprep2Ok = false;
        bool prep2Toprep1Ok = false;
        bool prep1TogoalOk = false;



        int m_minIndx;
        int defineIdx = 25;
        int pre_ld_Idx;
        int ld_Idx;
        double max_steer = 1.22173035145;
        double kp1 = 0.4; double kp2 = 0.5; double kff = 0.5; double kpv = 0.5;
        
        

    public:
        int mode{-1};

        void lengthsCallBack_0(const std_msgs::Float64::ConstPtr &msg){ref_lengths_0 = *msg;lengthcallback0 = true;}
        void lengthsCallBack_1(const std_msgs::Float64::ConstPtr &msg){ref_lengths_1 = *msg;lengthcallback1 = true;}
        void lengthsCallBack_2(const std_msgs::Float64::ConstPtr &msg){ref_lengths_2 = *msg;lengthcallback2 = true;}
        void lengthsCallBack_3(const std_msgs::Float64::ConstPtr &msg){ref_lengths_3 = *msg;lengthcallback3 = true;}
        void lengthsCallBack_4(const std_msgs::Float64::ConstPtr &msg){ref_lengths_4 = *msg;lengthcallback4 = true;}
        void lengthsCallBack_5(const std_msgs::Float64::ConstPtr &msg){ref_lengths_5 = *msg;lengthcallback5 = true;}
        void lengthsCallBack_6(const std_msgs::Float64::ConstPtr &msg){ref_lengths_6 = *msg;lengthcallback6 = true;}

        void pinitCallBack(const geometry_msgs::PoseStamped::ConstPtr &msg){
            p_init = *msg;
            pinitcallback = true;
        }

        void refPathCallBack(const nav_msgs::Path::ConstPtr &msg){
            ref_path_ = *msg;
            refPathcallback = true;
        }

        void vehicleCallback(const nav_msgs::OdometryConstPtr &msg){
            egovehiclOdom = *msg;
            x = egovehiclOdom.pose.pose.position.x;
            y = egovehiclOdom.pose.pose.position.y;
            pre_x_ = x;
            pre_y_ = y;
            yaw = tf::getYaw(egovehiclOdom.pose.pose.orientation);
            carlaVehicleOk = true;
        }

        void modeCallback(const std_msgs::Int32ConstPtr &msg){
            mode = msg->data;
        }

        void getLdindex(){
            nav_msgs::Path park_path;
            int sample_size = ref_path_.poses.size();
            double m_dminDist = 10000;

            for(int i_pose = 0;i_pose < sample_size ; i_pose += 1){
                geometry_msgs::PointStamped pathPose_map;
                pathPose_map.header.frame_id = "/map";
                pathPose_map.header.stamp = ros::Time(0);
                pathPose_map.point.x = ref_path_.poses[i_pose].pose.position.x;
                pathPose_map.point.y = ref_path_.poses[i_pose].pose.position.y;
                geometry_msgs::PointStamped pathPose_body;
                try{
                    tfListen.transformPoint("/ego_vehicle/rear2",pathPose_map,pathPose_body);
                    double m_dDist = sqrt(pow(pathPose_body.point.x,2) + pow(pathPose_body.point.y,2));
                    if(m_dDist < m_dminDist){
                        m_dminDist = m_dDist;
                        m_minIndx = i_pose;
                    }
                }
                catch(tf::TransformException &ex){}
            }
            ld_Idx = m_minIndx - defineIdx;
            std_msgs::Int32 closest_idx;
            closest_idx.data = m_minIndx;
            pub_closetIDX.publish(closest_idx);
        }

        void egovehicleRef(){
            ros::Rate rate(10);
            while(ros::ok()){
                ros::spinOnce();

                if((carlaVehicleOk == false)||(mode == 0)){
                    rate.sleep();
                    continue;
                }

                if(lengthcallback0 == true && lengthcallback1 == true && lengthcallback2 == true&& lengthcallback3 == true&& lengthcallback4 == true&& lengthcallback5 == true&& lengthcallback6 == true && refPathcallback == true){
                    x = egovehiclOdom.pose.pose.position.x;
                    y = egovehiclOdom.pose.pose.position.y;
                    yaw = tf::getYaw(egovehiclOdom.pose.pose.orientation);
                    curr_speed = egovehiclOdom.twist.twist.linear.x;

                    static int step = 0;
                    double step1_x = ref_path_.poses[400].pose.position.x;
                    double step1_y = ref_path_.poses[400].pose.position.y;
                    double step2_x = ref_path_.poses[0].pose.position.x;
                    double step2_y = ref_path_.poses[0].pose.position.y;
                    double step1_dist = sqrt(pow(step1_x - x,2) + pow(step1_y - y,2));
                    double step2_dist = sqrt(pow(step2_x - x,2) + pow(step2_y - y,2));



                    pre_ld_Idx = ld_Idx;
                    getLdindex();
                    if((step == 0)&&(ld_Idx<400)){
                        ld_Idx = 400;
                        if(m_minIndx < 400){
                            m_minIndx = 400;
                        }
                    }

                    if((step==1)&&(m_minIndx>=400)){
                        m_minIndx = 399;
                    }

                    double ld;
                    double beta;
                    double error_s;
                    double error_v;
                    double error_yaw;

                    ld = sqrt(pow(ref_path_.poses[ld_Idx].pose.position.x - x,2) + pow(ref_path_.poses[ld_Idx].pose.position.y - y,2));
                    beta = atan2(ref_path_.poses[ld_Idx].pose.position.y - y, ref_path_.poses[ld_Idx].pose.position.x - x) - yaw;
                    if(beta > M_PI){
                        beta = beta - 2*M_PI;
                    }
                    else if(beta < -M_PI){
                        beta = beta + 2*M_PI;
                    }

                    error_yaw = tf::getYaw(ref_path_.poses[ld_Idx].pose.orientation) - yaw;
                    if(error_yaw > M_PI){
                        error_yaw = error_yaw - 2*M_PI;
                    }
                    else if(error_yaw < -M_PI){
                        error_yaw = error_yaw + 2*M_PI;
                    }
                
                    float limit = 0.25;
                    if((step1_dist > limit)&&(step==0)){
                        error_v =  ref_path_.poses[m_minIndx].pose.position.z - curr_speed;

                        park_cmd_.gear = 1;
                        park_cmd_.reverse = false;
                        park_cmd_.brake = 0;
                        park_cmd_.throttle = error_v * kpv;
                        park_cmd_.steer = -(atan2(2.0*sin(beta)*egovehicleLength,ld))/max_steer;
                        if(park_cmd_.throttle < 0){
                            park_cmd_.throttle = 0;
                        }
                    }
                    else if((step1_dist < limit)&&(step==0)&&(curr_speed > 0.2)){
                        park_cmd_.steer = 0;
                        park_cmd_.throttle = 0;
                        park_cmd_.brake = 1;
                    }
                    else if((step1_dist < limit)&&(step==0)&&(curr_speed < 0.2)){
                        step = 1;
                    }
                    else if((step2_dist > limit)&&(step==1)){
                        error_v =  -(ref_path_.poses[m_minIndx].pose.position.z - curr_speed);

                        park_cmd_.gear = -1;
                        park_cmd_.reverse = true;
                        park_cmd_.brake = 0;
                        park_cmd_.throttle = error_v * kpv;
                        park_cmd_.steer = -(atan2(2.0*sin(beta)*egovehicleLength,ld))/max_steer;
                    }
                    else if((step2_dist < limit)&&(step==1)){
                        park_cmd_.steer = 0;
                        park_cmd_.throttle = 0;
                        park_cmd_.brake = 1;
                        cout << " parking complete\n";
                    }
                }
                pub_odom();
                rate.sleep();
            }
        }

        void pub_odom(){
            pub_cmd.publish(park_cmd_);
        }
};

int main(int argc, char **argv){
    ros::init(argc, argv, "control_test");

    control c;

    return 0;
}
