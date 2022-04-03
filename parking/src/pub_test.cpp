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
#include<nav_msgs/Odometry.h>
#include<std_msgs/Float64MultiArray.h>
#include<std_msgs/Float64.h>
#include<std_msgs/Int32.h>

//Using
using namespace std;

class pointPub{
    public:
        pointPub(){
            pub_init = nh_.advertise<geometry_msgs::PoseStamped>("/p_init",100);
            pub_goal = nh_.advertise<geometry_msgs::PoseStamped>("/p_goal",100);
            saveGoalPoint();
            sub_idx = nh_.subscribe("/parking_idx",100,&pointPub::idxCallback,this);
            sub_rear = nh_.subscribe("/rear",100,&pointPub::rearCallback,this);
            sub_mode = nh_.subscribe("/mode",100,&pointPub::modeCallback,this);

        }
        
    private:
        //Node Handle
        ros::NodeHandle nh_;
        //Publisher
        ros::Publisher pub_init;
        ros::Publisher pub_goal;
        //Subscriber
        ros::Subscriber sub_idx;
        ros::Subscriber sub_rear;
        ros::Subscriber sub_mode;
        geometry_msgs::PoseStamped init;
        geometry_msgs::PoseStamped goal;
        //nav_msgs::Path initTogoal;
        int slot_idx_;
        double Pi = 3.141592;
        double slot[60][3];
        int mode{0};
        tf::Quaternion q;
    public:
        void saveGoalPoint(){
            slot[0][0] = -27.16250488; slot[0][1] = 18.81921875; slot[0][2] = Pi;
            slot[1][0] = -27.16250488; slot[1][1] = 21.61921875; slot[1][2] = Pi;
            slot[2][0] = -27.16250488; slot[2][1] = 24.41921875; slot[2][2] = Pi; 
            slot[3][0] = -27.16250488; slot[3][1] = 27.21921875; slot[3][2] = Pi;
            slot[4][0] = -27.16250488; slot[4][1] = 30.01921875; slot[4][2] = Pi;
            slot[5][0] = -27.16250488; slot[5][1] = 32.81921875; slot[5][2] = Pi;
            slot[6][0] = -27.16250488; slot[6][1] = 35.61921875; slot[6][2] = Pi;
            slot[7][0] = -27.16250488; slot[7][1] = 38.41921875; slot[7][2] = Pi;
            slot[8][0] = -27.16250488; slot[8][1] = 41.21921875; slot[8][2] = Pi;
            slot[9][0] = -27.16250488; slot[9][1] = 44.01921875; slot[9][2] = Pi;

            slot[10][0] = -24.73445801; slot[10][1] = 18.81921875; slot[10][2] = 0;
            slot[11][0] = -24.73445801; slot[11][1] = 21.61921875; slot[11][2] = 0;
            slot[12][0] = -24.73445801; slot[12][1] = 24.41921875; slot[12][2] = 0;
            slot[13][0] = -24.73445801; slot[13][1] = 27.21921875; slot[13][2] = 0;
            slot[14][0] = -24.73445801; slot[14][1] = 30.01921875; slot[14][2] = 0;
            slot[15][0] = -24.73445801; slot[15][1] = 32.81921875; slot[15][2] = 0;
            slot[16][0] = -24.73445801; slot[16][1] = 35.61921875; slot[16][2] = 0;
            slot[17][0] = -24.73445801; slot[17][1] = 38.41921875; slot[17][2] = 0;
            slot[18][0] = -24.73445801; slot[18][1] = 41.21921875; slot[18][2] = 0;
            slot[19][0] = -24.73445801; slot[19][1] = 44.01921875; slot[19][2] = 0;

            slot[20][0] = -11.07066772; slot[20][1] = 18.81921875; slot[20][2] = Pi;
            slot[21][0] = -11.07066772; slot[21][1] = 21.61921875; slot[21][2] = Pi;
            slot[22][0] = -11.07066772; slot[22][1] = 24.41921875; slot[22][2] = Pi;
            slot[23][0] = -11.07066772; slot[23][1] = 27.21921875; slot[23][2] = Pi;
            slot[24][0] = -11.07066772; slot[24][1] = 30.01921875; slot[24][2] = Pi;
            slot[25][0] = -11.07066772; slot[25][1] = 32.81921875; slot[25][2] = Pi;
            slot[26][0] = -11.07066772; slot[26][1] = 35.61921875; slot[26][2] = Pi;
            slot[27][0] = -11.07066772; slot[27][1] = 38.41921875; slot[27][2] = Pi;
            slot[28][0] = -11.07066772; slot[28][1] = 41.21921875; slot[28][2] = Pi;
            slot[29][0] = -11.07066772; slot[29][1] = 44.01921875; slot[29][2] = Pi;

            slot[30][0] = -8.7669165; slot[30][1] = 18.81921875; slot[30][2] = 0;
            slot[31][0] = -8.7669165; slot[31][1] = 21.61921875; slot[31][2] = 0;
            slot[32][0] = -8.7669165; slot[32][1] = 24.41921875; slot[32][2] = 0;
            slot[33][0] = -8.7669165; slot[33][1] = 27.21921875; slot[33][2] = 0;
            slot[34][0] = -8.7669165; slot[34][1] = 30.01921875; slot[34][2] = 0;
            slot[35][0] = -8.7669165; slot[35][1] = 32.81921875; slot[35][2] = 0;
            slot[36][0] = -8.7669165; slot[36][1] = 35.61921875; slot[36][2] = 0;
            slot[37][0] = -8.7669165; slot[37][1] = 38.41921875; slot[37][2] = 0;
            slot[38][0] = -8.7669165; slot[38][1] = 41.21921875; slot[38][2] = 0;
            slot[39][0] = -8.7669165; slot[39][1] = 44.01921875; slot[39][2] = 0;

            slot[40][0] = 5.6330835; slot[40][1] = 18.81921875; slot[40][2] = Pi;
            slot[41][0] = 5.6330835; slot[41][1] = 21.61921875; slot[41][2] = Pi;
            slot[42][0] = 5.6330835; slot[42][1] = 24.41921875; slot[42][2] = Pi;
            slot[43][0] = 5.6330835; slot[43][1] = 27.21921875; slot[43][2] = Pi;
            slot[44][0] = 5.6330835; slot[44][1] = 30.01921875; slot[44][2] = Pi;
            slot[45][0] = 5.6330835; slot[45][1] = 32.81921875; slot[45][2] = Pi;
            slot[46][0] = 5.6330835; slot[46][1] = 35.61921875; slot[46][2] = Pi;
            slot[47][0] = 5.6330835; slot[47][1] = 38.41921875; slot[47][2] = Pi;
            slot[48][0] = 5.6330835; slot[48][1] = 41.21921875; slot[48][2] = Pi;
            slot[49][0] = 5.6330835; slot[49][1] = 44.01921875; slot[49][2] = Pi;

            slot[50][0] = 7.71096802; slot[50][1] = 18.81921875; slot[50][2] = 0;
            slot[51][0] = 7.71096802; slot[51][1] = 21.61921875; slot[51][2] = 0;
            slot[52][0] = 7.71096802; slot[52][1] = 24.41921875; slot[52][2] = 0;
            slot[53][0] = 7.71096802; slot[53][1] = 27.21921875; slot[53][2] = 0;
            slot[54][0] = 7.71096802; slot[54][1] = 30.01921875; slot[54][2] = 0;
            slot[55][0] = 7.71096802; slot[55][1] = 32.81921875; slot[55][2] = 0;
            slot[56][0] = 7.71096802; slot[56][1] = 35.61921875; slot[56][2] = 0;
            slot[57][0] = 7.71096802; slot[57][1] = 38.41921875; slot[57][2] = 0;
            slot[58][0] = 7.71096802; slot[58][1] = 41.21921875; slot[58][2] = 0;
            slot[59][0] = 7.71096802; slot[59][1] = 44.01921875; slot[59][2] = 0;
        }
        void pub_point(){
            // init.header.frame_id = "/map";
            // init.pose.position.x = -35.2;
            // init.pose.position.y = 42.5;
            // init.pose.position.z = 0;
            // q = tf::createQuaternionFromRPY(0.0,0.0,-Pi*0.5);
            // init.pose.orientation.x = q[0];
            // init.pose.orientation.y = q[1];
            // init.pose.orientation.z = q[2];
            // init.pose.orientation.w = q[3];

            goal.header.frame_id = "/map";
            goal.pose.position.x = slot[slot_idx_][0] ;
            goal.pose.position.y = slot[slot_idx_][1];
            goal.pose.position.z = 0;
            q = tf::createQuaternionFromRPY(0.0,0.0,slot[slot_idx_][2]);
            goal.pose.orientation.x = q[0];
            goal.pose.orientation.y = q[1];
            goal.pose.orientation.z = q[2];
            goal.pose.orientation.w = q[3];
            pub_init.publish(init);
            pub_goal.publish(goal);
        }

        void idxCallback(const std_msgs::Int32ConstPtr &msg){
            if(msg->data != -1){
                slot_idx_ = msg->data;
            }
            if(mode == 1){
                pub_point();
            }
        }

        void rearCallback(const nav_msgs::OdometryConstPtr &msg){
            init.header.frame_id = "/map";
            init.pose = msg->pose.pose;
        }

        void modeCallback(const std_msgs::Int32ConstPtr &msg){
            mode = msg->data;
        }
};

int main(int argc, char **argv){
    ros::init(argc,argv,"pub_test");
    pointPub pub;
    ros::spin();
    return 0;
}