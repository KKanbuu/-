//STD header
#include<vector>
#include<cmath>
#include<string>
#include<iostream>

//ROS header
#include<ros/ros.h>
#include<tf/tf.h>


//Message header
#include<std_msgs/Int32.h>

using namespace std;

class indexPub{
    public:
        indexPub(){
            pub_index = nh_.advertise<std_msgs::Int32>("/slot_idx",100);
        }
    
    private:
        ros::NodeHandle nh_;
        ros::Publisher pub_index;

        std_msgs::Int32 slot_idx_;

    public:
        void pubIdx(){
            slot_idx_.data = 8;
            pub_index.publish(slot_idx_); 
        }
};

int main(int argc, char **argv){
    ros::init(argc, argv,"pub_index");
    indexPub pubI;

    ros::Rate rate(10);
    while(ros::ok()){
        pubI.pubIdx();
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}