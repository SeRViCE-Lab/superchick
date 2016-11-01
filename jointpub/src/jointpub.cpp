#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "super_state_publisher");
    ros::NodeHandle n;
    ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
    tf::TransformBroadcaster broadcaster;
    ros::Rate loop_rate(30);

    const double degree = M_PI/180;

    // robot state
    double bladder = 0, tinc = degree, table=0, angle=0, height=0, hinc=0.005;

    // message declarations
    geometry_msgs::TransformStamped head_trans;
    sensor_msgs::JointState joint_state;
    head_trans.header.frame_id = "/world";
    head_trans.child_frame_id = "/vicon/Superdude/head";

    while (ros::ok()) {
        //update joint_state
        joint_state.header.stamp = ros::Time::now();
        joint_state.name.resize(2);
        joint_state.position.resize(2);
        joint_state.name[0] ="table_to_bladder";
        joint_state.position[0] = table;
        joint_state.name[1] ="bladder_to_headnball";
        joint_state.position[1] = bladder;


        // update transform
        // (moving in a circle with radius=2)
        head_trans.header.stamp = ros::Time::now();
        head_trans.transform.translation.x = cos(angle)*2;
        head_trans.transform.translation.y = sin(angle)*2;
        head_trans.transform.translation.z = .7;
        head_trans.transform.rotation = tf::createQuaternionMsgFromYaw(angle+M_PI/2);

        //send the joint state and transform
        joint_pub.publish(joint_state);
        broadcaster.sendTransform(head_trans);

        // Create new robot state
        bladder += tinc;
        if (bladder<-.5 || bladder>0) tinc *= -1;
        height += hinc;
        if (height>.2 || height<0) hinc *= -1;
        table += degree;
        angle += degree/4;

        // This will adjust as needed per iteration
        loop_rate.sleep();
    }


    return 0;
}
