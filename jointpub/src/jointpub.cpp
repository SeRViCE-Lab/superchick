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
    int rate = 30;
    // ros::param::get("rate", rate);
    ros::Rate loop_rate(rate);

    const double degree = M_PI/180;

    // robot state
    double bladder = 0, tinc = degree, table=0, angle=0, height=0, hinc=0.005;

    // message declarations
    geometry_msgs::TransformStamped head_trans;
    sensor_msgs::JointState joint_state;
    head_trans.header.frame_id = "/world";
    head_trans.child_frame_id = "/vicon/head_transform";

    while (ros::ok()) 
    {
        //update joint_state
        joint_state.header.stamp = ros::Time::now();
        joint_state.name.resize(13);
        joint_state.position.resize(13);
        joint_state.name[0] ="table_to_bladder";
        joint_state.position[0] = 0; //table;
        joint_state.name[1] ="bladder_to_headnball";
        joint_state.position[1] = 0.2; //bladder;
        joint_state.name[2] ="base_to_right_leg";
        joint_state.position[2] =  0.3;
        joint_state.name[3] ="base_to_left_leg";
        joint_state.position[3] = 0.3;
        joint_state.name[4] ="rightleg_to_tabletop";
        joint_state.position[4] = 0.35;
        joint_state.name[5] ="tablecover_to_tabletop";
        joint_state.position[5] = -0.08;
        joint_state.name[6] ="table_to_bladder";
        joint_state.position[6] = 0;
        joint_state.name[7] ="base_panel_to_table_top";
        joint_state.position[7] = -0.18;
        joint_state.name[8] ="motion_capture_to_world";
        joint_state.position[8] = 2.54;
        joint_state.name[9] ="panel_12_oc_to_base_panel";
        joint_state.position[9] = .200025/2;
        joint_state.name[10] ="panel_9_oc_to_panel_12_oc";
        joint_state.position[10] = 0;
        joint_state.name[11] ="panel_3_oc_to_panel_12_oc";
        joint_state.position[11] = 0;
        joint_state.name[12] ="leftleg_to_tabletop";
        joint_state.position[12] = 0.35;


        // update transform
        // (moving in a circle with radius=2)
        head_trans.header.stamp = ros::Time::now();
        head_trans.transform.translation.x = cos(angle)*2;
        head_trans.transform.translation.y = sin(angle)*2;
        head_trans.transform.translation.z = 0; 
        head_trans.transform.rotation = tf::createQuaternionMsgFromYaw(angle+M_PI/2);

        //send the joint state and transform
        joint_pub.publish(joint_state);
        broadcaster.sendTransform(head_trans);

        // Create new robot state
        bladder += tinc;
        if (bladder<-1.0 || bladder>1) tinc *= -1;
        height += hinc;
        if (height>.5 || height<0) hinc *= -1;
        table += degree;
        angle += degree/4;

        // This will adjust as needed per iteration
        loop_rate.sleep();
    }


    return 0;
}
