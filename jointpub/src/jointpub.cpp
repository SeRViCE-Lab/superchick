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
        joint_state.position[1] = 0; //bladder;
        joint_state.name[2] ="base_to_right_leg";
        joint_state.position[2] =  0;
        joint_state.name[3] ="base_to_left_leg";
        joint_state.position[3] = 0;
        joint_state.name[4] ="rightleg_to_tabletop";
        joint_state.position[4] = 0.0;
        joint_state.name[5] ="tablecover_to_tabletop";
        joint_state.position[5] = -0;
        joint_state.name[6] ="table_to_bladder";
        joint_state.position[6] = 0;
        joint_state.name[7] ="base_panel_to_table_top";
        joint_state.position[7] = 0;
        joint_state.name[8] ="vicon_to_world";
        joint_state.position[8] = 0;
        joint_state.name[9] ="panel_12_oc_to_base_panel";
        joint_state.position[9] = 0;
        joint_state.name[10] ="panel_9_oc_to_panel_12_oc";
        joint_state.position[10] = 0;
        joint_state.name[11] ="panel_3_oc_to_panel_12_oc";
        joint_state.position[11] = 0;
        joint_state.name[12] ="leftleg_to_tabletop";
        joint_state.position[12] = 0.0;
        joint_state.name[13] = "table_to_big_bladder";
        joint_state.name[14] = "table_to_rightneckbladder";
        joint_state.name[15] = "table_to_leftneckbladder";
        joint_state.name[16] = "bigbladder_to_head";
        joint_state.name[17] = "rightbladder_to_neck";
        joint_state.name[18] = "leftbladder_to_neck";
        joint_state.name[19] = "tableTop_to_basePanel";
        joint_state.name[20] = "vicon_to_world";

        //send the joint state and transform
        joint_pub.publish(joint_state);
        broadcaster.sendTransform(head_trans);

        // This will adjust as needed per iteration
        loop_rate.sleep();
    }


    return 0;
}
