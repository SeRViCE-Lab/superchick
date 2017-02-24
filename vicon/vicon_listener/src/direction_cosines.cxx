/*
Olalekan Ogunmolu. 
SeRViCe Lab, 
Nov. 11, 2015*/

#include "ros/ros.h"
#include <ros/spinner.h>
#include "std_msgs/String.h"
#include <ros/console.h>
#include <tf/transform_broadcaster.h>
#include <thread>
#include <string>
#include <fstream>
#include <mutex>
#include <chrono>

#include <vicon_bridge/Markers.h>
#include <geometry_msgs/Point.h>

#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>

#include "boost/bind.hpp"
#include <boost/thread.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>

const std::string Superchick_name = "Superchicko";
std::string listener = "vicon_listener";

std::string subject, segment;
const std::string globalTopicName = "/vicon/Superdude/head";
using namespace Eigen;

class Receiver
{ 

private:
    float xm, ym, zm;
    bool save, print, sim, running; 
    double headRoll, headPitch, headYaw;
    double panelRoll, panelPitch, panelYaw;

    Vector3f rpy;

    std::string foreheadname, \
                leftcheekname, \
                rightcheekname, \
                chinname;

    geometry_msgs::Point chin, forehead, leftcheek, rightcheek;
    geometry_msgs::Point botRight, botLeft, topRight, topLeft;

    std::vector<geometry_msgs::Point> headMarkersVector, 
                                      panelMarkersVector;
    
    std::vector<std::thread> threads;
    ros::NodeHandle nm_;
    ros::Publisher pub;
    Matrix3d R; //basis rotation matrix with respect to camera
    boost::mutex rotation_mutex_, markers_mutex;
    geometry_msgs::Vector3 headTrans; 
    geometry_msgs::Quaternion headQuat;
    std::mutex mutex;
    bool updatePose;
    ros::AsyncSpinner spinner;
    unsigned long const hardware_threads;

    //for rigidTransforms
    geometry_msgs::Vector3 panelTrans;
    geometry_msgs::Quaternion panelQuat;

    std::vector<std::thread> threadsVector;
    std::thread testQuatThread, modGramScmidtThread;

    Matrix3d headMGS, tableMGS;

    //use exactTimePolicy to process panel markers and head markers
    // using namespace message_filters;

    using vicon_sub = message_filters::Subscriber<vicon_bridge::Markers> ;
    using head_sub  = message_filters::Subscriber<geometry_msgs::TransformStamped>;
    using pane_sub  = message_filters::Subscriber<geometry_msgs::TransformStamped>;
    using headSyncPolicy = message_filters::sync_policies::ExactTime<vicon_bridge::Markers, 
                                geometry_msgs::TransformStamped, geometry_msgs::TransformStamped>;

    vicon_sub subVicon;
    head_sub  subHead;
    pane_sub  subPanel;  

    message_filters::Synchronizer<headSyncPolicy> sync;                            

public:
    Receiver(ros::NodeHandle nm, bool save, bool print, bool sim)
        :  nm_(nm), save(save), print(print), sim(sim), hardware_threads(std::thread::hardware_concurrency()),
           subVicon(nm_, "/vicon/markers", 1), subHead(nm_, "vicon/Superdude/head", 1), 
           subPanel(nm_,"/vicon/Panel/rigid", 1), spinner(2),
           sync(headSyncPolicy(10), subVicon, subHead, subPanel), updatePose(false)
    {      
        // ExactTime takes a queue size as its constructor argument, hence SyncPolicy(10)
       sync.registerCallback(boost::bind(&Receiver::callback, this, _1, _2, _3));
       // ROS_INFO("Class construct");
    }

    ~Receiver()
    { }

    Receiver(Receiver const&) =delete;
    Receiver& operator=(Receiver const&) = delete;

    void run()
    {
      spawn();
      unspawn();
    }
private:
    void spawn()
    {
        if(spinner.canStart())
            spinner.start(); 
        running = true; 
        
        while(!updatePose)
            std::this_thread::sleep_for(std::chrono::milliseconds(1));     

        //spawn the threads
        modGramScmidtThread = std::thread(&Receiver::modGramSchmidt, this);
        
        if(modGramScmidtThread.joinable())
            modGramScmidtThread.join();
    }

    void unspawn()
    {
        spinner.stop();
        modGramScmidtThread.detach();
        running = false;
    }

    void callback(const vicon_bridge::MarkersConstPtr& markers_msg, 
                const geometry_msgs::TransformStampedConstPtr& panel_msg,
                const geometry_msgs::TransformStampedConstPtr& head_msg)
    {   
        //solve all vicon markers here
        geometry_msgs::Point forehead, leftcheek, rightcheek, chin;
        //Retrieve geometry_msgs translation for four markers on superchicko 
        std::vector<geometry_msgs::Point> headMarkersVector;
        headMarkersVector.resize(4);
        headMarkersVector[0] = forehead    = markers_msg -> markers[0].translation;
        headMarkersVector[1] = leftcheek   = markers_msg -> markers[1].translation;
        headMarkersVector[2] = rightcheek  = markers_msg -> markers[2].translation;
        headMarkersVector[3] = chin        = markers_msg -> markers[3].translation;

        std::vector<geometry_msgs::Point> panelMarkersVector;
        panelMarkersVector.resize(4);
        panelMarkersVector[0] = botLeft    = markers_msg->markers[4].translation;
        panelMarkersVector[1] = botRight   = markers_msg->markers[5].translation;
        panelMarkersVector[2] = topRight   = markers_msg->markers[6].translation;
        panelMarkersVector[3] = topLeft    = markers_msg->markers[7].translation;

        //solve head transformedstamp markers here
        geometry_msgs::Vector3 headTrans = head_msg->transform.translation;
        geometry_msgs::Quaternion headQuat = head_msg->transform.rotation;

        //solve panel transformedstamp markers here
        geometry_msgs::Vector3 panelTrans = panel_msg->transform.translation;
        geometry_msgs::Quaternion panelQuat = panel_msg->transform.rotation;

        //convert translations to millimeters
        metersTomilli(std::move(headTrans)); 
        metersTomilli(std::move(panelTrans));

        boost::mutex::scoped_lock lock(markers_mutex);
        // std::lock_guard<std::mutex> lock(mutex);
        this->forehead = forehead;
        this->leftcheek = leftcheek;
        this->rightcheek = rightcheek;
        this->chin = chin;

        this->headTrans   = headTrans;
        this->headQuat    = headQuat;

        this->panelTrans    = panelTrans;
        this->panelQuat  = panelQuat;

        this->headMarkersVector = headMarkersVector;
        this->panelMarkersVector= panelMarkersVector;

        updatePose          = true;   
        lock.unlock();
        // testQuat();
        // ROS_INFO_STREAM("check?: " << *markers_msg << *panel_msg << *head_msg);
    }

    void modGramSchmidt() noexcept
    {
        std::vector<geometry_msgs::Point> headMarkersVector, panelMarkersVector;
        
        //attached frame to face
        Vector3d fore, left, right, chin;  
        //base markers
        Vector3d botLeft, botRight, topRight, topLeft; 

        Matrix3d headMGS, tableMGS;

        for(; running && ros::ok() ;)
        {            
            if(updatePose)
            {   
                boost::mutex::scoped_lock lock(markers_mutex);
                // std::lock_guard<std::mutex> lock(mutex);
                {
                    headMarkersVector  = this->headMarkersVector;
                    panelMarkersVector = this->panelMarkersVector;
                    updatePose = false;
                }
                lock.unlock();
            }

            fore << headMarkersVector[0].x, headMarkersVector[0].y,
                    headMarkersVector[0].z;
            left << headMarkersVector[1].x, headMarkersVector[1].y,
                    headMarkersVector[1].z;  
            right<< headMarkersVector[2].x, headMarkersVector[2].y,
                    headMarkersVector[2].z;
            chin << headMarkersVector[3].x, headMarkersVector[3].y,
                    headMarkersVector[3].z;   

            botLeft << panelMarkersVector[0].x, panelMarkersVector[0].y,
                       panelMarkersVector[0].z;
            botRight<< panelMarkersVector[1].x, panelMarkersVector[1].y,
                       panelMarkersVector[1].z;
            topRight<< panelMarkersVector[2].x, panelMarkersVector[2].y,
                       panelMarkersVector[2].z;
            topLeft << panelMarkersVector[3].x, panelMarkersVector[3].y,
                       panelMarkersVector[3].z;
            
            //see https://ocw.mit.edu/courses/mathematics/18-335j-introduction-to-numerical-methods-fall-2010/lecture-notes/MIT18_335JF10_lec10a_hand.pdf
            std::vector<Vector3d> v(4), q(4);   
            //compute orthonormal basis vectors for face markers       
            v[0] = fore;   v[1] = left;   v[2] = right;    v[3] = chin;
            double r[3][3] = {};    
            for(auto i = 0; i < 3; ++i)
            {
                r[i][i] = v[i].norm();
                q[i]    = v[i]/r[i][i];
                for(auto j = i +1; j < 3; ++j)
                {
                    r[i][j] = q[i].transpose() * v[j];
                    v[j]    = v[j] - r[i][j] * q[i];
                }
            }  

            
            // populate headMGS rotation basis matrix
            for(auto i =0; i < 3; ++i)
            { 
                headMGS.col(i) = q[i];
            }
            this->headMGS = headMGS;

            //compute basis vectors for table markers
            v.clear(); q.clear();
            v[0] = botLeft; v[1] = botRight; v[2] = topRight; v[3] = topLeft;
            for(auto i = 0; i < 4; ++i)
            {
                r[i][i] = v[i].norm();
                q[i]    = v[i]/r[i][i];
                for(auto j = i +1; j < 4; ++j)
                {
                    r[i][j] = q[i].transpose() * v[j];
                    v[j]    = v[j] - r[i][j] * q[i];
                }
            }  
            // populate tableMGS rotation basis matrix
            for(auto i =0; i < 3; ++i)
            { 
                tableMGS.col(i) = q[i];
            }
            this->tableMGS = tableMGS;  

            ROS_INFO_STREAM("Table MGS Vectors: " << tableMGS);
            ROS_INFO_STREAM("Head MGS Vectors: " << headMGS);                     
        } 
    }

    void testQuat() noexcept
    {
        geometry_msgs::Quaternion headQuat, panelQuat;
        // if(updatePose)
        // {                        

        boost::mutex::scoped_lock lock(markers_mutex);
        // std::lock_guard<std::mutex> lock(mutex);
        {                
            headQuat = this->headQuat;
            panelQuat = this->panelQuat;
            updatePose = false;
        }
        lock.unlock();
        // }
        double roll, rolla, pitch, pitcha, yaw, yawa;

        getRPYFromQuaternion(std::forward<geometry_msgs::Quaternion>(headQuat), 
                             std::forward<double>(roll),
                             std::forward<double>(pitch), 
                             std::forward<double>(yaw));

        // rad2deg(std::move(roll));
        // rad2deg(std::move(pitch));
        // rad2deg(std::move(yaw));

        printf("\n|\troll \t|\tpitch \t|\tyaw\n %f , \t%f, \t%f", 
                        roll, pitch, yaw); 

    }

    void getRPYFromQuaternion(geometry_msgs::Quaternion&& rotQuat, double&& roll,
                             double&& pitch, double&& yaw)
    {

        tf::Quaternion q(rotQuat.x, rotQuat.y, rotQuat.z, rotQuat.w);
        tf::Matrix3x3 m(q);
        m.getRPY(roll, pitch, yaw);
        rad2deg(std::move(roll)); rad2deg(std::move(pitch)); rad2deg(std::move(yaw)); 
        // if(print)
        // {
        //     printf("\n|\tx \t|\ty \t|\tz \t|\troll \t|\tpitch \t|\tyaw\n %f, \t%f, \t%f,  \t%f , \t%f, \t%f", translation.x, 
        //         translation.y, translation.z, 
        //         roll, pitch, yaw);  
        // }
    }

    inline void metersTomilli(geometry_msgs::Vector3&& translation)
    {
        translation.x   *= 1000;
        translation.y   *= 1000;
        translation.z   *= 1000;
    }

    inline void rad2deg(double&& x)
    {
        x  *= 180/M_PI;
    }

    void getHeadPrincipalDirections()
    {
        geometry_msgs::Point forehead, leftcheek, rightcheek, chin;

    }
    
/*
    void savepoints()
    {
        //Now we write the points to a text file for visualization processing
        std::ofstream midface;
        midface.open("midface.csv", std::ofstream::out | std::ofstream::app);
        midface << xm <<"\t" <<ym << "\t" << zm << "\n";
        midface.close();
    }

    //From Rotation Matrix, find rpy
    Vector3f rollpy(MatrixXd R, facemidpts facepoints)
    {
        float sp, cp;
        MatrixXf Rf = R.cast<float>();

        ros::Rate loop_rate(30);                       //publish at 30Hz
        geometry_msgs::Twist posemsg;

        if(ros::ok())
        {           
            if (abs(R(0,0)) < .001 & abs(R(1,0)) < .001)           
            {
                // singularity
                rpy(0) = 0;
                rpy(1) = atan2(-Rf(2,0), Rf(0,0));
                rpy(2) = atan2(-Rf(1,2), Rf(1,1));

                posemsg.linear.x = facepoints.x;
                posemsg.linear.y = facepoints.y;
                posemsg.linear.z = facepoints.z;

                posemsg.angular.x = rpy(0);
                posemsg.angular.y = rpy(1);
                posemsg.angular.z = rpy(2);
           }

            else
            {   
                rpy(0) = atan2(Rf(1,0), Rf(0,0));
                sp = sin(rpy(0));
                cp = cos(rpy(0));
                rpy(1) = atan2(-Rf(2,0), cp * Rf(0,0) + sp * Rf(1,0));
                rpy(2) = atan2(sp * Rf(0,2) - cp * Rf(1,2), cp*Rf(1,1) - sp*Rf(0,1));
                
                posemsg.linear.x = facepoints.x;
                posemsg.linear.y = facepoints.y;
                posemsg.linear.z = facepoints.z;

                posemsg.angular.x = rpy(0);
                posemsg.angular.y = rpy(1);
                posemsg.angular.z = rpy(2);      
            }   

            // ROS_INFO("x, y, z, roll, pitch, yaw: %f %f %f %f %f %f", \
            //     posemsg.linear.x, posemsg.linear.y, posemsg.linear.z,
            //     posemsg.angular.x, posemsg.angular.y, posemsg.angular.z);  

            ros::Rate r(100);
            tf::TransformBroadcaster broadcaster;

            if(nm_.ok()){                
                tf::Transform transform;
                transform.setOrigin(tf::Vector3(0.0,0.0,0.2));
                transform.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));

                broadcaster.sendTransform(
                  tf::StampedTransform( transform,
                    ros::Time::now(),"table_link", globalTopicName));
                r.sleep();
            }

            pub.publish(posemsg);
            loop_rate.sleep();
        }

        // headAboveTable();

        return rpy;
    }
    

    void headAboveTable()
    {
        Matrix3d Rot;
        facemidpts facepoints;
        boost::mutex::scoped_lock lock(rotation_mutex_);
        Rot = this->R;
        facepoints = this->facepoints;
        lock.unlock();

        Vector3d faceWRTCamera(facepoints.x, facepoints.y, facepoints.z);


        Vector3d headHeight = Rot * faceWRTCamera;

        //check norms
        double faceTableNorm = faceWRTCamera.norm();
        double faceCamNorm =    std::sqrt(std::pow(facepoints.x, 2) + 
                                std::pow(facepoints.y, 2) + 
                                std::pow(facepoints.z,2));

        // ROS_INFO_STREAM("headHeight: " << headHeight.transpose());
        // ROS_INFO_STREAM("facepoints: " << faceWRTCamera.transpose());

        geometry_msgs::Quaternion rotQuat;
        if(updatePose)
        {
            rotQuat = this->rotQuat;
            tf::Quaternion q(rotQuat.x, rotQuat.y, rotQuat.z, rotQuat.w);
            tf::Matrix3x3 m(q);
            Eigen::Matrix3d e;
            tf::matrixTFToEigen (m, e);
            headHeight  = e * faceWRTCamera;
            // ROS_INFO_STREAM("headHeight Stable: " << headHeight.transpose());
            updatePose = false;
        }
        // ROS_INFO("face in Cam %f, face on Table %f", faceCamNorm, faceTableNorm);
    }*/
};


int main(int argc, char **argv)
{
    
    uint32_t options = 0;

    ros::init(argc, argv, listener, options);

    bool save, print, sim;

    ros::NodeHandle nm;

    save = nm.getParam("save", save) ;
    print = nm.getParam("print", print);
    sim = nm.getParam("sim", sim);

    Receiver  r(nm, save, print, sim);
    r.run();

    if(!ros::ok())
    {
      return 0;
    }

    ros::shutdown();
}
    
    
    
    