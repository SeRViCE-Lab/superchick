/*
Olalekan Ogunmolu. 
SeRViCe Lab, 
Nov. 11, 2015*/

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <ros/callback_queue.h>         //for multithreaded spinning
#include <ros/console.h>
#include <tf/transform_broadcaster.h>
#include <thread>
#include <iomanip>
#include <string>
#include <fstream>
#include <iostream>

#include <vicon_bridge/Markers.h>
#include <vicon_listener/structs.h>
#include <geometry_msgs/Twist.h>

#include <boost/asio.hpp>
#include "boost/bind.hpp"
#include "boost/date_time/posix_time/posix_time_types.hpp"

const std::string Superchick_name = "Superchicko";
std::string listener = "vicon_listener";

std::string subject, segment;

const short multicast_port = 30001;
const int max_message_count = 300;                //because my head can never be more than 2m above the ground :)

class sender
{
public:
  sender(boost::asio::io_service& io_service,
      const boost::asio::ip::address& multicast_address, 
      float  x, float y, float z, float roll, \
      float pitch, float yaw, bool print)
    : endpoint_(multicast_address, multicast_port),
      socket_(io_service, endpoint_.protocol()),
      timer_(io_service), x_(x), y_(y), z_(z), \
                        roll_(roll), pitch_(pitch), yaw_(yaw), print(print)
  {
    std::ostringstream os;
    os << std::fixed << std::setfill ('0') << std::setprecision (4) << x_ 
                    << ", " << y_ << ", "<< z_ << ", "<< roll_ << ", "
                    << pitch_ << ", "<< yaw_ ;
    message_ = os.str();

    socket_.async_send_to(
        boost::asio::buffer(message_), endpoint_,
        boost::bind(&sender::handle_send_to, this,
          boost::asio::placeholders::error));
    if(print) 
    {   
        ROS_INFO_STREAM("Sent (x,y,z, r, p, y): "<< std::setw('0') << std::fixed << std::setprecision(4)<< message_ <<")");
    }
  }

  void handle_send_to(const boost::system::error_code& error)
  {
    if (!error && z_ < max_message_count)
    {
      timer_.expires_from_now(boost::posix_time::seconds(1));
      timer_.async_wait(
          boost::bind(&sender::handle_timeout, this,
            boost::asio::placeholders::error));
    }
  }

  void handle_timeout(const boost::system::error_code& error)
  {
    if (!error )
    {
      std::ostringstream os;

      os << std::fixed << std::setfill ('0') << std::setprecision (4) << x_ 
                       << ", " << y_ << ", "<< z_ << ", "<< roll_ << ", "
                       << pitch_ << ", "<< yaw_ ;

      message_ = os.str();

      if(print){ ROS_WARN("Message Timed Out. Please look into your send::handle_timeout function"); }

      socket_.async_send_to(
          boost::asio::buffer(message_), endpoint_,
          boost::bind(&sender::handle_send_to, this,
            boost::asio::placeholders::error));
    }
  }

private:
  boost::asio::ip::udp::endpoint endpoint_;
  boost::asio::ip::udp::socket socket_;
  boost::asio::deadline_timer timer_;
  float x_, y_, z_;
  float roll_, pitch_, yaw_;
  std::string message_;
  bool print;
};

class Receiver
{ 

private:
    float xm, ym, zm;
    unsigned short port;
    bool save, print, sim; 

    Vector3f rpy;
    facemidpts facepoints;

    std::string foreheadname, \
                leftcheekname, \
                rightcheekname, \
                chinname;

    geometry_msgs::Point_<std::allocator<void> > chin, forehead, leftcheek, rightcheek;
    
    std::vector<std::thread> threads;
    ros::NodeHandle nm_;
    ros::Publisher pub;
// friend server;                       //somehow I could not get g++ to compile by exposing everything within  Receiver to server
public:
    Receiver(const ros::NodeHandle nm, bool save, bool print, bool sim)
        :  nm_(nm), save(save), print(print), sim(sim)
    {          
        pub  = nm_.advertise<geometry_msgs::Twist>("/vicon/headtwist", 100);
    }

    void callback(const vicon_bridge::Markers::ConstPtr& markers_msg)
    {   
        //Retrieve geometry_msgs translation for four markers on superchicko    
        forehead            = markers_msg -> markers[0].translation;
        leftcheek           = markers_msg -> markers[1].translation;
        rightcheek          = markers_msg -> markers[2].translation;
        chin                = markers_msg -> markers[3].translation;

        foreheadname        = markers_msg -> markers[0].marker_name;
        leftcheekname       = markers_msg -> markers[1].marker_name;
        rightcheekname      = markers_msg -> markers[2].marker_name;
        chinname            = markers_msg -> markers[3].marker_name;

        if(print)
        {
/*            std::cout << "Header stamp: "<< markers_msg -> header.stamp <<  
             "   | Frame Number: " << markers_msg -> frame_number << std::endl;  */

            // ROS_INFO_STREAM("markers_msg: " << *markers_msg);          

            //Print a bunch'o'stuff to assert correctness of the above
            std::cout << "\n" << std::endl;
            std::cout << foreheadname   <<  ":\n "    << forehead     << std::endl;
            std::cout << leftcheekname  <<  ":\n "    << leftcheek    << std::endl;
            std::cout << rightcheekname <<  ":\n "    << rightcheek   << std::endl;
            std::cout << chinname       <<  ":\n "    << chin         << std::endl;
        }

        headmarkers markers = {forehead, leftcheek, rightcheek, chin};
        facemidpts midFacePoints_       = midpoint(markers);
    }

    ~Receiver()
    { }

    //Here, I basically find the midpoint of the four pts intersecting at the middle of the face
    facemidpts midpoint(headmarkers markers)                     
    {
        xm = 0.25 * (markers.foreo.x + markers.lefto.x + markers.righto.x + markers.chino.x);
        ym = 0.25 * (markers.foreo.y + markers.lefto.y + markers.righto.y + markers.chino.y);
        zm = 0.25 * (markers.foreo.z + markers.lefto.z + markers.righto.z + markers.chino.z); 

        if (save)
          {
            savepoints(/*xm, ym, zm*/);
          }        

        facepoints = {xm, ym, zm};

        modgramschmidt(markers, facepoints);                    //compute stable orthogonalization and normalization

        return facepoints;
    }
    
//   Now I compute the Gram-Schmidt orthonormalization and orthogonalization for the four vectors
//     https://en.m.wikipedia.org/wiki/Gram%E2%80%93Schmidt_process#CITEREFGolubVan_Loan1996
//     http://www.cs.cmu.edu/~kiranb/animation/p245-shoemake.pdf
    void modgramschmidt(headmarkers markers, facemidpts facepoints)
    {
        //define req'd sys of finitely independent set
        Vector3d v1(markers.foreo.x, markers.foreo.y, markers.foreo.z);
        Vector3d v2(markers.lefto.x, markers.lefto.y, markers.lefto.z);
        Vector3d v3(markers.righto.x, markers.righto.y, markers.righto.z);
        Vector3d v4(markers.chino.x, markers.chino.y, markers.chino.z);    

        Vector3d u1, u2, u3, u4, u0;                            //define orthogonal set S' = {u1, u2, u3, u4}
        Vector3d u31, u32, u41, u42, u43;                       //define intermediate orthogonal that avoids round-off errors

        u1 = v1 ;                                               //we set u1 to v1
        u2 = v2     - proj(u1, v2);

        u31 = v3    - proj(u1, v3);
        u32 = u31   - proj(u2, u31);
        u3 = u32;

        u41 = v4    - proj(u1, v4);
        u42 = u41   - proj(u2, u41);
        u43 = u42   - proj(u3, u42);
        u4 = u43;

        //orthogonality check: dot products among vectors should be null
        float a1, a2, a3, a4;
        a1 = u1.dot(u2); a2 = u2.dot(u3); a3 = u3.dot(u4); a4 = u2.dot(u4);

        Vector3d e1, e2, e3, e4;                                //define orthonormal set S' = {e1, e2, e3, e4}
        e1 = u1 / u1.norm();
        e2 = u2 / u2.norm();
        e3 = u3 / u3.norm();
        e4 = u4 / u4.norm();

        //orthonormality check:: ||e1||, ||e2||, ||e3||, ||e4|| should be 1
        if( !(e1.norm() == 1 || e2.norm() || e3.norm() || e4.norm()) )
        {            

        }
        orth gonal  = {u1, u2, u3, u4};
        orth normal = {e1, e2, e3, e4};

        rot(normal, facepoints);                                //compute rotation matrix
    }

    Vector3d proj(Vector3d u, Vector3d v)                       //computes the projection of vector v onto u.
    {
        float ratio = v.dot(u) / u.dot(u);
        Vector3d projuv = ratio * u;

        return projuv;
    }

    void savepoints(/*float xm, float ym, float zm*/)
    {
        //Now we write the points to a text file for visualization processing
        std::ofstream midface;
        midface.open("midface.csv", std::ofstream::out | std::ofstream::app);
        midface << xm <<"\t" <<ym << "\t" << zm << "\n";
        midface.close();
    }

    MatrixXd rot(orth normal, facemidpts facepoints)
    {
        Vector3d col1, col2, col3, col4;
        col1 = normal.e1;                       //each of these are 3 X 1 in dim
        col2 = normal.e2;
        col3 = normal.e3;
        col4 = normal.e4;

        // std::cout << "col1: " << col1 << "\n col1.size(): \n" << col1.rows() << ", " << col1.cols() << std::endl;

        MatrixXd E(3, 4);
        E.col(0) = col1;
        E.col(1) = col2;
        E.col(2) = col3;
        E.col(3) = col4;

        MatrixXd R(3,3);
        R.col(0)   = -col1;
        R.col(1)   = col2;
        R.col(2)   = col3;

        MatrixXd Rt = R.transpose().eval();

        MatrixXd I(3,3);
        I = R * Rt;


        float det = R.determinant();

        rollpy(R, facepoints);                  //computes roll-pitch-yaw motion
        return R;
    }

    //From Rotation Matrix, find rpy
    Vector3f rollpy(MatrixXd R, facemidpts facepoints)
    {
        float sp, cp;
        MatrixXf Rf = R.cast<float>();

        ros::Rate loop_rate(30);                       //publish at 30Hz*/
        geometry_msgs::Twist posemsg;

        if(ros::ok())
        {           
            boost::asio::io_service io_service;
            const std::string multicast_address = "235.255.0.1";

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
                
                sender s(io_service, boost::asio::ip::address::from_string(multicast_address), posemsg.linear.x, \
                                                        posemsg.linear.y, posemsg.linear.z, posemsg.angular.x, \
                                                        posemsg.angular.y, posemsg.angular.z, print);
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
           
                sender s(io_service, boost::asio::ip::address::from_string(multicast_address), posemsg.linear.x, \
                                                        posemsg.linear.y, posemsg.linear.z, posemsg.angular.x, \
                                                        posemsg.angular.y, posemsg.angular.z, print);
            }   

            io_service.run();
            pub.publish(posemsg);
            loop_rate.sleep();
            if(print)                
            {
                // ROS_INFO_STREAM("Head Translation: " << posemsg.linear);
                // ROS_INFO_STREAM("Head RPY Angles: " << posemsg.angular);

            }
        }

        return rpy;
    }
};

void help()
{
  
    ROS_INFO_STREAM( "\n\n" 
                    <<  std::setw('0') << std::setw(35)  <<"Error ******************************************************************\n"
                    <<  std::setw('0') << std::setw(35)  << "Usage: rosrun vicon_listener vicon_listener <subject_name> <segment_name>\n"
                    <<  std::setw('0') << std::setw(35)  << "To Save facepoints: rosrun vicon_listener vicon_listener -s (or --save)\n");
}


int main(int argc, char **argv)
{
    
    uint32_t options = 0;

    ros::init(argc, argv, listener, options);
    
    if (argc < 3)
      {
        help();
      }


    bool save, print, sim;

    ros::NodeHandle nm;
    save = nm.getParam("save", save) ;
    print = nm.getParam("print", print);
    sim = nm.getParam("sim", sim);

    for(size_t i = 1; i < (size_t)argc; ++i)
    {        
        subject = argv[1];
        segment = argv[2]; 
        print   = print || argv[3];
        save    = save  || argv[4];
        sim     = sim   || argv[5];
    }

    std::string base_name = "vicon";

    std::string pubname    = subject + "/" + segment ;

    ROS_INFO_STREAM("Subscribing to: /" << base_name <<  "/" << pubname);


    // ROS_INFO_STREAM("print: " << print << " | save: " << save);
    Receiver  obj(nm, save, print, sim);

    ros::Subscriber sub = nm.subscribe("vicon/markers", 1000, &Receiver::callback, &obj );    

    ros::spin();

  return 1;
}
    
    
    
    