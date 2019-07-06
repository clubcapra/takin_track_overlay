#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/core/ocl.hpp>
//#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>
#include <chrono>
#include <thread>
#include <string>
#include <ros/console.h>

using namespace cv;
using namespace std;

//our sensitivity value to be used in the absdiff() function
static int SENSITIVITY_VALUE = 20;
//size of blur used to smooth the intensity image output from absdiff() function
static int BLUR_SIZE = 30;
//we'll have just one object to search for
//and keep track of its position.
int theObject[2] = {0,0};
//bounding rectangle of the object, we will use the center of this as its position.

        
int main(int argc, char *argv[])
{

    ros::init(argc,argv,"takin_track_overlay");

    ros::NodeHandle nh("~");
    
    string source;
    nh.param<string>("source",source,"");
    string output = source.substr(0,source.find_first_of("/",1))+"/track_overlay"+source.substr(source.find_first_of("/",1));
    //cv::namedWindow("view");
    //cv::startWindowThread();
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub_overlay = it.advertise( output+"/image_raw",1);
    
    string source_encoding;
    
    nh.param<string>("source_encoding",source_encoding,"");
    ROS_INFO("Source: %s",source.c_str());
    ROS_INFO("Output: %s",output.c_str());   

    nh.param("sensitivity",SENSITIVITY_VALUE,SENSITIVITY_VALUE);
    ROS_INFO("sensitivity: %i",SENSITIVITY_VALUE);

    nh.param("blur",BLUR_SIZE,BLUR_SIZE);
    ROS_INFO("blur: %i",BLUR_SIZE);


    image_transport::Subscriber sub = it.subscribe(source, 1, [&](const sensor_msgs::ImageConstPtr& msg)
    {
        Mat current_frame;

        try
        {
            current_frame=cv_bridge::toCvShare(msg, source_encoding)->image;

        //cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
        //cv::waitKey(1);
        //this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        catch (cv_bridge::Exception& e)
        {
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
        }

        cv::line(current_frame,cv::Point(92,480),cv::Point(305,265),cv::Scalar(0, 0, 255),3);
        cv::line(current_frame,cv::Point(565, 480),cv::Point(356,265),cv::Scalar(0, 0, 255),3);


        pub_overlay.publish(cv_bridge::CvImage(
            std_msgs::Header() /* empty header */,
            sensor_msgs::image_encodings::BGR8 /* image format */,
            current_frame    /* the opencv image object */
        ).toImageMsg());
	
    
    });
    ros::spin();
    //cv::destroyWindow("view");


    return 0;
}

