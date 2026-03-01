/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include<opencv2/core/core.hpp>

#include"../../../include/System.h"

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <stereo_msgs/DisparityImage.h>
#include <message_filters/sync_policies/exact_time.h>

#include <image_transport/subscriber_filter.h>

#include <image_geometry/stereo_camera_model.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <libelas/elas.h>


using namespace std;

class ImageGrabber
{
private:
    boost::scoped_ptr<Elas::parameters> param;
    boost::shared_ptr<Elas> elas_;
    image_geometry::StereoCameraModel model_;
    boost::shared_ptr<image_transport::Publisher> depth_pub_;


    int queue_size_;
    // Struct parameters
    int disp_min;
    int disp_max;
    double support_threshold;
    int support_texture;
    int candidate_stepsize;
    int incon_window_size;
    int incon_threshold;
    int incon_min_support;
    bool add_corners;
    int grid_size;
    double beta;
    double gamma;
    double sigma;
    double sradius;
    int match_texture;
    int lr_threshold;
    double speckle_sim_threshold;
    int speckle_size;
    int ipol_gap_width;
    bool filter_median;
    bool filter_adaptive_mean;
    bool postprocess_only_left;
    bool subsampling;

public:
    ImageGrabber(ORB_SLAM3::System* pSLAM):mpSLAM(pSLAM){
        ros::NodeHandle local_nh("~");
        local_nh.param("queue_size", queue_size_, 5);
        local_nh.param<int>("disp_min", disp_min, 0);
        local_nh.param<int>("disp_max", disp_max, 255);
        local_nh.param<double>("support_threshold", support_threshold, 0.95);
        local_nh.param<int>("support_texture", support_texture, 10);
        local_nh.param<int>("candidate_stepsize", candidate_stepsize, 5);
        local_nh.param<int>("incon_window_size", incon_window_size, 5);
        local_nh.param<int>("incon_threshold", incon_threshold, 5);
        local_nh.param<int>("incon_min_support", incon_min_support, 5);
        local_nh.param<bool>("add_corners", add_corners, 0);
        local_nh.param<int>("grid_size", grid_size, 20);
        local_nh.param<double>("beta", beta, 0.02);
        local_nh.param<double>("gamma", gamma, 3);
        local_nh.param<double>("sigma", sigma, 1);
        local_nh.param<double>("sradius", sradius, 2);
        local_nh.param<int>("match_texture", match_texture, 1);
        local_nh.param<int>("lr_threshold", lr_threshold, 2);
        local_nh.param<double>("speckle_sim_threshold", speckle_sim_threshold, 1);
        local_nh.param<int>("speckle_size", speckle_size, 200);
        local_nh.param<int>("ipol_gap_width", ipol_gap_width, 300);
        local_nh.param<bool>("filter_median", filter_median, 0);
        local_nh.param<bool>("filter_adaptive_mean", filter_adaptive_mean, 1);
        local_nh.param<bool>("postprocess_only_left", postprocess_only_left, 1);
        local_nh.param<bool>("subsampling", subsampling, 0); 
     
        param.reset(new Elas::parameters);
        /* Parameters tunned*/
        param->disp_min = disp_min;
        param->disp_max = disp_max;
        param->support_threshold = support_threshold;
        param->support_texture = support_texture;
        param->candidate_stepsize = candidate_stepsize;
        param->incon_window_size = incon_window_size;
        param->incon_threshold = incon_threshold;
        param->incon_min_support = incon_min_support;
        param->add_corners = add_corners;
        param->grid_size = grid_size;
        param->beta = beta;
        param->gamma = gamma;
        param->sigma = sigma;
        param->sradius = sradius;
        param->match_texture = match_texture;
        param->lr_threshold = lr_threshold;
        param->speckle_sim_threshold = speckle_sim_threshold;
        param->speckle_size = speckle_size;
        param->ipol_gap_width = ipol_gap_width;
        param->filter_median = filter_median;
        param->filter_adaptive_mean = filter_adaptive_mean;
        param->postprocess_only_left = postprocess_only_left;
        param->subsampling = subsampling;
        elas_.reset(new Elas(*param));
        
        image_transport::ImageTransport local_it(local_nh);
        depth_pub_.reset(new image_transport::Publisher(local_it.advertise("depth", 1)));

    }
     void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD);

    void process(const sensor_msgs::ImageConstPtr &l_image_msg, const sensor_msgs::ImageConstPtr &r_image_msg, const sensor_msgs::CameraInfoConstPtr &l_info_msg, const sensor_msgs::CameraInfoConstPtr &r_info_msg);

    ORB_SLAM3::System* mpSLAM;
    bool do_rectify;
    cv::Mat M1l,M2l,M1r,M2r;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Stereo");
    ros::start();

    if(argc != 4)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM3 Stereo path_to_vocabulary path_to_settings do_rectify" << endl;
        ros::shutdown();
        return 1;
    }    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::RGBD, true);

    ImageGrabber igb(&SLAM);

    stringstream ss(argv[3]);
	ss >> boolalpha >> igb.do_rectify;

    if(igb.do_rectify)
    {      
        // Load settings related to stereo calibration
        cv::FileStorage fsSettings(argv[2], cv::FileStorage::READ);
        if(!fsSettings.isOpened())
        {
            cerr << "ERROR: Wrong path to settings" << endl;
            return -1;
        }

        cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
        fsSettings["LEFT.K"] >> K_l;
        fsSettings["RIGHT.K"] >> K_r;

        fsSettings["LEFT.P"] >> P_l;
        fsSettings["RIGHT.P"] >> P_r;

        fsSettings["LEFT.R"] >> R_l;
        fsSettings["RIGHT.R"] >> R_r;

        fsSettings["LEFT.D"] >> D_l;
        fsSettings["RIGHT.D"] >> D_r;

        int rows_l = fsSettings["LEFT.height"];
        int cols_l = fsSettings["LEFT.width"];
        int rows_r = fsSettings["RIGHT.height"];
        int cols_r = fsSettings["RIGHT.width"];

        if(K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
                rows_l==0 || rows_r==0 || cols_l==0 || cols_r==0)
        {
            cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
            return -1;
        }

        cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,igb.M1l,igb.M2l);
        cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,igb.M1r,igb.M2r);
    }

    ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::Image> left_sub(nh, "/camera/left/image_rect", 1);
    message_filters::Subscriber<sensor_msgs::Image> right_sub(nh, "/camera/right/image_rect", 1);
    message_filters::Subscriber<sensor_msgs::CameraInfo>  left_info (nh, "/camera/left/camera_info", 1);
    message_filters::Subscriber<sensor_msgs::CameraInfo>  right_info(nh, "/camera/right/camera_info", 1);


/*
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> ApproximatePolicy;
    typedef message_filters::Synchronizer<ApproximatePolicy> ApproximateSync;
    boost::shared_ptr<ApproximateSync> approximate_sync_;
    approximate_sync_.reset(new ApproximateSync(ApproximatePolicy(10), left_sub,  right_sub,  left_info,  right_info));
    approximate_sync_->registerCallback(boost::bind(&ImageGrabber::process, igb, _1, _2, _3, _4));
*/
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), left_sub,right_sub,  left_info,  right_info);
    sync.registerCallback(boost::bind(&ImageGrabber::process ,&igb,_1,_2,_3,_4));

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory_TUM_Format.txt");
    SLAM.SaveTrajectoryTUM("FrameTrajectory_TUM_Format.txt");
    SLAM.SaveTrajectoryKITTI("FrameTrajectory_KITTI_Format.txt");

    ros::shutdown();

    return 0;
}

                                                                             
void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB, const sensor_msgs::ImageConstPtr& msgD)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrD;
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    mpSLAM->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,cv_ptrRGB->header.stamp.toSec());
}

 void ImageGrabber::process(const sensor_msgs::ImageConstPtr &l_image_msg, const sensor_msgs::ImageConstPtr &r_image_msg,const sensor_msgs::CameraInfoConstPtr &l_info_msg, const sensor_msgs::CameraInfoConstPtr &r_info_msg)
  {
        std::cout<<"Received images and camera info."<<std::endl;
        model_.fromCameraInfo(l_info_msg, r_info_msg);
        stereo_msgs::DisparityImagePtr disp_msg = boost::make_shared<stereo_msgs::DisparityImage>();
        disp_msg->header = l_info_msg->header;
        disp_msg->image.header = l_info_msg->header;
        disp_msg->image.height = l_image_msg->height;
        disp_msg->image.width = l_image_msg->width;
        disp_msg->image.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
        disp_msg->image.step = disp_msg->image.width * sizeof(float);
        disp_msg->image.data.resize(disp_msg->image.height * disp_msg->image.step);
        disp_msg->min_disparity = param->disp_min;
        disp_msg->max_disparity = param->disp_max;

        // Stereo parameters
        float f = model_.right().fx();
        float T = model_.baseline();
        float depth_fact = T * f * 1000.0f;
        uint16_t bad_point = std::numeric_limits<uint16_t>::max();

        // Have a synchronised pair of images, now to procss using elas
        // convert images if necessary 
        uint8_t *l_image_data, *r_image_data;
        int32_t l_step, r_step;
        cv_bridge::CvImageConstPtr l_cv_ptr, r_cv_ptr;
        if (l_image_msg->encoding == sensor_msgs::image_encodings::MONO8)
        {
        l_image_data = const_cast<uint8_t *>(&(l_image_msg->data[0]));
        l_step = l_image_msg->step;
        }
        else
        {
        l_cv_ptr = cv_bridge::toCvShare(l_image_msg, sensor_msgs::image_encodings::MONO8);
        l_image_data = l_cv_ptr->image.data;
        l_step = l_cv_ptr->image.step[0];
        }
        if (r_image_msg->encoding == sensor_msgs::image_encodings::MONO8)
        {
        r_image_data = const_cast<uint8_t *>(&(r_image_msg->data[0]));
        r_step = r_image_msg->step;
        }
        else
        {
        r_cv_ptr = cv_bridge::toCvShare(r_image_msg, sensor_msgs::image_encodings::MONO8);
        r_image_data = r_cv_ptr->image.data;
        r_step = r_cv_ptr->image.step[0];
        }
        ROS_ASSERT(l_step == r_step);
        ROS_ASSERT(l_image_msg->width == r_image_msg->width);
        ROS_ASSERT(l_image_msg->height == r_image_msg->height);

        int32_t width = l_image_msg->width;
        int32_t height = l_image_msg->height;
        // Allocate
        const int32_t dims[3] = {l_image_msg->width, l_image_msg->height, l_step};
        //float* l_disp_data = new float[width*height*sizeof(float)];
        float *l_disp_data = reinterpret_cast<float *>(&disp_msg->image.data[0]);
        float *r_disp_data = new float[width * height * sizeof(float)];
        // Procss
        elas_->process(l_image_data, r_image_data, l_disp_data, r_disp_data, dims);
        // Find the max for scaling the image colour
        float disp_max = 0;
        for (int32_t i = 0; i < width * height; i++)
        {
        if (l_disp_data[i] > disp_max)
            disp_max = l_disp_data[i];
        if (r_disp_data[i] > disp_max)
            disp_max = r_disp_data[i];
        }

        cv_bridge::CvImage out_depth_msg;
        out_depth_msg.header = l_image_msg->header;
        out_depth_msg.encoding = sensor_msgs::image_encodings::MONO16;
        out_depth_msg.image = cv::Mat(height, width, CV_16UC1);
        //创建out_depth_msg_image_data 的指针，它指向 out_depth_msg 的图像数据
        uint16_t *out_depth_msg_image_data = reinterpret_cast<uint16_t *>(&out_depth_msg.image.data[0]);
        std::vector<int32_t> inliers;
        for (int32_t i = 0; i < width * height; i++)
        {
            float disp = l_disp_data[i];
            out_depth_msg_image_data[i] = disp <= 0.0f ? bad_point : (uint16_t)(depth_fact / disp); 


            if (l_disp_data[i] > 0)
                inliers.push_back(i);
        }
         // Publish
        depth_pub_->publish(out_depth_msg.toImageMsg());
        
        sensor_msgs::ImageConstPtr msgD;
        // 使用 toImageMsg 将 CvImage 转换为 sensor_msgs::Image
        //msgD = cv_bridge::CvImage(l_image_msg->header,  sensor_msgs::image_encodings::MONO8, out_depth_msg.image).toImageMsg();
        GrabRGBD(l_image_msg ,out_depth_msg.toImageMsg());

        delete r_disp_data;
        return;
  }
