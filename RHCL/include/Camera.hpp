/*
Copyright 2021, Yang Luo"
This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

@Author
Yang Luo, PHD
Shenyang Institute of Automation, Chinese Academy of Sciences.
 email: luoyang@sia.cn
*/

#ifndef RHCL_PROJECT_CAMERA_H
#define RHCL_PROJECT_CAMERA_H

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/logger.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <opencv2/opencv.hpp>
//#include <signal.h>
#include <csignal>
#include <cstdlib>
#include <string>
#include <iostream>
#include <chrono>
#include <Eigen/Core>

namespace RHCL {

    class Camera {
    public:
        enum Processor {
            CPU, OPENCL, OPENGL, CUDA
        };


        Camera(Processor p = CPU, bool mirror = false, std::string serial = std::string());
        ~Camera();

        libfreenect2::Freenect2Device::IrCameraParams getIrParameters();

        libfreenect2::Freenect2Device::ColorCameraParams getRgbParameters();

        void disableLog();

        void enableLog();

        void printParameters();

        void storeParameters();

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr getCloud();

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr
        getCloud(const libfreenect2::Frame *rgb, const libfreenect2::Frame *depth,
                 pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr
        updateCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr
        updateCloud(const libfreenect2::Frame *rgb, const libfreenect2::Frame *depth,
                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

        void shutDown();

        void mirror();

        libfreenect2::SyncMultiFrameListener *getListener();

        void getDepth(cv::Mat depth_mat);

        void getIr(cv::Mat ir_mat);

        void getColor(cv::Mat &color_mat);

        void get(cv::Mat &color_mat,
                 cv::Mat &depth_mat,
                 const bool full_hd = true,
                 const bool remove_points = false);

        void get(cv::Mat &color_mat,
                 cv::Mat &depth_mat,
                 cv::Mat &ir_mat,
                 const bool full_hd = true,
                 const bool remove_points = false);

        void get(cv::Mat &color_mat,
                 cv::Mat &depth_mat,
                 pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                 const bool full_hd = true,
                 const bool remove_points = false);

        void prepareMake3D(const libfreenect2::Freenect2Device::IrCameraParams & depth_p);

    public:
        libfreenect2::Freenect2 freenect2_;
        libfreenect2::Freenect2Device *dev_ = 0;
        libfreenect2::PacketPipeline *pipeline_ = 0;
        libfreenect2::Registration *registration_ = 0;
        libfreenect2::SyncMultiFrameListener listener_;
        libfreenect2::Logger *logger_ = nullptr;
        libfreenect2::FrameMap frames_;
        libfreenect2::Frame undistorted_, registered_, big_mat_;
        Eigen::Matrix<float, 512, 1> colmap;
        Eigen::Matrix<float, 424, 1> rowmap;
        std::string serial_;
        int map_[512 * 424];
        float qnan_;
        bool mirror_;

    private:
        static void sigint_handler(int s);
        static bool stop;
    };

} //namespace RHCL

#endif //RHCL_PROJECT_CAMERA_H
