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

#ifndef RHCL_CAMERA_HPP
#define RHCL_CAMERA_HPP

#include "ICamera.hpp"
#include <iostream>

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/logger.h>

#include <csignal>
#include <cstdlib>
#include <string>
#include <iostream>
#include <chrono>
#include <Eigen/Core>

namespace RHCL {

    class Freenect : public ICamera {
    public:
        Freenect(Processor p = OPENGL, bool mirror = false, std::string serial = std::string()) :
                ICamera(p, mirror),
                listener_(
                        libfreenect2::Frame::Color |
                        libfreenect2::Frame::Ir |
                        libfreenect2::Frame::Depth),
                undistorted_(512, 424,
                             4),
                registered_(512, 424, 4),
                big_mat_(1920, 1082, 4),
                qnan_(std::numeric_limits<float>::quiet_NaN()) {

//            signal(SIGINT, sigint_handler);

            if (freenect2_.enumerateDevices() == 0) {
                std::cout << "no kinect2 connected!" << std::endl;
                exit(-1);
            }

            switch (p) {
                case CPU:
                    std::cout << "creating Cpu processor" << std::endl;
                    if (serial.empty())
                        dev_ = freenect2_.openDefaultDevice(new libfreenect2::CpuPacketPipeline());
                    else
                        dev_ = freenect2_.openDevice(serial, new libfreenect2::CpuPacketPipeline());
                    std::cout << "created" << std::endl;
                    break;
#ifdef WITH_OPENCL
                    case OPENCL:
                std::cout << "creating OpenCL processor" << std::endl;
                if (serial.empty())
                    dev_ = freenect2_.openDefaultDevice(new libfreenect2::OpenCLPacketPipeline());
                else
                    dev_ = freenect2_.openDevice(serial, new libfreenect2::OpenCLPacketPipeline());
                break;
#endif
                case OPENGL:
                    std::cout << "creating OpenGL processor" << std::endl;
                    if (serial.empty())
                        dev_ = freenect2_.openDefaultDevice(new libfreenect2::OpenGLPacketPipeline());
                    else
                        dev_ = freenect2_.openDevice(serial, new libfreenect2::OpenGLPacketPipeline());
                    break;
#ifdef WITH_CUDA
                    case CUDA:
                std::cout << "creating Cuda processor" << std::endl;
                if (serial.empty())
                    dev_ = freenect2_.openDefaultDevice(new libfreenect2::CudaPacketPipeline());
                else
                    dev_ = freenect2_.openDevice(serial, new libfreenect2::CudaPacketPipeline());
                break;
#endif
                default:
                    std::cout << "creating Cpu processor" << std::endl;
                    if (serial_.empty())
                        dev_ = freenect2_.openDefaultDevice(new libfreenect2::CpuPacketPipeline());
                    else
                        dev_ = freenect2_.openDevice(serial, new libfreenect2::CpuPacketPipeline());
                    break;
            }

            if (!serial.empty())
                serial_ = serial;
            else
                serial_ = freenect2_.getDefaultDeviceSerialNumber();

            dev_->setColorFrameListener(&listener_);
            dev_->setIrAndDepthFrameListener(&listener_);
            dev_->start();

            logger_ = libfreenect2::getGlobalLogger();

            registration_ = new libfreenect2::Registration(dev_->getIrCameraParams(), dev_->getColorCameraParams());

            prepareMake3D(dev_->getIrCameraParams());

            std::cout << "p is " << p << std::endl;
        }

        void getDepth(ImgMat &depthMat) override {
            listener_.waitForNewFrame(frames_);

            extractFrame(frames_, depthMat, libfreenect2::Frame::Depth);

            listener_.release(frames_);

            std::cout << "Get Depth Image" << std::endl;
        }

        void getIr(ImgMat &irMat) override {
            listener_.waitForNewFrame(frames_);

            extractFrame(frames_, irMat, libfreenect2::Frame::Ir);

            listener_.release(frames_);

            std::cout << "Get Ir Image" << std::endl;
        }

        void getColor(ImgMat &colorMat) override {
            listener_.waitForNewFrame(frames_);

            extractFrame(frames_, colorMat, libfreenect2::Frame::Color);

            listener_.release(frames_);

            std::cout << "Get Color Image" << std::endl;
        }

        void getPointCloud(PointCloudPtr pPc) override {
            listener_.waitForNewFrame(frames_);

            generateCloud(frames_, pPc);

            listener_.release(frames_);

            std::cout << "Get Point Cloud" << std::endl;
        }

        void getAll(ImgMat &colorMat, ImgMat &depthMat, PointCloudPtr pcPtr) override {

            listener_.waitForNewFrame(frames_);

            extractFrame(frames_, colorMat, libfreenect2::Frame::Color);
            extractFrame(frames_, depthMat, libfreenect2::Frame::Depth);

            generateCloud(frames_, pcPtr); // 获取点云
            listener_.release(frames_);

            std::cout << "Get All Image" << std::endl;
        }

    private:
//        bool stop = false;
//        void sigint_handler(int s) {
//            stop = true;
//        }

        void extractFrame(libfreenect2::FrameMap& frameMap, ImgMat& imgMat, libfreenect2::Frame::Type type) {
            libfreenect2::Frame *frame = frameMap[type];

            int TYPE = CV_8UC4;
            switch (frame->format) {
                case libfreenect2::Frame::Format::RGBX :
                    TYPE = CV_8UC4;
                    std::cout << "Frame Format is RGBX" << std::endl;
                    break;
                case libfreenect2::Frame::Format::Float :
                    TYPE = CV_32FC1;
                    std::cout << "Frame Format is Float" << std::endl;
                    break;
                case libfreenect2::Frame::Format::BGRX :
                    TYPE = CV_8UC4;
                    std::cout << "Frame Format is BGRX" << std::endl;
                    break;
                case libfreenect2::Frame::Format::Gray :
                    TYPE = CV_8UC1;
                    std::cout << "Frame Format is Gray" << std::endl;
                default:
                    break;
            }

            cv::Mat tmp(frame->height, frame->width, TYPE, frame->data);

            if (mirror_ == true) {
                cv::flip(tmp, imgMat, 1); //左右翻转
            } else {
                    imgMat = tmp.clone();
            }
        }

        void generateCloud(libfreenect2::FrameMap& frameMap, PointCloudPtr pPc) {

            libfreenect2::Frame *rgb = frameMap[libfreenect2::Frame::Color];
            libfreenect2::Frame *depth = frameMap[libfreenect2::Frame::Depth];

            registration_->apply(rgb, depth, &undistorted_, &registered_, true, &big_mat_, map_);
            const std::size_t w = undistorted_.width;
            const std::size_t h = undistorted_.height;

            cv::Mat tmp_itD0(undistorted_.height, undistorted_.width, CV_8UC4, undistorted_.data);
            cv::Mat tmp_itRGB0(registered_.height, registered_.width, CV_8UC4, registered_.data);

            if (mirror_ == true) {

                cv::flip(tmp_itD0, tmp_itD0, 1);
                cv::flip(tmp_itRGB0, tmp_itRGB0, 1);

            }

            const float *itD0 = (float *) tmp_itD0.ptr();
            const char *itRGB0 = (char *) tmp_itRGB0.ptr();

            pPc = pcl::make_shared<PointCloud>(w, h);

            auto *itP = &pPc->points[0];
            bool is_dense = true;

            for (std::size_t y = 0; y < h; ++y) {

                const unsigned int offset = y * w;
                const float *itD = itD0 + offset;
                const char *itRGB = itRGB0 + offset * 4;
                const float dy = rowmap(y);

                for (std::size_t x = 0; x < w; ++x, ++itP, ++itD, itRGB += 4) {
                    const float depth_value = *itD / 1000.0f;

                    if (!std::isnan(depth_value) && !(std::abs(depth_value) < 0.0001)) {

                        const float rx = colmap(x) * depth_value;
                        const float ry = dy * depth_value;
                        itP->z = depth_value;
                        itP->x = rx;
                        itP->y = ry;

                        itP->b = itRGB[0];
                        itP->g = itRGB[1];
                        itP->r = itRGB[2];
                    } else {
                        itP->z = qnan_;
                        itP->x = qnan_;
                        itP->y = qnan_;

                        itP->b = qnan_;
                        itP->g = qnan_;
                        itP->r = qnan_;
                        is_dense = false;
                    }
                }
            }
            pPc->is_dense = is_dense;
        }


        void prepareMake3D(const libfreenect2::Freenect2Device::IrCameraParams & depth_p) {
            const int w = 512;
            const int h = 424;
            float * pm1 = colmap.data();
            float * pm2 = rowmap.data();
            for(int i = 0; i < w; i++)
            {
                *pm1++ = (i-depth_p.cx + 0.5) / depth_p.fx;
            }
            for (int i = 0; i < h; i++)
            {
                *pm2++ = (i-depth_p.cy + 0.5) / depth_p.fy;
            }
        }

    private:
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

    };
}


#endif //RHCL_CAMERA_HPP
