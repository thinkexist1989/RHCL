//
// Created by think on 2021/1/12.
//

#include "Camera.hpp"

namespace RHCL {
    bool Camera::stop = false;

    Camera::~Camera() {

    }

    Camera::Camera(RHCL::Camera::Processor p, bool mirror, std::string serial) : mirror_(mirror), listener_(
            libfreenect2::Frame::Color | libfreenect2::Frame::Ir | libfreenect2::Frame::Depth),
                                                                                 undistorted_(512, 424, 4),
                                                                                 registered_(512, 424, 4),
                                                                                 big_mat_(1920, 1082, 4),
                                                                                 qnan_(std::numeric_limits<float>::quiet_NaN()) {

        signal(SIGINT, sigint_handler);

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

    }

    void Camera::sigint_handler(int s) {
        stop = true;
    }

    libfreenect2::Freenect2Device::IrCameraParams Camera::getIrParameters() {
        libfreenect2::Freenect2Device::IrCameraParams ir = dev_->getIrCameraParams();
        return ir;
    }

    libfreenect2::Freenect2Device::ColorCameraParams Camera::getRgbParameters() {
        libfreenect2::Freenect2Device::ColorCameraParams rgb = dev_->getColorCameraParams();
        return rgb;
    }

    void Camera::disableLog() {
        logger_ = libfreenect2::getGlobalLogger();
        libfreenect2::setGlobalLogger(nullptr);
    }

    void Camera::enableLog() {
        libfreenect2::setGlobalLogger(logger_);
    }

    void Camera::printParameters() {
        libfreenect2::Freenect2Device::ColorCameraParams cp = getRgbParameters();
        std::cout << "rgb fx=" << cp.fx << ",fy=" << cp.fy <<
                  ",cx=" << cp.cx << ",cy=" << cp.cy << std::endl;
        libfreenect2::Freenect2Device::IrCameraParams ip = getIrParameters();
        std::cout << "ir fx=" << ip.fx << ",fy=" << ip.fy <<
                  ",cx=" << ip.cx << ",cy=" << ip.cy <<
                  ",k1=" << ip.k1 << ",k2=" << ip.k2 << ",k3=" << ip.k3 <<
                  ",p1=" << ip.p1 << ",p2=" << ip.p2 << std::endl;
    }

    void Camera::storeParameters() {
        libfreenect2::Freenect2Device::ColorCameraParams cp = getRgbParameters();
        libfreenect2::Freenect2Device::IrCameraParams ip = getIrParameters();

        cv::Mat rgb = (cv::Mat_<float>(3, 3) << cp.fx, 0, cp.cx, 0, cp.fy, cp.cy, 0, 0, 1);
        cv::Mat depth = (cv::Mat_<float>(3, 3) << ip.fx, 0, ip.cx, 0, ip.fy, ip.cy, 0, 0, 1);
        cv::Mat depth_dist = (cv::Mat_<float>(1, 5) << ip.k1, ip.k2, ip.p1, ip.p2, ip.k3);
        std::cout << "storing " << serial_ << std::endl;
        cv::FileStorage fs("calib_" + serial_ + ".yml", cv::FileStorage::WRITE);

        fs << "CcameraMatrix" << rgb;
        fs << "DcameraMatrix" << depth << "distCoeffs" << depth_dist;

        fs.release();
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr Camera::getCloud() {
        const short w = undistorted_.width;
        const short h = undistorted_.height;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>(w, h));

        return updateCloud(cloud);
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr
    Camera::getCloud(const libfreenect2::Frame *rgb, const libfreenect2::Frame *depth,
                     pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {
        const short w = undistorted_.width;
        const short h = undistorted_.height;
        if (cloud->size() != w * h)
            cloud->resize(w * h);
        return updateCloud(rgb, depth, cloud);
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr Camera::updateCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {
        listener_.waitForNewFrame(frames_);
        libfreenect2::Frame *rgb = frames_[libfreenect2::Frame::Color];
        libfreenect2::Frame *depth = frames_[libfreenect2::Frame::Depth];

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

        pcl::PointXYZRGB *itP = &cloud->points[0];
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
        cloud->is_dense = is_dense;
        listener_.release(frames_);

        return cloud;
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr
    Camera::updateCloud(const libfreenect2::Frame *rgb, const libfreenect2::Frame *depth,
                        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {
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

        pcl::PointXYZRGB *itP = &cloud->points[0];
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
        cloud->is_dense = is_dense;

        return cloud;
    }

    void Camera::shutDown() {
        dev_->stop();
        dev_->close();
    }

    void Camera::mirror() {
        mirror_ != mirror_;
    }

    libfreenect2::SyncMultiFrameListener *Camera::getListener() {
        return &listener_;
    }

    // Use only if you want only depth, else use get(cv::Mat, cv::Mat) to have the images aligned
    void Camera::getDepth(cv::Mat depth_mat) {
        listener_.waitForNewFrame(frames_);
        libfreenect2::Frame *depth = frames_[libfreenect2::Frame::Depth];

        cv::Mat depth_tmp(depth->height, depth->width, CV_32FC1, depth->data);

        if (mirror_ == true) {
            cv::flip(depth_tmp, depth_mat, 1);
        } else {
            depth_mat = depth_tmp.clone();
        }
        listener_.release(frames_);
    }

    void Camera::getIr(cv::Mat ir_mat) {
        listener_.waitForNewFrame(frames_);
        libfreenect2::Frame *ir = frames_[libfreenect2::Frame::Ir];

        cv::Mat ir_tmp(ir->height, ir->width, CV_32FC1, ir->data);

        if (mirror_ == true) {
            cv::flip(ir_tmp, ir_mat, 1);
        } else {
            ir_mat = ir_tmp.clone();
        }
        listener_.release(frames_);
    }

    // Use only if you want only color, else use get(cv::Mat, cv::Mat) to have the images aligned
    void Camera::getColor(cv::Mat &color_mat) {
        listener_.waitForNewFrame(frames_);
        libfreenect2::Frame *rgb = frames_[libfreenect2::Frame::Color];

        cv::Mat tmp_color(rgb->height, rgb->width, CV_8UC4, rgb->data);

        if (mirror_ == true) {
            cv::flip(tmp_color, color_mat, 1);
        } else {
            color_mat = tmp_color.clone();
        }
        listener_.release(frames_);
    }

    // Depth and color are aligned and registered
    void Camera::get(cv::Mat &color_mat,
                     cv::Mat &depth_mat,
                     const bool full_hd,
                     const bool remove_points) {
        listener_.waitForNewFrame(frames_);
        libfreenect2::Frame *rgb = frames_[libfreenect2::Frame::Color];
        libfreenect2::Frame *depth = frames_[libfreenect2::Frame::Depth];

        registration_->apply(rgb, depth, &undistorted_, &registered_, remove_points, &big_mat_, map_);

        cv::Mat tmp_depth(undistorted_.height, undistorted_.width, CV_32FC1, undistorted_.data);
        cv::Mat tmp_color;
        if (full_hd)
            tmp_color = cv::Mat(rgb->height, rgb->width, CV_8UC4, rgb->data);
        else
            tmp_color = cv::Mat(registered_.height, registered_.width, CV_8UC4, registered_.data);

        if (mirror_ == true) {
            cv::flip(tmp_depth, depth_mat, 1);
            cv::flip(tmp_color, color_mat, 1);
        } else {
            color_mat = tmp_color.clone();
            depth_mat = tmp_depth.clone();
        }

        listener_.release(frames_);
    }

    // Depth and color are aligned and registered
    void Camera::get(cv::Mat &color_mat,
                cv::Mat &depth_mat,
                cv::Mat &ir_mat,
                const bool full_hd,
                const bool remove_points) {
        listener_.waitForNewFrame(frames_);
        libfreenect2::Frame *rgb = frames_[libfreenect2::Frame::Color];
        libfreenect2::Frame *depth = frames_[libfreenect2::Frame::Depth];
        libfreenect2::Frame *ir = frames_[libfreenect2::Frame::Ir];

        registration_->apply(rgb, depth, &undistorted_, &registered_, remove_points, &big_mat_, map_);

        cv::Mat tmp_depth(undistorted_.height, undistorted_.width, CV_32FC1, undistorted_.data);
        cv::Mat tmp_color;
        cv::Mat ir_tmp(ir->height, ir->width, CV_32FC1, ir->data);

        if (full_hd)
            tmp_color = cv::Mat(rgb->height, rgb->width, CV_8UC4, rgb->data);
        else
            tmp_color = cv::Mat(registered_.height, registered_.width, CV_8UC4, registered_.data);

        if (mirror_ == true) {
            cv::flip(tmp_depth, depth_mat, 1);
            cv::flip(tmp_color, color_mat, 1);
            cv::flip(ir_tmp, ir_mat, 1);
        } else {
            color_mat = tmp_color.clone();
            depth_mat = tmp_depth.clone();
            ir_mat = ir_tmp.clone();
        }

        listener_.release(frames_);
    }

    // All frame and cloud are aligned. There is a small overhead in the double call to registration->apply which has to be removed
    void Camera::get(cv::Mat &color_mat,
                     cv::Mat &depth_mat,
                     pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                     const bool full_hd,
                     const bool remove_points) {
        listener_.waitForNewFrame(frames_);
        libfreenect2::Frame *rgb = frames_[libfreenect2::Frame::Color];
        libfreenect2::Frame *depth = frames_[libfreenect2::Frame::Depth];

        registration_->apply(rgb, depth, &undistorted_, &registered_, remove_points, &big_mat_, map_);

        cv::Mat tmp_depth(undistorted_.height, undistorted_.width, CV_32FC1, undistorted_.data);
        cv::Mat tmp_color;

        if (full_hd)
            tmp_color = cv::Mat(rgb->height, rgb->width, CV_8UC4, rgb->data);
        else
            tmp_color = cv::Mat(registered_.height, registered_.width, CV_8UC4, registered_.data);

        if (mirror_ == true) {
            cv::flip(tmp_depth, depth_mat, 1);
            cv::flip(tmp_color, color_mat, 1);
        } else {
            color_mat = tmp_color.clone();
            depth_mat = tmp_depth.clone();
        }

        cloud = getCloud(rgb, depth, cloud);
        listener_.release(frames_);
    }

    void Camera::prepareMake3D(const libfreenect2::Freenect2Device::IrCameraParams &depth_p) {
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

}
