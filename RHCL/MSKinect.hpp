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


#ifndef RHCL_MSKINECT_HPP
#define RHCL_MSKINECT_HPP

#include "ICamera.hpp"

#include <Kinect.h> // Microsoft K4W Header
#include <NuiKinectFusionApi.h> // Microsoft K4W Fusion Header

#include <thread>

namespace RHCL {

    class MSKinect : public ICamera {
    public:
        MSKinect(Processor p = GPU, bool mirror = false, std::string serial = std::string()) :
                ICamera(p, mirror) {

            initialize();
        }

        ~MSKinect() {

            safeRelease(colorFrameReader);
            safeRelease(depthFrameReader);
            safeRelease(infraredFrameReader);
            safeRelease(coordinateMapper);

            safeRelease(kinect);

            std::cout << "Deconstruct MSKinect..." << std::endl;
        }

        void getColor(ImgMat &colorMat) override {
            IColorFrame *colorFrame;
            HRESULT hr = colorFrameReader->AcquireLatestFrame(&colorFrame);

            if (FAILED(hr)) {
                std::cerr << "ColorFrameReader::AcquiredLatestFrame::Failed" << std::endl;
                return;
            }

//        ColorImageFormat colorImageFormat;
//        colorFrame->get_RawColorImageFormat(&colorImageFormat);
//        std::cout << "ColorImageFormat is: " << colorImageFormat;

            colorFrame->CopyConvertedFrameDataToArray(colorBuffer.size(), &colorBuffer[0], ColorImageFormat_Bgra);

            safeRelease(colorFrame); //用后释放资源

            colorMat = cv::Mat(colorHeight, colorWidth, CV_8UC4, &colorBuffer[0]).clone(); //注意：存储形式是BGRA

        }

        void getDepth(ImgMat &depthMat) override {
            IDepthFrame *depthFrame;
            HRESULT hr = depthFrameReader->AcquireLatestFrame(&depthFrame);

            if (FAILED(hr)) {
                std::cerr << "DepthFrameReader::AcquiredLatestFrame::Failed" << std::endl;
                return;
            }

            depthFrame->CopyFrameDataToArray(depthBuffer.size(), &depthBuffer[0]);

            safeRelease(depthFrame);

            depthMat = cv::Mat(depthHeight, depthWidth, CV_16UC1, &depthBuffer[0]).clone();
        }

        void getIr(ImgMat &irMat) override {
            IInfraredFrame *infraredFrame;
            HRESULT hr = infraredFrameReader->AcquireLatestFrame(&infraredFrame);

            if(FAILED(hr)) {
                std::cerr << "InfraredFrameReader::AcquiredLatestFrame::Failed" << std::endl;
                return;
            }

            infraredFrame->CopyFrameDataToArray(infraredBuffer.size(), &infraredBuffer[0]);

            safeRelease(infraredFrame);

            irMat = cv::Mat(infraredHeight, infraredWidth, CV_16UC1, &infraredBuffer[0]).clone();
        }

        void getPointCloud(PointCloudPtr pPc) override {
            cv::Mat mat;
            getColor(mat);
            getDepth(mat); //先获取新的depthBuffer
            std::vector<ColorSpacePoint> colorSpacePoint(depthWidth *depthHeight); //在彩色空间
            coordinateMapper->MapDepthFrameToColorSpace(depthWidth * depthHeight,
                                                        &depthBuffer[0],
                                                        colorSpacePoint.size(),
                                                        &colorSpacePoint[0]);


            std::vector<CameraSpacePoint> cameraSpacePoints(depthWidth *depthHeight);
            coordinateMapper->MapDepthFrameToCameraSpace(depthWidth * depthHeight, &depthBuffer[0],
                                                         cameraSpacePoints.size(), &cameraSpacePoints[0]);
            pPc = pcl::make_shared<PointCloud>();

#pragma omp parallel for
            for (int i = 0; i < cameraSpacePoints.size(); i++) {
                cv::Vec3b color = colorMat.at<cv::Vec3b>(colorSpacePoint[i].Y, colorSpacePoint[i].X);
                pPc->push_back(Point(cameraSpacePoints[i].X,
                                        cameraSpacePoints[i].Y,
                                        cameraSpacePoints[i].Z,
                                        color[0],
                                        color[1],
                                        color[2],
                                        255));
            }

        }

        void getAll(ImgMat &colorMat, ImgMat &depthMat, PointCloudPtr pcPtr) override {
            getColor(colorMat);
            getDepth(depthMat);
            getPointCloud(pcPtr);
        }


    private:
        // Safe release for interfaces
        template<class Interface>
        inline void safeRelease(Interface *&pInterfaceToRelease) {
            if (pInterfaceToRelease != NULL) {
                pInterfaceToRelease->Release();
                pInterfaceToRelease = NULL;
            }
        }

        void initialize() {
            cv::setUseOptimized(true); // 开启CPU指令集优化功能
            std::cout << "CPU optimized status: " << cv::useOptimized() << std::endl; //查询是否开启了CPU指令集优化

            initSensor();

            initColor();
            initDepth();
            initInfrared();

           std::this_thread::sleep_for( std::chrono::seconds( 2 ) );
        }

        void initSensor() {
            GetDefaultKinectSensor(&kinect);
            HRESULT hr = kinect->Open();
            if (FAILED(hr)) {
                std::cerr << "IKinectSensor::FAILED::Can not open Kinect Sensor!" << std::endl;
                return;
            } else {
                BOOLEAN isOpen = FALSE;
                kinect->get_IsOpen(&isOpen);

                if (!isOpen) {
                    std::cerr << "IKinectSensor::FAILED::Can not open Kinect Sensor!" << std::endl;
                    return;
                }
            }

            kinect->get_CoordinateMapper(&coordinateMapper); //获取Coordinate Mapper
        }

        void initColor() {
            IColorFrameSource *colorFrameSource;
            kinect->get_ColorFrameSource(&colorFrameSource);
            colorFrameSource->OpenReader(&colorFrameReader);

            IFrameDescription *colorFrameDescription;
            colorFrameSource->CreateFrameDescription(ColorImageFormat_Bgra, &colorFrameDescription);
            colorFrameDescription->get_Width(&colorWidth);
            colorFrameDescription->get_Height(&colorHeight);
            colorFrameDescription->get_BytesPerPixel(&colorBytesPerPixel);

            safeRelease(colorFrameDescription);

            std::cout << "Color Image:\n" << "Width->" << colorWidth << "; Height->" << colorHeight
                      << "; Bytes Per Pixel->"
                      << colorBytesPerPixel << std::endl;

            colorBuffer.resize(colorWidth * colorHeight * colorBytesPerPixel);
        }

        void initDepth() {
            IDepthFrameSource *depthFrameSource;
            kinect->get_DepthFrameSource(&depthFrameSource);
            depthFrameSource->OpenReader(&depthFrameReader);

            IFrameDescription *depthFrameDescription;
            depthFrameSource->get_FrameDescription(&depthFrameDescription);
            depthFrameDescription->get_Width(&depthWidth);
            depthFrameDescription->get_Height(&depthHeight);
            depthFrameDescription->get_BytesPerPixel(&depthBytesPerPixel);

            safeRelease(depthFrameDescription);

            std::cout << "Depth Image:\n" << "Width->" << depthWidth << "; Height->" << depthHeight
                      << "; Bytes Per Pixel->"
                      << depthBytesPerPixel << std::endl;

            depthBuffer.resize(depthWidth * depthHeight);
        }

        void initInfrared() {
            IInfraredFrameSource *infraredFrameSource;
            kinect->get_InfraredFrameSource(&infraredFrameSource);
            infraredFrameSource->OpenReader(&infraredFrameReader);

            IFrameDescription *infraredFrameDescription;
            infraredFrameSource->get_FrameDescription(&infraredFrameDescription);
            infraredFrameDescription->get_Width(&infraredWidth);
            infraredFrameDescription->get_Height(&infraredHeight);
            infraredFrameDescription->get_BytesPerPixel(&infraredBytesPerPixel);

            safeRelease(infraredFrameDescription);

            std::cout << "Infrared Image:\n" << "Width->" << infraredWidth << "; Height->" << infraredHeight
                      << "; Bytes Per Pixel->"
                      << infraredBytesPerPixel << std::endl;

            infraredBuffer.resize(infraredWidth * infraredHeight * infraredBytesPerPixel);
        }

        void setIdentityMatrix(Matrix4 &mat) {
            mat.M11 = 1;  mat.M12 = 0;  mat.M13 = 0;  mat.M14 = 0;
            mat.M21 = 0;  mat.M22 = 1;  mat.M23 = 0;  mat.M24 = 0;
            mat.M31 = 0;  mat.M32 = 0;  mat.M33 = 1;  mat.M34 = 0;
            mat.M41 = 0;  mat.M42 = 0;  mat.M43 = 0;  mat.M44 = 1;
        }

    private:
        IKinectSensor *kinect = nullptr; //Kinect传感器接口指针
        ICoordinateMapper *coordinateMapper = nullptr; //坐标映射

        IColorFrameReader *colorFrameReader = nullptr; //彩色帧读取器
        IDepthFrameReader *depthFrameReader = nullptr; //深度帧读取器
        IInfraredFrameReader *infraredFrameReader = nullptr; //红外帧读取器

        std::vector<BYTE> colorBuffer;
        int colorWidth;
        int colorHeight;
        unsigned int colorBytesPerPixel;
        cv::Mat colorMat;

        std::vector<UINT16> depthBuffer;
        int depthWidth;
        int depthHeight;
        unsigned int depthBytesPerPixel;
        cv::Mat depthMat;

        std::vector<UINT16> infraredBuffer;
        int infraredWidth;
        int infraredHeight;
        unsigned int infraredBytesPerPixel;
        cv::Mat infraredMat;

        PointCloudPtr pointCloudPtr; //点云指针

//        PointCloudPtr reconstruction; //重建后的三维点云

    }; // class MSKinect

} // namespace RHCL


#endif //RHCL_MSKINECT_HPP
