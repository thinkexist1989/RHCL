//
// Created by think on 2021/2/1.
//

#ifndef RHCL_PROJECT_KINECT2_H
#define RHCL_PROJECT_KINECT2_H

#include <Kinect.h> // Microsoft K4W Header
#include <NuiKinectFusionApi.h> // Microsoft K4W Fusion Header
// OpenCV
#include <opencv2/opencv.hpp>   //OpenCV 头文件
// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace RHCL {

    class Kinect2 {
        typedef pcl::PointXYZRGBA PointT;
        typedef pcl::PointCloud<PointT> PointCloudT;
    public:
        enum Processor {
            CPU, GPU
        };

        Kinect2(Processor p = GPU, bool mirror = false, std::string serial = std::string());

        ~Kinect2();

        void initialize(); //初始化函数，用于初始化Kinect摄像头




    private:
        void initSensor(); //初始化Kinect传感器，获取kinect传感器接口
        IKinectSensor *kienct; //Kinect传感器接口

        ICoordinateMapper *coordinateMapper;

        /*=====彩色图像======*/
        void initColor(); //初始化彩色摄像头
        void ColorUpdate();

        IColorFrameReader *colorFrameReader; // color frame reader
        std::vector<BYTE> colorBuffer;
        int colorWidth;
        int colorHeight;
        unsigned int colorBytesPerPixel;
        cv::Mat colorMat;

        /*=====深度图像======*/
        void initDepth(); //初始化深度摄像头
        void DepthUpdate();

        IDepthFrameReader *depthFrameReader; // depth frame reader
        std::vector<UINT16> depthBuffer;
        int depthWidth;
        int depthHeight;
        unsigned int depthBytesPerPixel;
        cv::Mat depthMat;

        /*=====红外图像======*/
        void initInfrared(); //初始化红外摄像头
        void InfraredUpdate();

        IInfraredFrameReader *infraredFrameReader;
        std::vector<UINT16> infraredBuffer;
        int infraredWidth;
        int infraredHeight;
        unsigned int infraredBytesPerPixel;
        cv::Mat infraredMat;

        /*=====人体检测======*/
        void initBody(); //初始化身体摄像头
        void BodyUpdate();

        IBodyFrameReader *bodyFrameReader; // body frame reader
        cv::Mat bodyMat;

        /*=====图像空间映射======*/
        void initMapper();

        void MapperUpdate();

        cv::Mat colorMappedMat; //color Mapped Mat
        std::vector<DepthSpacePoint> color2depth; //彩色图像映射到深度空间的点 xy
        std::vector<ColorSpacePoint> depth2color; //深度图像映射到彩色图像的点 xy

        /*=====点云图像======*/
        void initPointCloud();

        void PointCloudUpdate();

        PointCloudT::Ptr cloud; //点云显示

        /*=====Fusion重建图像======*/
        void initFusion();

        void FusionUpdate();

        INuiFusionColorReconstruction *colorReconstruction; //
        INuiFusionReconstruction *reconstruction; //
        INuiFusionMesh *mesh; //

        NUI_FUSION_RECONSTRUCTION_PARAMETERS reconstructionParameters; // 三维重建参数
        NUI_FUSION_CAMERA_PARAMETERS cameraParameters; // 相机内参参数
        NUI_FUSION_RECONSTRUCTION_PROCESSOR_TYPE processorType; //处理器类型
        Matrix4 worldToCameraTransform; //FIXME: 世界到相机坐标系的变换？

        // These parameters are for optionally clipping the input depth image
        float fMinDepthThreshold = NUI_FUSION_DEFAULT_MINIMUM_DEPTH;   // min depth in meters
        float fMaxDepthThreshold = NUI_FUSION_DEFAULT_MAXIMUM_DEPTH;    // max depth in meters

        NUI_FUSION_IMAGE_FRAME *floatDepthImage; //深度浮点图像
        NUI_FUSION_IMAGE_FRAME *smoothDepthImage; //平滑后的深度浮点图像
        NUI_FUSION_IMAGE_FRAME *pointCloudImage; //点云图像
        NUI_FUSION_IMAGE_FRAME *surfaceImage;    //
        cv::Mat surfaceMat;

        int lostFrameCounter = 0;
        bool isTrackingFailed = false;

        void resetReconstruction();

        void SetIdentityMatrix(Matrix4 &mat);

        PointCloudT::Ptr cloudMesh; //重建后的三维点云文件

    };

}

#endif //RHCL_PROJECT_KINECT2_H
