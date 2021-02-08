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

#ifndef SAFE_DELETE
#define SAFE_DELETE(p) { if (p) { delete (p); (p)=NULL; } }
#endif

#ifndef SAFE_DELETE_ARRAY
#define SAFE_DELETE_ARRAY(p) { if (p) { delete[] (p); (p)=NULL; } }
#endif

#ifndef SAFE_FUSION_RELEASE_IMAGE_FRAME
#define SAFE_FUSION_RELEASE_IMAGE_FRAME(p) { if (p) { static_cast<void>(NuiFusionReleaseImageFrame(p)); (p)=NULL; } }
#endif

    public:
        enum Processor {
            CPU, GPU
        };

        Kinect2(Processor p = GPU, bool mirror = false, std::string serial = std::string());

        ~Kinect2();

        void initialize(); //初始化函数，用于初始化Kinect摄像头

    private:
        Processor processorType; //使用的处理器类型

        /*=====设备======*/
        void initSensor(); //初始化Kinect传感器，获取kinect传感器接口
        IKinectSensor *kinect; //Kinect传感器接口

        ICoordinateMapper *coordinateMapper;

        /*=====彩色图像======*/
        void initColor(); //初始化彩色摄像头
//        void ColorUpdate();

        cv::Mat getColorImage(); //获取彩色图像

        IColorFrameReader *colorFrameReader; // color frame reader
        std::vector<BYTE> colorBuffer;
        int colorWidth;
        int colorHeight;
        unsigned int colorBytesPerPixel;
        cv::Mat colorMat;

        /*=====深度图像======*/
        void initDepth(); //初始化深度摄像头
//        void DepthUpdate();

        cv::Mat getDepthImage(); //获取深度图像

        IDepthFrameReader *depthFrameReader; // depth frame reader
        std::vector<UINT16> depthBuffer;
        int depthWidth;
        int depthHeight;
        unsigned int depthBytesPerPixel;
        cv::Mat depthMat;

        /*=====红外图像======*/
        void initInfrared(); //初始化红外摄像头
        void InfraredUpdate();

        cv::Mat getInfraredImage(); //获取红外图像

        IInfraredFrameReader *infraredFrameReader;
        std::vector<UINT16> infraredBuffer;
        int infraredWidth;
        int infraredHeight;
        unsigned int infraredBytesPerPixel;
        cv::Mat infraredMat;

        /*=====人体检测======*/
        void initBody(); //初始化身体摄像头
        void BodyUpdate();

        cv::Mat getBodyImage(); //获取身体图像

        IBodyFrameReader *bodyFrameReader; // body frame reader
        cv::Mat bodyMat;

        /*=====图像空间映射======*/
//        void initMapper();

//        cv::Mat colorMappedMat; //color Mapped Mat
//        std::vector<DepthSpacePoint> color2depth; //彩色图像映射到深度空间的点 xy
//        std::vector<ColorSpacePoint> depth2color; //深度图像映射到彩色图像的点 xy

        /*=====点云图像======*/
//        void initPointCloud();

        PointCloudT::Ptr getCloud(); // 获取点云

        PointCloudT::Ptr cloud; //点云显示

        /*=====Fusion重建图像======*/
        void initFusion();

        void FusionUpdate();

        PointCloudT::Ptr getFusionCloud(); //获取Kinect Fusion得到的点云
        void resetFusionCloud();
        void SetIdentityMatrix(Matrix4 &mat);


        INuiFusionColorReconstruction *colorReconstruction; //
        INuiFusionReconstruction *reconstruction; //
        INuiFusionMesh *mesh; //

        NUI_FUSION_RECONSTRUCTION_PARAMETERS reconstructionParameters; // 三维重建参数
        NUI_FUSION_CAMERA_PARAMETERS cameraParameters; // 相机内参参数
//        NUI_FUSION_RECONSTRUCTION_PROCESSOR_TYPE processorType; //处理器类型
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


        PointCloudT::Ptr cloudMesh; //重建后的三维点云文件

        // Safe release for interfaces
        template<class Interface>
        inline void safeRelease(Interface *&pInterfaceToRelease) {
            if (pInterfaceToRelease != NULL) {
                pInterfaceToRelease->Release();
                pInterfaceToRelease = NULL;
            }
        }

    };

}

#endif //RHCL_PROJECT_KINECT2_H
