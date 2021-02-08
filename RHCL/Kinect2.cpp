//
// Created by think on 2021/2/1.
//

#include "Kinect2.hpp"
#include <omp.h>

namespace RHCL {

    Kinect2::Kinect2(Kinect2::Processor p, bool mirror, std::string serial) {
        processorType = p;

        initialize(); //初始化
    }

    Kinect2::~Kinect2() {
        safeRelease(colorFrameReader);
        safeRelease(depthFrameReader);
        safeRelease(infraredFrameReader);
        safeRelease(bodyFrameReader);
        safeRelease(coordinateMapper);
        safeRelease(reconstruction);

        SAFE_FUSION_RELEASE_IMAGE_FRAME(floatDepthImage);
        SAFE_FUSION_RELEASE_IMAGE_FRAME(smoothDepthImage);
        SAFE_FUSION_RELEASE_IMAGE_FRAME(pointCloudImage);
        SAFE_FUSION_RELEASE_IMAGE_FRAME(surfaceImage);

        safeRelease(kinect);
    }

    void Kinect2::initialize() {
        cv::setUseOptimized(true); // 开启CPU指令集优化功能
        std::cout << "CPU optimized status: " << cv::useOptimized() << std::endl; //查询是否开启了CPU指令集优化

        initSensor(); //初始化Kinect设备
        initColor(); //初始化彩色图像
        initDepth(); //初始化深度图像
        initInfrared(); //初始化红外图像
        initBody(); //TODO: 初始化身体图像
//        initPointCloud(); //TODO: 初始化点云
        initFusion();

    }

    void Kinect2::initSensor() {
        GetDefaultKinectSensor(&kinect); //获取默认的Kinect设备
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

    void Kinect2::initColor() {
        IColorFrameSource *colorFrameSource;
        kinect->get_ColorFrameSource(&colorFrameSource);
        colorFrameSource->OpenReader(&colorFrameReader);

        IFrameDescription *colorFrameDescription;
        colorFrameSource->CreateFrameDescription(ColorImageFormat_Bgra, &colorFrameDescription);
        colorFrameDescription->get_Width(&colorWidth);
        colorFrameDescription->get_Height(&colorHeight);
        colorFrameDescription->get_BytesPerPixel(&colorBytesPerPixel);

        safeRelease(colorFrameDescription);

        std::cout << "Color Image:\n" << "Width->" << colorWidth << "; Height->" << colorHeight << "; Bytes Per Pixel->"
                  << colorBytesPerPixel << std::endl;

        colorBuffer.resize(colorWidth * colorHeight * colorBytesPerPixel);
    }

    void Kinect2::initDepth() {
        IDepthFrameSource *depthFrameSource;
        kinect->get_DepthFrameSource(&depthFrameSource);
        depthFrameSource->OpenReader(&depthFrameReader);

        IFrameDescription *depthFrameDescription;
        depthFrameSource->get_FrameDescription(&depthFrameDescription);
        depthFrameDescription->get_Width(&depthWidth);
        depthFrameDescription->get_Height(&depthHeight);
        depthFrameDescription->get_BytesPerPixel(&depthBytesPerPixel);

        safeRelease(depthFrameDescription);

        std::cout << "Depth Image:\n" << "Width->" << depthWidth << "; Height->" << depthHeight << "; Bytes Per Pixel->"
                  << depthBytesPerPixel << std::endl;

        depthBuffer.resize(depthWidth * depthHeight * depthBytesPerPixel);
    }

    void Kinect2::initInfrared() {
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

    void Kinect2::initFusion() {
        reconstructionParameters.voxelsPerMeter = 256; // 128, 256, 384, 512, 640, 768
        reconstructionParameters.voxelCountX = 384; // 128, 256, 384, 512, 640
        reconstructionParameters.voxelCountY = 384; // 128, 256, 384, 512, 640
        reconstructionParameters.voxelCountZ = 384; // 128, 256, 384, 512, 640

        NUI_FUSION_RECONSTRUCTION_PROCESSOR_TYPE p;
        if (processorType == CPU)
            p = NUI_FUSION_RECONSTRUCTION_PROCESSOR_TYPE_CPU;
        else if (processorType == GPU)
            p = NUI_FUSION_RECONSTRUCTION_PROCESSOR_TYPE_AMP; // GPU Process
        else {
            std::cout << "Use Default CPU processor" << std::endl;
            p = NUI_FUSION_RECONSTRUCTION_PROCESSOR_TYPE_CPU;
        }

        // We don't know these at object creation time, so we use nominal values.
        // These will later be updated in response to the CoordinateMappingChanged event.
        cameraParameters.focalLengthX = NUI_KINECT_DEPTH_NORM_FOCAL_LENGTH_X;
        cameraParameters.focalLengthY = NUI_KINECT_DEPTH_NORM_FOCAL_LENGTH_Y;
        cameraParameters.principalPointX = NUI_KINECT_DEPTH_NORM_PRINCIPAL_POINT_X;
        cameraParameters.principalPointY = NUI_KINECT_DEPTH_NORM_PRINCIPAL_POINT_Y;

        HRESULT hr = S_OK;

        WCHAR description[MAX_PATH];
        WCHAR instancePath[MAX_PATH];
        UINT memorySize = 0;

        hr = NuiFusionGetDeviceInfo(p, -1, description, ARRAYSIZE(description), instancePath,
                                    ARRAYSIZE(instancePath), &memorySize);

        if (FAILED(hr)) {
            if (hr == E_NUI_BADINDEX) {
                // This error code is returned either when the device index is out of range for the processor
                // type or there is no DirectX11 capable device installed. As we set -1 (auto-select default)
                // for the device index in the parameters, this indicates that there is no DirectX11 capable
                // device. The options for users in this case are to either install a DirectX11 capable device
                // (see documentation for recommended GPUs) or to switch to non-real-time CPU based
                // reconstruction by changing the processor type to NUI_FUSION_RECONSTRUCTION_PROCESSOR_TYPE_CPU.
                std::cerr
                        << "No DirectX11 device detected, or invalid device index - Kinect Fusion requires a DirectX11 device for GPU-based reconstruction."
                        << std::endl;
            } else {
                std::cout << "Failed in call to NuiFusionGetDeviceInfo." << std::endl;
            }
            return;
        }

        hr = NuiFusionCreateReconstruction(&reconstructionParameters, p, -1, nullptr, &reconstruction);

        if (FAILED(hr)) {
            if (E_NUI_GPU_FAIL == hr) {
                WCHAR buf[MAX_PATH];
                swprintf_s(buf, ARRAYSIZE(buf), L"Device %d not able to run Kinect Fusion, or error initializing.", -1);
                std::cerr << buf << std::endl;
            } else if (E_NUI_GPU_OUTOFMEMORY == hr) {
                WCHAR buf[MAX_PATH];
                swprintf_s(buf, ARRAYSIZE(buf),
                           L"Device %d out of memory error initializing reconstruction - try a smaller reconstruction volume.",
                           -1);
                std::cerr << buf << std::endl;
            } else if (NUI_FUSION_RECONSTRUCTION_PROCESSOR_TYPE_CPU != processorType) {
                WCHAR buf[MAX_PATH];
                swprintf_s(buf, ARRAYSIZE(buf),
                           L"Failed to initialize Kinect Fusion reconstruction volume on device %d.",
                           -1);
                std::cerr << buf << std::endl;
            } else {
                std::cerr << "Failed to initialize Kinect Fusion reconstruction volume on CPU." << std::endl;
            }

            return;
        }

        hr = NuiFusionCreateImageFrame(NUI_FUSION_IMAGE_TYPE_FLOAT, depthWidth, depthHeight, nullptr, &floatDepthImage);
        if (FAILED(hr)) {
            std::cerr << "Failed to initialize Kinect Fusion image." << std::endl;
            return;
        }

        hr = NuiFusionCreateImageFrame(NUI_FUSION_IMAGE_TYPE_FLOAT, depthWidth, depthHeight, nullptr,
                                       &smoothDepthImage);
        if (FAILED(hr)) {
            std::cerr << "Failed to initialize Kinect Fusion image." << std::endl;
            return;
        }

        hr = NuiFusionCreateImageFrame(NUI_FUSION_IMAGE_TYPE_POINT_CLOUD, depthWidth, depthHeight, nullptr,
                                       &pointCloudImage);
        if (FAILED(hr)) {
            std::cerr << "Failed to initialize Kinect Fusion image." << std::endl;
            return;
        }

        hr = NuiFusionCreateImageFrame(NUI_FUSION_IMAGE_TYPE_COLOR, depthWidth, depthHeight, nullptr, &surfaceImage);
        if (FAILED(hr)) {
            std::cerr << "Failed to initialize Kinect Fusion image." << std::endl;
            return;
        }

        cloudMesh.reset(new PointCloudT);
//    cloudMesh->points.resize(depthWidth * depthHeight);


    }

    cv::Mat Kinect2::getColorImage() {
        IColorFrame *colorFrame;
        HRESULT hr = colorFrameReader->AcquireLatestFrame(&colorFrame);

        if (FAILED(hr)) {
            std::cerr << "ColorFrameReader::AcquiredLatestFrame::Failed" << std::endl;
            return cv::Mat();
        }

//        ColorImageFormat colorImageFormat;
//        colorFrame->get_RawColorImageFormat(&colorImageFormat);
//        std::cout << "ColorImageFormat is: " << colorImageFormat;

        colorFrame->CopyConvertedFrameDataToArray(colorBuffer.size(), &colorBuffer[0], ColorImageFormat_Bgra);

        safeRelease(colorFrame); //用后释放资源

        colorMat = cv::Mat(colorHeight, colorWidth, CV_8UC4, &colorBuffer[0]).clone(); //注意：存储形式是BGRA

        return colorMat;
    }

    cv::Mat Kinect2::getDepthImage() {
        IDepthFrame *depthFrame;
        HRESULT hr = depthFrameReader->AcquireLatestFrame(&depthFrame);

        if (FAILED(hr)) {
            std::cerr << "DepthFrameReader::AcquiredLatestFrame::Failed" << std::endl;
            return cv::Mat();
        }

        depthFrame->CopyFrameDataToArray(depthBuffer.size(), &depthBuffer[0]);

        safeRelease(depthFrame);

        depthMat = cv::Mat(depthHeight, depthWidth, CV_16UC1, &depthBuffer[0]).clone();

        return depthMat;
    }

    Kinect2::PointCloudT::Ptr Kinect2::getCloud() {
        getColorImage();
        getDepthImage(); //先获取新的depthBuffer
        std::vector<ColorSpacePoint> colorSpacePoint(depthWidth * depthHeight); //在彩色空间
        coordinateMapper->MapDepthFrameToColorSpace(depthWidth*depthHeight,
                                                    &depthBuffer[0],
                                                     colorSpacePoint.size(),
                                                    &colorSpacePoint[0]);



        std::vector<CameraSpacePoint> cameraSpacePoints(depthWidth * depthHeight);
        coordinateMapper->MapDepthFrameToCameraSpace(depthWidth * depthHeight, &depthBuffer[0],
                                                     cameraSpacePoints.size(), &cameraSpacePoints[0]);
        PointCloudT::Ptr cloud = pcl::make_shared<PointCloudT>();

#pragma omp parallel for
        for (int i = 0; i < cameraSpacePoints.size(); i++) {
            cv::Vec3b color = colorMat.at<cv::Vec3b>(colorSpacePoint[i].Y, colorSpacePoint[i].X);
            cloud->push_back(PointT(cameraSpacePoints[i].X,
                                    cameraSpacePoints[i].Y,
                                    cameraSpacePoints[i].Z,
                                    color[0],
                                    color[1],
                                    color[2],
                                    255));
        }

        return cloud;
        //        return PointCloudT::Ptr();
    }

    Kinect2::PointCloudT::Ptr Kinect2::getFusionCloud() {
        HRESULT hr = reconstruction->DepthToDepthFloatFrame(&depthBuffer[0], depthBuffer.size()*depthBytesPerPixel, floatDepthImage, fMinDepthThreshold, fMaxDepthThreshold, true); //最后一个参数是否镜像
        if (FAILED(hr)) {
            std::cout << "Kinect Fusion NuiFusionDepthToDepthFloatFrame call failed." << std::endl;
            return nullptr;
        }

        hr = reconstruction->SmoothDepthFloatFrame(floatDepthImage, smoothDepthImage,
                                                   NUI_FUSION_DEFAULT_SMOOTHING_KERNEL_WIDTH,
                                                   NUI_FUSION_DEFAULT_SMOOTHING_DISTANCE_THRESHOLD);
        if (FAILED(hr)) {
            std::cout << "Kinect Fusion NuiFusionSmoothDepthFloatFrame call failed." << std::endl;
            return nullptr;
        }

        hr = reconstruction->GetCurrentWorldToCameraTransform(&worldToCameraTransform);
        if (FAILED(hr)) {
            std::cout << "Kinect Fusion NuiFusionSmoothDepthFloatFrame call failed." << std::endl;
            return nullptr;
        }

        /******处理Frame******/
        hr = reconstruction->ProcessFrame(smoothDepthImage, NUI_FUSION_DEFAULT_ALIGN_ITERATION_COUNT,
                                          NUI_FUSION_DEFAULT_INTEGRATION_WEIGHT,
                                          nullptr,
                                          &worldToCameraTransform);

        if (FAILED(hr)) {
            if (hr == E_NUI_FUSION_TRACKING_ERROR) {
                lostFrameCounter++;
                isTrackingFailed = true;
                std::cout << "Kinect Fusion camera tracking failed! Align the camera to the last tracked position. " << std::endl;
            } else {
                std::cout << "Kinect Fusion ProcessFrame call failed!" << std::endl;
                return nullptr;
            }
        } else {
            Matrix4 calculatedCameraPose;
            hr = reconstruction->GetCurrentWorldToCameraTransform(&calculatedCameraPose);

            if (SUCCEEDED(hr)) {
                // Set the pose
                worldToCameraTransform = calculatedCameraPose;
                lostFrameCounter = 0;
                isTrackingFailed = false;
            }
        }

        if (isTrackingFailed && lostFrameCounter >= 100) {

            resetFusionCloud();

            // Set bad tracking message
            std::cout << "Kinect Fusion camera tracking failed, automatically reset volume." << std::endl;
        }

        /*=====计算点云======*/
        hr = reconstruction->CalculatePointCloud(pointCloudImage, &worldToCameraTransform);
        if (FAILED(hr)) {
            std::cout << "Kinect Fusion CalculatePointCloud call failed." << std::endl;
            return nullptr;
        }

        hr = NuiFusionShadePointCloud(pointCloudImage, &worldToCameraTransform, nullptr, surfaceImage, nullptr);
        if (FAILED(hr))
        {
            std::cout << "Kinect Fusion NuiFusionShadePointCloud call failed." << std::endl;
            return nullptr;
        }

        //可以用于显示
        NUI_FUSION_BUFFER* surfaceImageFrameBuffer = surfaceImage->pFrameBuffer;
        surfaceMat = cv::Mat(depthHeight, depthWidth, CV_8UC4, surfaceImageFrameBuffer->pBits);

        reconstruction->CalculateMesh(1, &mesh);
        const Vector3* vertices;
        mesh->GetVertices(&vertices);

        PointCloudT::Ptr fusionCloud = pcl::make_shared<PointCloudT>();

#pragma omp parallel for
        for(int i = 0; i < mesh->VertexCount(); i++) {
            fusionCloud->push_back(PointT(vertices[i].x, vertices[i].y, vertices[i].z, 0, 0, 0, 0xFF));
        }

    }

    void Kinect2::resetFusionCloud() {
        SetIdentityMatrix(worldToCameraTransform);
        HRESULT hr = reconstruction->ResetReconstruction(&worldToCameraTransform, nullptr);

        if (FAILED(hr)) {
            return;
        }

        lostFrameCounter = 0;
    }

    void Kinect2::SetIdentityMatrix(Matrix4 &mat) {
        mat.M11 = 1;  mat.M12 = 0;  mat.M13 = 0;  mat.M14 = 0;
        mat.M21 = 0;  mat.M22 = 1;  mat.M23 = 0;  mat.M24 = 0;
        mat.M31 = 0;  mat.M32 = 0;  mat.M33 = 1;  mat.M34 = 0;
        mat.M41 = 0;  mat.M42 = 0;  mat.M43 = 0;  mat.M44 = 1;
    }

}
