//
// Created by think on 2021/2/1.
//

#include "Kinect2.hpp"

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
        initMapper();
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

        if(FAILED(hr)) {
            std::cerr << "DepthFrameReader::AcquiredLatestFrame::Failed" << std::endl;
            return cv::Mat();
        }

        depthFrame->CopyFrameDataToArray(depthBuffer.size(), &depthBuffer[0]);

        safeRelease(depthFrame);

        depthMat = cv::Mat(depthHeight, depthWidth, CV_16UC1, &depthBuffer[0]).clone();

        return depthMat;
    }


}
