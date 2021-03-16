//
// Created by think on 2021/3/10.
//
#include <iostream>

#include <CameraFactory.hpp>
#include <thread>

int main(int argc, char** argv){

//    RHCL::ICamera* camera = new RHCL::Freenect(RHCL::ICamera::OPENGL, false);
//    RHCL::ICamera* camera = new RHCL::MSKinect(RHCL::MSKinect::GPU, false);
    auto camera = RHCL::CameraFactory::create();
    RHCL::ICamera::ImgMat mat;
//    freenect.getAll(mat);

//    camera->getDepth(mat);
//    camera->getIr(mat);

    std::cout << "Hello World" << std::endl;


    while(true) {
        camera->getColor(mat);
        cv::imshow("ColorMat", mat);

//        camera->getIr(mat);
//        cv::imshow("IrMat", mat);

        cv::Mat a;
        camera->getDepth(a);
        cv::imshow("DepthMat", a);
//        std::cout << a << std::endl;


        int c = cv::waitKey(1);

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}