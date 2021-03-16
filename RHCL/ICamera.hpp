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

#ifndef RHCL_ICAMERA_HPP
#define RHCL_ICAMERA_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <opencv2/opencv.hpp>

namespace RHCL {
    class ICamera {
    public:
        typedef pcl::PointXYZRGBA Point;
        typedef pcl::PointCloud<Point> PointCloud;
        typedef PointCloud::Ptr PointCloudPtr;

        typedef cv::Mat ImgMat;

        enum Processor {
            CPU,
            GPU,
            OPENCL,
            OPENGL,
            CUDA
        };

        ICamera(Processor p = CPU, bool mirror = false) : p_(p) {

        }

        virtual ~ICamera() = 0 {} //纯虚析构，多态基类，即使纯虚析构也要有声明，因为子类析构需要调用父类析构函数

        virtual void getDepth(ImgMat &depthMat) = 0;

        virtual void getIr(ImgMat &irMat) = 0;

        virtual void getColor(ImgMat &colorMat) = 0;

        virtual void getPointCloud(PointCloudPtr pPc) = 0;

        virtual void getAll(ImgMat &colorMat, ImgMat &depthMat, PointCloudPtr pcPtr) = 0;


    protected:
        bool mirror_; //是否镜像
        Processor p_; //处理器类型
    };
}

#endif //RHCL_ICAMERA_HPP