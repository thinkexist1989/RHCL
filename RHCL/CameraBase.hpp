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

//CameraBase是所有Camera的基类，定义操作接口

#ifndef RHCL_PROJECT_CAMERABASE_HPP
#define RHCL_PROJECT_CAMERABASE_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace RHCL {

    class CameraBase {
    public:
        /** @brief 枚举处理器类型
         * CPU, OPENCL, OPENGL, CUDA
         */
        enum Processor {
            CPU, OPENCL, OPENGL, CUDA
        };

        CameraBase(Processor p = OPENGL, bool mirror = false);

        virtual ~CameraBase() = 0; //纯虚析构函数，必须实现




    };

}

#endif //RHCL_PROJECT_CAMERABASE_HPP
