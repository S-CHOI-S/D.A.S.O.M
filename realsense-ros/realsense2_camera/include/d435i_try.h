#ifndef D435I_TRY_H
#define D435I_TRY_H

#include <librealsense2/rs.hpp>
#include <../include/base_realsense_node.h>

class MyRealSenseNode : public realsense2_camera::BaseRealSenseNode
{
 public:
    MyRealSenseNode(ros::NodeHandle& nodeHandle, ros::NodeHandle& privateNodeHandle,
                    rs2::device dev, const std::string& serial_no);
    ~MyRealSenseNode();

        //: realsense2_camera::BaseRealSenseNode(nodeHandle, privateNodeHandle, dev, serial_no)
//    {
        // 추가로 필요한 초기화 작업이나 설정을 수행할 수 있음
//    }

    // 필요한 추가 함수를 정의할 수 있음
};

#endif // D435I_TRY_H
