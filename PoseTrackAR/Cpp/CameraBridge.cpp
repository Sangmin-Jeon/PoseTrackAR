//
//  CameraBridge.cpp
//  PoseTrackAR
//
//  Created by 전상민 on 6/3/25.
//

#include <stdio.h>
#include "CameraBridge.h"

#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>

struct ObjState {
    Intrinsics_C intrinsics;
    bool has_intrinsics = false;

    std::vector<cv::Point2f> imagePoints;
    std::vector<cv::Point3f> objectPoints;
    bool has_detection = false;
};

static ObjState obj_state;

// 카메라 내부 파라미터
void receive_intrinsics(struct Intrinsics_C intr) {
    printf("[C++] fx: %.2f, fy: %.2f, cx: %.2f, cy: %.2f, width: %d, height: %d\n",
           intr.fx, intr.fy, intr.cx, intr.cy, intr.width, intr.height);
    
    obj_state.intrinsics = intr;
    obj_state.has_intrinsics = true;
    
}


// 카메라 이미지
void receive_camera_frame(void* baseAddress, int width, int height, int bytesPerRow) {
    // 입력: BGRA 포맷
    cv::Mat matBGRA(height, width, CV_8UC4, baseAddress, bytesPerRow);
    
    std::cout << "[C++] 이미지 사이즈" << matBGRA.size << std::endl;
    
}


void cal_pose_coord(struct DetectionObject_C obj) {
    
    send_calculate_coordinate_to_swift(0.0, 0.0, 0.0);
    
}

// 객체 정보
void receive_object_detection_info(struct DetectionObject_C obj) {
    std::cout << "[C++] 객체 정보: " << obj.label << std::endl;

    // 내부 파라미터가 설정되어 있어야 진행 가능
    if (!obj_state.has_intrinsics) {
        std::cerr << "[C++] Intrinsics not received yet.\n";
        return;
    }
    
    cal_pose_coord(obj);
    
}
