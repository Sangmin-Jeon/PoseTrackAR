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
    
    cv::Mat current_frame;    // 원본 BGRA
    cv::Mat gray_frame;       // PnP용 그레이스케일
    cv::Rect last_bbox;
    bool has_frame = false;

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

// 3) receive_camera_frame: ROI 기반 특징점 검출
void receive_camera_frame(void* baseAddress, int width, int height, int bytesPerRow) {
    cv::Mat matBGRA(height, width, CV_8UC4, baseAddress, bytesPerRow);
    obj_state.current_frame = matBGRA.clone();
    obj_state.has_frame = true;

    // 그레이 스케일 (검출용)
    cv::cvtColor(obj_state.current_frame, obj_state.gray_frame, cv::COLOR_BGRA2GRAY);

    std::vector<cv::Point2f> corners;
    if (obj_state.has_detection) {
        // ROI 안에서만 코너 검출
        cv::Mat roiGray = obj_state.gray_frame(obj_state.last_bbox);
        cv::goodFeaturesToTrack(roiGray, corners, 100, 0.01, 10.0);

        // ROI 좌표계 → 전체 이미지 좌표계
        for (auto& p : corners) {
            p.x += obj_state.last_bbox.x;
            p.y += obj_state.last_bbox.y;
        }
    } else {
        // Detection 전, 전체 이미지에서 검출
        cv::goodFeaturesToTrack(obj_state.gray_frame, corners, 100, 0.01, 10.0);
    }

    // 코너 표시
    for (const auto& pt : corners) {
        cv::circle(obj_state.current_frame, pt, 5, cv::Scalar(0,255,0,255), 2);
    }

    // BGRA → RGBA 없이 BGRA 그대로 전달
    send_processed_frame_to_swift(
        obj_state.current_frame.data,
        obj_state.current_frame.cols,
        obj_state.current_frame.rows,
        static_cast<int>(obj_state.current_frame.step)
    );
}

void cal_pose_coord(struct DetectionObject_C obj) {
    send_calculate_coordinate_to_swift(0.0, 0.0, 0.0);
}

// 객체 정보 - 헤더 파일의 시그니처와 일치하도록 수정
void receive_object_detection_info(struct DetectionObject_C obj) {
    if (!obj_state.has_intrinsics) return;

    int W = obj_state.intrinsics.width;
    int H = obj_state.intrinsics.height;
    // 정규화된 좌표 → 픽셀 좌표로 변환
    obj_state.last_bbox = cv::Rect(
        int(obj.bbox_x * W),
        int(obj.bbox_y * H),
        int(obj.bbox_width  * W),
        int(obj.bbox_height * H)
    ) & cv::Rect(0, 0, W, H);

    obj_state.has_detection = true;
}
