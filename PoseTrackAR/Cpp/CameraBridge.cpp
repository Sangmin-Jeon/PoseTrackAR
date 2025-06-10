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
    bool object_points_initialized = false;
};

static ObjState obj_state;

void setupObjectPoints() {
    obj_state.objectPoints.clear();

    // TODO: 측정된 물리값(cm)을 m 단위로 변환
    float bottomH    =  2.5f / 100.0f;  // 바닥 앞중앙 높이: 2.5cm → 0.025m
    float chestH     =  4.7f / 100.0f;  // 가슴 하트 중앙: 4.7cm → 0.047m
    float eyeH       =  9.4f / 100.0f;  // 눈 중심     : 9.4cm → 0.094m
    float brimLeftH  =  6.8f / 100.0f;  // 왼쪽 챙 끝  : 6.8cm → 0.068m
    float brimRightH =  6.5f / 100.0f;  // 오른쪽 챙 끝: 6.5cm → 0.065m
    float hatH       =  8.0f / 100.0f;  // 모자 끝      : 8.0cm → 0.080m

    // 3D 기준점 (X,Y,Z) — X,Y는 모두 0, Z에만 높이 정보
    // 인덱스 순서대로 imagePoints 에도 같은 순서로 대응할 것
    obj_state.objectPoints = {
        { 0.0f,  0.0f, bottomH    },  // 0: 바닥 앞중앙
        { 0.0f,  0.0f, chestH     },  // 1: 가슴 하트 중앙
        { 0.0f,  0.0f, eyeH       },  // 2: 눈 중심
        { 0.0f,  0.0f, brimLeftH  },  // 3: 왼쪽 챙 끝
        { 0.0f,  0.0f, brimRightH },  // 4: 오른쪽 챙 끝
        { 0.0f,  0.0f, hatH       }   // 5: 모자 끝
    };
    
    obj_state.object_points_initialized = true;
    printf("[C++] Object points initialized with %zu points\n", obj_state.objectPoints.size());
}

// 카메라 내부 파라미터
void receive_intrinsics(struct Intrinsics_C intr) {
    printf("[C++] fx: %.2f, fy: %.2f, cx: %.2f, cy: %.2f, width: %d, height: %d\n",
           intr.fx, intr.fy, intr.cx, intr.cy, intr.width, intr.height);
    
    obj_state.intrinsics = intr;
    obj_state.has_intrinsics = true;
    
    // 3D 객체 포인트 초기화
    if (!obj_state.object_points_initialized) {
        setupObjectPoints();
    }
}

void cal_pose_coord() {
    if (!obj_state.has_intrinsics) {
        printf("[C++] No intrinsics available for PnP\n");
        send_calculate_coordinate_to_swift(0.0f, 0.0f, 0.0f);
        return;
    }

    if (!obj_state.object_points_initialized) {
        printf("[C++] Object points not initialized\n");
        send_calculate_coordinate_to_swift(0.0f, 0.0f, 0.0f);
        return;
    }

    // 1) 2D–3D 대응점 개수 맞춰서 준비
    size_t N = std::min(obj_state.imagePoints.size(), obj_state.objectPoints.size());

    // 2) 적어도 4개 이상 있어야 PnP가 동작
    if (N < 4) {
        printf("[C++] Not enough points for PnP: %zu (need at least 4)\n", N);
        send_calculate_coordinate_to_swift(0.0f, 0.0f, 0.0f);
        return;
    }

    std::vector<cv::Point2f> imgPts(obj_state.imagePoints.begin(),
                                   obj_state.imagePoints.begin() + N);
    std::vector<cv::Point3f> objPts(obj_state.objectPoints.begin(),
                                   obj_state.objectPoints.begin() + N);

    // 3) 카메라 매트릭스/왜곡 계수 설정
    cv::Mat K = (cv::Mat_<double>(3,3) <<
        obj_state.intrinsics.fx, 0, obj_state.intrinsics.cx,
        0, obj_state.intrinsics.fy, obj_state.intrinsics.cy,
        0, 0, 1);
    cv::Mat dist = cv::Mat::zeros(1,5,CV_64F);

    // 4) solvePnP 호출
    cv::Mat rvec, tvec;
    bool ok = cv::solvePnP(
        objPts, imgPts, K, dist,
        rvec, tvec,
        false, cv::SOLVEPNP_ITERATIVE
    );
    if (!ok) {
        printf("[C++] solvePnP failed\n");
        send_calculate_coordinate_to_swift(0.0f, 0.0f, 0.0f);
        return;
    }

    // 5) 정상시 좌표 전송
    printf("[C++] PnP result: x=%.3f, y=%.3f, z=%.3f\n",
           tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));
    
    send_calculate_coordinate_to_swift(
        static_cast<float>(tvec.at<double>(0)),
        static_cast<float>(tvec.at<double>(1)),
        static_cast<float>(tvec.at<double>(2))
    );
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

    // 코너를 imagePoints로 사용 (최대 6개까지만)
    obj_state.imagePoints.clear();
    size_t maxPoints = std::min(corners.size(), obj_state.objectPoints.size());
    for (size_t i = 0; i < maxPoints; ++i) {
        obj_state.imagePoints.push_back(corners[i]);
    }

    // 코너 표시
    for (const auto& pt : corners) {
        cv::circle(obj_state.current_frame, pt, 5, cv::Scalar(0,255,0,255), 2);
    }
    
    // PnP 계산 시도
    cal_pose_coord();

    // BGRA → RGBA 없이 BGRA 그대로 전달
    send_processed_frame_to_swift(
        obj_state.current_frame.data,
        obj_state.current_frame.cols,
        obj_state.current_frame.rows,
        static_cast<int>(obj_state.current_frame.step)
    );
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
    printf("[C++] Detection received: bbox(%d,%d,%d,%d)\n",
           obj_state.last_bbox.x, obj_state.last_bbox.y,
           obj_state.last_bbox.width, obj_state.last_bbox.height);
}
