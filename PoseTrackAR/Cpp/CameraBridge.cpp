//
//  CameraBridge.cpp
//  PoseTrackAR
//
//  Created by 전상민 on 6/3/25.
//


#include <stdio.h>
#include "CameraBridge.h"

#include <opencv2/opencv.hpp>

// TouchPoint 을 그대로 저장할 수 있도록
typedef KeyPoints_C TouchPoint_C;


struct ObjState {
    Intrinsics_C intrinsics;
    bool has_intrinsics = false;
    
    cv::Mat current_frame;    // 원본 BGRA
    cv::Mat gray_frame;       // PnP용 그레이스케일
    cv::Rect last_bbox;
    bool has_frame = false;

    std::vector<KeyPoints_C> touchPoints; // 터치로 받은 2D 포인트

    std::vector<cv::Point2f> imagePoints;
    std::vector<cv::Point3f> objectPoints;
    bool has_detection = false;
    bool object_points_initialized = false;
};

static ObjState obj_state;

// 3D 모델 포인트 초기화 (바닥점 기준, OpenCV 카메라 좌표계)
void setupObjectPoints() {
    obj_state.objectPoints.clear();

    float bottomY =  3.5f, bottomZ =  2.35f;
    float chestY  =  2.3f, chestZ  =  4.3f;
    float eyeY    =  2.5f, eyeDist =  2.7f;
    float brimY   =  2.5f, brimZ   =  6.5f, brimDist = 8.5f;
    float hatY    =  0.5f, hatZ    = 12.4f;
    float halfEye  = eyeDist * 0.5f;
    float halfBrim = brimDist * 0.5f;

    // 바닥(0,0,0) 기준 상대 좌표
    float relY0 = 0.0f,                    relZ0 = 0.0f;
    float relY1 = chestY - bottomY,        relZ1 = chestZ - bottomZ;
    float relY2 = eyeY   - bottomY,        relZ2 = 8.8f - bottomZ;
    float relY3 = eyeY   - bottomY,        relZ3 = 9.0f - bottomZ;
    float relY4 = brimY  - bottomY,        relZ4 = brimZ - bottomZ;
    float relY5 = brimY  - bottomY,        relZ5 = brimZ - bottomZ;
    float relY6 = hatY   - bottomY,        relZ6 = hatZ - bottomZ;
    float relY7 = hatY   - bottomY,        relZ7 = hatZ - bottomZ;

    // OpenCV 카메라 좌표계로 변환: (X→X, Y→–Z, Z→Y)
    auto toCV = [&](float x, float y, float z){
        return cv::Point3f(x, -z, y);
    };

    obj_state.objectPoints = {
        toCV(   0.0f, relY0, relZ0),
        toCV(   0.0f, relY1, relZ1),
        toCV(-halfEye, relY2, relZ2),
        toCV( halfEye, relY3, relZ3),
        toCV(-halfBrim, relY4, relZ4),
        toCV( halfBrim, relY5, relZ5),
        toCV(-1.7f,    relY6, relZ6),
        toCV( 2.8f,    relY7, relZ7)
    };

    obj_state.object_points_initialized = true;
    printf("[C++] ObjectPoints initialized: %zu pts\n", obj_state.objectPoints.size());
}

void receive_intrinsics(Intrinsics_C intr) {
    obj_state.intrinsics = intr;
    obj_state.has_intrinsics = true;
    printf("[C++] Intrinsics received: fx=%.2f fy=%.2f cx=%.2f cy=%.2f w=%d h=%d\n",
           intr.fx, intr.fy, intr.cx, intr.cy, intr.width, intr.height);
    if (!obj_state.object_points_initialized) {
        setupObjectPoints();
    }
}

void cal_pose_coord() {
    if (!obj_state.has_intrinsics || !obj_state.object_points_initialized) {
        send_calculate_coordinate_to_swift(0,0,0);
        return;
    }

    size_t P = obj_state.touchPoints.size();
    size_t M = obj_state.objectPoints.size();
    size_t N = std::min(P, M);
    if (N != 8) {
        printf("[C++] Not enough points for PnP: %zu (need ≥4)\n", N);
        send_calculate_coordinate_to_swift(0,0,0);
        return;
    }

    // 2D 및 3D 대응점 준비
    std::vector<cv::Point2f> imgPts;
    std::vector<cv::Point3f> objPts;
    imgPts.reserve(N);
    objPts.reserve(N);
    for (size_t i = 0; i < N; ++i) {
        imgPts.emplace_back(obj_state.touchPoints[i].x,
                            obj_state.touchPoints[i].y);
        objPts.push_back(obj_state.objectPoints[i]);
    }

    // 카메라 매트릭스
    cv::Mat K = (cv::Mat_<double>(3,3) <<
        obj_state.intrinsics.fx, 0, obj_state.intrinsics.cx,
        0, obj_state.intrinsics.fy, obj_state.intrinsics.cy,
        0, 0, 1);
    cv::Mat dist = cv::Mat::zeros(1,5,CV_64F);

    cv::Mat rvec, tvec;
    bool ok = cv::solvePnP(objPts, imgPts, K, dist, rvec, tvec, false, cv::SOLVEPNP_ITERATIVE);
    if (!ok) {
        printf("[C++] solvePnP failed\n");
        send_calculate_coordinate_to_swift(0,0,0);
        return;
    }

    float x_cm = tvec.at<double>(0)*100.0f;
    float y_cm = tvec.at<double>(1)*100.0f;
    float z_cm = tvec.at<double>(2)*100.0f;
    printf("[C++] PnP: x=%.1fcm y=%.1fcm z=%.1fcm\n", x_cm, y_cm, z_cm);
    send_calculate_coordinate_to_swift(x_cm, y_cm, z_cm);
}

void receive_camera_frame(
      void* baseAddress,
      int width, int height,
      int bytesPerRow,
      const KeyPoints_C* points,
      int pointCount)
{
    // 1) 화면 버퍼 → Mat
    cv::Mat frameBGRA(height, width, CV_8UC4, baseAddress, bytesPerRow);
    obj_state.current_frame = frameBGRA.clone();
    cv::cvtColor(obj_state.current_frame, obj_state.gray_frame, cv::COLOR_BGRA2GRAY);

    // 2) 터치 포인트 갱신
    obj_state.touchPoints.clear();
    printf("[C++] receive_camera_frame — pointCount=%d\n", pointCount);
    for (int i = 0; i < pointCount; ++i) {
        printf("  Touch[%d]=(% .2f, % .2f)\n", i, points[i].x, points[i].y);
        obj_state.touchPoints.push_back(points[i]);
    }

    // 3) PnP 계산
    cal_pose_coord();

    // 4) Swift로 화면 전송
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
