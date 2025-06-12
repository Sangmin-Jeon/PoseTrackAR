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

    std::vector<TouchPoint_C> touchPoints; // 터치로 받은 2D 포인트 (픽셀 좌표)

    std::vector<cv::Point2f> imagePoints;
    std::vector<cv::Point3f> objectPoints;
    bool has_detection = false;
    bool object_points_initialized = false;
};

static ObjState obj_state;

// 3D 모델 포인트 초기화 (바닥점 기준, OpenCV 카메라 좌표계)
// 단위: cm
void setupObjectPoints() {
    obj_state.objectPoints.clear();

    // 예시: 객체의 3D 포인트 정의 (cm 단위)
    // 실제 객체에 맞게 조정 필요
    obj_state.objectPoints = {
        cv::Point3f(0.0f, 0.0f, 0.0f),    // 0: 바닥 중심
        cv::Point3f(0.0f, 5.0f, 0.0f),    // 1: 가슴 부분
        cv::Point3f(-2.5f, 10.0f, 0.0f),  // 2: 왼쪽 눈
        cv::Point3f(2.5f, 10.0f, 0.0f),   // 3: 오른쪽 눈
        cv::Point3f(-5.0f, 15.0f, 0.0f),  // 4: 모자 왼쪽
        cv::Point3f(5.0f, 15.0f, 0.0f),   // 5: 모자 오른쪽
        cv::Point3f(-1.0f, 20.0f, 0.0f),  // 6: 모자 상단 왼쪽
        cv::Point3f(1.0f, 20.0f, 0.0f)    // 7: 모자 상단 오른쪽
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
        // touchPoints는 이미 픽셀 좌표로 저장됨
        float pixelX = obj_state.touchPoints[i].x;
        float pixelY = obj_state.touchPoints[i].y;
        imgPts.emplace_back(pixelX, pixelY);
        objPts.push_back(obj_state.objectPoints[i]);
    }

    // 카메라 매트릭스
    cv::Mat cameraMatrix = (cv::Mat_<double>(3,3) <<
        obj_state.intrinsics.fx, 0, obj_state.intrinsics.cx,
        0, obj_state.intrinsics.fy, obj_state.intrinsics.cy,
        0, 0, 1);
    cv::Mat distCoeffs = cv::Mat::zeros(1,5,CV_64F);

    cv::Mat rvec, tvec;
    bool ok = cv::solvePnP(objPts, imgPts, cameraMatrix, distCoeffs, rvec, tvec, false, cv::SOLVEPNP_ITERATIVE);
    if (!ok) {
        printf("[C++] solvePnP failed\n");
        send_calculate_coordinate_to_swift(0,0,0);
        return;
    }

    // tvec은 cm 단위로 가정
    float x_cm = tvec.at<double>(0);
    float y_cm = tvec.at<double>(1);
    float z_cm = tvec.at<double>(2);
    printf("[C++] PnP: x=%.1fcm y=%.1fcm z=%.1fcm\n", x_cm, y_cm, z_cm);
    send_calculate_coordinate_to_swift(x_cm, y_cm, z_cm);

    // 디버깅: 투영 포인트와 터치 포인트 비교 (픽셀 단위)
    std::vector<cv::Point2f> projectedPts;
    cv::projectPoints(objPts, rvec, tvec, cameraMatrix, distCoeffs, projectedPts);
    for (size_t i = 0; i < N; ++i) {
        float dx = projectedPts[i].x - imgPts[i].x;
        float dy = projectedPts[i].y - imgPts[i].y;
        float error = std::sqrt(dx*dx + dy*dy);
        printf("[C++] Point %zu: projected=(%.1f,%.1f), touch=(%.1f,%.1f), error=%.2f px\n",
               i, projectedPts[i].x, projectedPts[i].y, imgPts[i].x, imgPts[i].y, error);
    }
}

void receive_camera_frame(
      void* baseAddress,
      int width, int height,
      int bytesPerRow,
      const KeyPoints_C* points,
      int pointCount)
{
    // 화면 버퍼 → Mat
    cv::Mat frameBGRA(height, width, CV_8UC4, baseAddress, bytesPerRow);
    obj_state.current_frame = frameBGRA.clone();
    cv::cvtColor(obj_state.current_frame, obj_state.gray_frame, cv::COLOR_BGRA2GRAY);

    // 터치 포인트 갱신 (정규화된 좌표 -> 픽셀 좌표)
    obj_state.touchPoints.clear();
    printf("[C++] receive_camera_frame — pointCount=%d\n", pointCount);
    for (int i = 0; i < pointCount; ++i) {
        // 정규화된 좌표를 픽셀 좌표로 변환
        float pixelX = points[i].x * obj_state.intrinsics.width;
        float pixelY = points[i].y * obj_state.intrinsics.height;
        printf("  Touch[%d]=(%.2f, %.2f) -> Pixel=(%.2f, %.2f)\n", i, points[i].x, points[i].y, pixelX, pixelY);
        obj_state.touchPoints.push_back({pixelX, pixelY});
    }

    // PnP 계산
    cal_pose_coord();

    // Swift로 화면 전송
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
