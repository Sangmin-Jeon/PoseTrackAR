//
//  CameraBridge.cpp
//  PoseTrackAR
//
//  Created by 전상민 on 6/3/25.
//


#include "CameraBridge.h"
#include <cstdio>
#include <limits>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <vector>
#include <algorithm>

// 전역 상태 변수. static으로 선언하여 이 파일 내에서만 접근 가능하도록
static struct ObjState {
    Intrinsics_C intrinsics;
    bool has_intrinsics = false;

    cv::Mat current_frame;
    cv::Mat gray_frame;
    
    // --- 참조 데이터 ---
    std::vector<cv::Point2f> reference_pixels;
    std::vector<cv::Point3f> object_3d_points;
    
    // --- 특징점 데이터 (랜드마크 기반) ---
    cv::Mat reference_gray;
    std::vector<cv::KeyPoint> reference_landmark_keypoints;
    cv::Mat reference_landmark_descriptors;
    bool reference_initialized = false;
    
    // --- 특징점 검출 및 매칭 ---
    cv::Ptr<cv::ORB> orb_detector;
    cv::Ptr<cv::BFMatcher> matcher;

    // --- YOLO ROI를 위해 추가된 변수 ---
    cv::Rect last_bbox;      // YOLO가 찾은 마지막 바운딩 박스
    bool has_detection = false; // 현재 프레임에 유효한 detection 정보가 있는지 여부
    
    // --- 상수 ---
    const float MAX_REPROJECTION_ERROR = 8.0f;
    const int MIN_MATCH_FOR_PNP = 4;
} obj_state;


// 8개의 랜드마크 위치에서 고유한 descriptor을 생성하는 함수
void setupLandmarkDescriptors() {
    if (obj_state.reference_gray.empty() || !obj_state.orb_detector) {
        printf("[ERROR] Cannot setup landmark descriptors.\n");
        return;
    }
    obj_state.reference_landmark_keypoints.clear();
    obj_state.reference_landmark_descriptors.release();

    for (size_t i = 0; i < obj_state.reference_pixels.size(); ++i) {
        cv::Point2f landmark_pixel = obj_state.reference_pixels[i];
        int roi_size = 80; // ROI 크기를 좀 더 넉넉하게
        cv::Rect roi(landmark_pixel.x - roi_size / 2, landmark_pixel.y - roi_size / 2, roi_size, roi_size);
        roi &= cv::Rect(0, 0, obj_state.reference_gray.cols, obj_state.reference_gray.rows);
        if (roi.width <= 0 || roi.height <= 0) continue;

        cv::Mat roi_gray = obj_state.reference_gray(roi);
        std::vector<cv::KeyPoint> roi_keypoints;
        cv::Mat roi_descriptors;
        obj_state.orb_detector->detectAndCompute(roi_gray, cv::noArray(), roi_keypoints, roi_descriptors);

        if (roi_keypoints.empty()) {
            printf("[WARNING] No keypoints found for landmark %zu.\n", i);
            continue;
        }

        int best_kp_idx = -1;
        float min_dist = std::numeric_limits<float>::max();
        // ROI 중앙에 가장 가까운 키포인트를 대표로 선택 (더 안정적)
        for (size_t j = 0; j < roi_keypoints.size(); ++j) {
            float dist = cv::norm(roi_keypoints[j].pt - cv::Point2f(roi_size/2.0f, roi_size/2.0f));
            if (dist < min_dist) {
                min_dist = dist;
                best_kp_idx = j;
            }
        }
        
        cv::KeyPoint best_kp = roi_keypoints[best_kp_idx];
        best_kp.pt.x += roi.x;
        best_kp.pt.y += roi.y;
        
        obj_state.reference_landmark_keypoints.push_back(best_kp);
        obj_state.reference_landmark_descriptors.push_back(roi_descriptors.row(best_kp_idx));
    }
    printf("[DEBUG] Landmark descriptors setup. Total %zu landmarks ready.\n", obj_state.reference_landmark_keypoints.size());
}

// 2D/3D 포인트 매핑 데이터 설정
void setupReferenceData() {
    obj_state.reference_pixels = {
        {1496.9f, 2757.9f},
        {1481.8f, 2425.2f},
        { 889.1f, 1965.6f},
        {2119.8f, 2004.9f},
        {1221.7f, 1687.4f},
        {1660.2f, 1660.2f},
        { 964.7f, 1593.6f},
        {1920.2f, 1418.3f}
    };
    
    obj_state.object_3d_points = {
        {0.0f,2.5f,0.0f},
        {0.0f,4.5f,0.0f},
        {-4.0f,6.6f,-5.0f},
        {4.0f,6.3f,-5.0f},
        {-1.5f,8.5f,2.0f},
        {1.5f,8.5f,2.0f},
        {-3.5f,9.0f,-3.0f},
        {4.0f,8.5f,-3.0f}
    };
}

// ORB 검출기와 매처 설정
void setupFeatureDetector() {
    obj_state.orb_detector = cv::ORB::create(2000);
    // knnMatch를 사용하기 위해 crossCheck는 false로 설정
    obj_state.matcher = cv::BFMatcher::create(cv::NORM_HAMMING, false);
}

// 화면에 매칭된 랜드마크를 시각화하는 함수
void drawMatchedLandmarks(const std::vector<cv::Point2f>& points, const std::vector<int>& landmark_indices) {
    for(size_t i = 0; i < points.size(); ++i) {
        cv::Point2f pt = points[i];
        int landmark_idx = landmark_indices[i];
        cv::Scalar color((landmark_idx * 40) % 255, (landmark_idx * 80) % 255, 200, 255);
        cv::circle(obj_state.current_frame, pt, 15, color, 3);
        cv::putText(obj_state.current_frame, std::to_string(landmark_idx), pt, cv::FONT_HERSHEY_SIMPLEX, 1.0, color, 2);
    }
}

// PnP 계산을 수행하는 메인 함수
void performPoseEstimation() {
    if (!obj_state.reference_initialized || !obj_state.has_intrinsics || obj_state.gray_frame.empty()) {
        send_calculate_coordinate_to_swift(0,0,0);
        return;
    }

    // #1. YOLO 탐지 여부 확인
    // 만약 YOLO가 객체를 탐지하지 않았다면, PnP 계산을 시도조차 하지 않음
    if (!obj_state.has_detection || obj_state.last_bbox.area() == 0) {
        send_calculate_coordinate_to_swift(0, 0, 0);
        obj_state.has_detection = false; // 다음 프레임을 위해 초기화
        return;
    }

    // #2. ROI 설정 및 특징점 검출
    cv::Mat roi_gray = obj_state.gray_frame(obj_state.last_bbox);
    
    std::vector<cv::KeyPoint> roi_keypoints;
    cv::Mat roi_descriptors;
    // ROI 안에서만 특징점 검출
    obj_state.orb_detector->detectAndCompute(roi_gray, cv::noArray(), roi_keypoints, roi_descriptors);
    
    // 검출 후, 다음 프레임을 위해 탐지 상태를 다시 초기화
    obj_state.has_detection = false;

    if (roi_keypoints.empty()) {
        send_calculate_coordinate_to_swift(0, 0, 0);
        return;
    }
    
    // #3. ROI 내부 좌표를 전체 이미지 좌표로 변환
    std::vector<cv::KeyPoint> current_keypoints = roi_keypoints;
    for(auto& kp : current_keypoints) {
        kp.pt.x += obj_state.last_bbox.x;
        kp.pt.y += obj_state.last_bbox.y;
    }


    // #4. knnMatch (ROI에서 찾은 디스크립터 사용)
    std::vector<std::vector<cv::DMatch>> knn_matches;
    obj_state.matcher->knnMatch(roi_descriptors, obj_state.reference_landmark_descriptors, knn_matches, 2);

    std::vector<cv::DMatch> good_matches;
    const float ratio_thresh = 0.70f;
    for (const auto& match_pair : knn_matches) {
        if (match_pair.size() == 2 && match_pair[0].distance < match_pair[1].distance * ratio_thresh) {
            good_matches.push_back(match_pair[0]);
        }
    }
    
    if (good_matches.size() < 4) {
        send_calculate_coordinate_to_swift(0, 0, 0);
        return;
    }
    
    // #5. 2D-3D 대응점 목록 생성
    std::vector<cv::Point3f> matched_3d_points;
    std::vector<cv::Point2f> matched_2d_points;
    std::vector<int> matched_landmark_indices;
    
    for (const auto& match : good_matches) {
        // 주의: good_matches의 queryIdx는 roi_keypoints의 인덱스이지만,
        // 우리는 current_keypoints (전체 좌표로 보정된)를 사용해야 함
        matched_2d_points.push_back(current_keypoints[match.queryIdx].pt);
        matched_3d_points.push_back(obj_state.object_3d_points[match.trainIdx]);
        matched_landmark_indices.push_back(match.trainIdx);
    }
    
    // #6. PnP Ransac 수행
    cv::Mat K = (cv::Mat_<double>(3,3) << obj_state.intrinsics.fx, 0, obj_state.intrinsics.cx, 0, obj_state.intrinsics.fy, obj_state.intrinsics.cy, 0,0,1);
    cv::Mat dc = cv::Mat::zeros(1,5,CV_64F);
    cv::Mat rvec, tvec;
    std::vector<int> inliers;
    bool ok = cv::solvePnPRansac(
                     matched_3d_points,
                     matched_2d_points,
                     K, dc,
                     rvec, tvec,
                     false,     // useExtrinsicGuess
                     100,       // iterationsCount
                     5.0f,      // reprojectionError
                     0.99,      // confidence
                     inliers,
                     cv::SOLVEPNP_ITERATIVE
                 );
    
    if (!ok || inliers.size() < 4) {
        send_calculate_coordinate_to_swift(0, 0, 0);
        return;
    }
    
    float x = tvec.at<double>(0);
    float y = tvec.at<double>(1);
    float z = tvec.at<double>(2);
    
    if (z < 1.0f || z > 500.0f) {
        send_calculate_coordinate_to_swift(0, 0, 0);
        return;
    }
    
    std::vector<cv::Point2f> inlier_points;
    std::vector<int> inlier_indices;
    for (int idx : inliers) {
        inlier_points.push_back(matched_2d_points[idx]);
        inlier_indices.push_back(matched_landmark_indices[idx]);
    }
    
    drawMatchedLandmarks(inlier_points, inlier_indices);
        
    // #7 Pnp 결과 값 전송
    printf("[SUCCESS] PnP: x=%.1f, y=%.1f, z=%.1f | Inliers: %zu/%zu\n",
           x, y, z, inliers.size(), matched_2d_points.size());
    send_calculate_coordinate_to_swift(x, y, z);
}

// Swift와 통신하는 인터페이스 함수들
extern "C" {

void load_reference_image(const char* imagePath) {
    cv::Mat ref_img = cv::imread(imagePath, cv::IMREAD_COLOR);
    if (ref_img.empty()) {
        printf("[ERROR] Failed to load reference image: %s\n", imagePath);
        return;
    }
    cv::cvtColor(ref_img, obj_state.reference_gray, cv::COLOR_BGR2GRAY);
    
    setupReferenceData();
    setupFeatureDetector();
    setupLandmarkDescriptors(); // 랜드마크 지문 생성
    
    obj_state.reference_initialized = true;
    printf("[DEBUG] Reference image and landmarks initialized.\n");
}

void receive_intrinsics(Intrinsics_C intr) {
    obj_state.intrinsics = intr;
    obj_state.has_intrinsics = true;
}

void receive_camera_frame(void *addr, int w, int h, int rowBytes, const KeyPoints_C *pts, int ptCnt) {
    if (!obj_state.reference_initialized) {
        // 초기화 전이면 아무것도 하지 않음
        cv::Mat empty_frame(h, w, CV_8UC4, cv::Scalar(0,0,0,255));
        send_processed_frame_to_swift(empty_frame.data, w, h, static_cast<int>(empty_frame.step));
        return;
    }
    cv::Mat frameBGRA(h, w, CV_8UC4, addr, rowBytes);
    obj_state.current_frame = frameBGRA.clone();
    cv::cvtColor(obj_state.current_frame, obj_state.gray_frame, cv::COLOR_BGRA2GRAY);
    
    performPoseEstimation();
    
    send_processed_frame_to_swift(obj_state.current_frame.data, w, h, static_cast<int>(obj_state.current_frame.step));
}

// YOLO 객체 추적 정보
void receive_object_detection_info(DetectionObject_C obj) {
    if (!obj_state.has_intrinsics) return;

    // 카메라 해상도 가져오기
    int W = obj_state.intrinsics.width;
    int H = obj_state.intrinsics.height;

    // 정규화된 YOLO 좌표 -> OpenCV 픽셀 좌표(cv::Rect)로 변환
    obj_state.last_bbox = cv::Rect(
        int(obj.bbox_x * W),
        int(obj.bbox_y * H),
        int(obj.bbox_width  * W),
        int(obj.bbox_height * H)
    );
    
    // 바운딩 박스가 이미지 경계를 벗어나지 않도록 보정
    obj_state.last_bbox &= cv::Rect(0, 0, W, H);

    // 탐지 성공 플래그 설정
    obj_state.has_detection = true;
    
    // (디버깅용) 수신된 바운딩 박스를 초록색으로 그리기
    cv::rectangle(obj_state.current_frame, obj_state.last_bbox, cv::Scalar(0, 255, 0, 255), 5);
}

void reset_pose_tracking() {}
bool is_pose_valid() { return false; }

} // extern "C"


/*
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

 */
