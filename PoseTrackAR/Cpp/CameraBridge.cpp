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
                     false,
                     100, 4.0f, 0.99,
                     inliers,
                     cv::SOLVEPNP_AP3P
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
