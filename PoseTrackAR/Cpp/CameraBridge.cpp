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

// 전역 상태 변수. static으로 선언하여 이 파일 내에서만 접근 가능하도록 합니다.
static struct ObjState {
    Intrinsics_C intrinsics;
    bool has_intrinsics = false;

    cv::Mat current_frame;
    cv::Mat gray_frame;
    
    // --- 참조 데이터 ---
    std::vector<cv::Point2f> reference_pixels;  // 랜드마크의 2D 픽셀 좌표
    std::vector<cv::Point3f> object_3d_points;  // 랜드마크의 3D 실측 좌표
    
    // --- 특징점 데이터 (랜드마크 기반) ---
    cv::Mat reference_gray;
    std::vector<cv::KeyPoint> reference_landmark_keypoints; // 8개 랜드마크의 대표 키포인트
    cv::Mat reference_landmark_descriptors;         // 8개 랜드마크의 대표 기술자 (지문)
    bool reference_initialized = false;
    
    // --- 특징점 검출 및 매칭 ---
    cv::Ptr<cv::ORB> orb_detector;
    cv::Ptr<cv::BFMatcher> matcher;
    
    // --- 상수 ---
    const float MAX_REPROJECTION_ERROR = 8.0f;
    const int MIN_MATCH_FOR_PNP = 4;
} obj_state;


// --- 내부 헬퍼 함수들 ---

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
        {1496.9f, 2757.9f}, {1481.8f, 2425.2f}, { 889.1f, 1965.6f}, {2119.8f, 2004.9f},
        {1221.7f, 1687.4f}, {1660.2f, 1660.2f}, { 964.7f, 1593.6f}, {1920.2f, 1418.3f}
    };
    obj_state.object_3d_points = {
        {0.0f,2.5f,0.0f}, {0.0f,4.5f,0.0f}, {-4.0f,6.6f,0.0f}, {4.0f,6.3f,0.0f},
        {-1.5f,8.5f,0.0f}, {1.5f,8.5f,0.0f}, {-3.5f,9.0f,0.0f}, {4.0f,8.5f,0.0f}
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

// 실제 PnP 계산을 수행하는 메인 함수
void performPoseEstimation() {
    if (!obj_state.reference_initialized || !obj_state.has_intrinsics || obj_state.gray_frame.empty()) {
        send_calculate_coordinate_to_swift(0,0,0);
        return;
    }

    // 1. 현재 프레임에서 특징점 검출
    std::vector<cv::KeyPoint> current_keypoints;
    cv::Mat current_descriptors;
    obj_state.orb_detector->detectAndCompute(obj_state.gray_frame, cv::noArray(), current_keypoints, current_descriptors);

    if (current_keypoints.empty() || obj_state.reference_landmark_descriptors.empty()) {
        send_calculate_coordinate_to_swift(0, 0, 0);
        return;
    }

    // 2. knnMatch와 비율 테스트로 좋은 매칭 필터링
    std::vector<std::vector<cv::DMatch>> knn_matches;
    obj_state.matcher->knnMatch(current_descriptors, obj_state.reference_landmark_descriptors, knn_matches, 2);

    std::vector<cv::DMatch> good_matches;
    const float ratio_thresh = 0.70f;
    for (const auto& match_pair : knn_matches) {
        if (match_pair.size() == 2 && match_pair[0].distance < match_pair[1].distance * ratio_thresh) {
            good_matches.push_back(match_pair[0]);
        }
    }
    
    if (good_matches.size() < 4) { // MIN_MATCH_FOR_PNP
        send_calculate_coordinate_to_swift(0, 0, 0);
        return;
    }
    
    // 3. 2D-3D 대응점 목록 생성
    std::vector<cv::Point3f> matched_3d_points;
    std::vector<cv::Point2f> matched_2d_points;
    std::vector<int> matched_landmark_indices;
    
    for (const auto& match : good_matches) {
        matched_2d_points.push_back(current_keypoints[match.queryIdx].pt);
        matched_3d_points.push_back(obj_state.object_3d_points[match.trainIdx]);
        matched_landmark_indices.push_back(match.trainIdx);
    }
    
    // 4. PnP Ransac 수행
    cv::Mat K = (cv::Mat_<double>(3,3) << obj_state.intrinsics.fx, 0, obj_state.intrinsics.cx, 0, obj_state.intrinsics.fy, obj_state.intrinsics.cy, 0,0,1);
    cv::Mat dc = cv::Mat::zeros(1,5,CV_64F);
    cv::Mat rvec, tvec;
    std::vector<int> inliers;

    bool ok = cv::solvePnPRansac(matched_3d_points,
                                 matched_2d_points,
                                 K, dc, rvec, tvec,
                                 false, 100, 4.0f, 0.99,
                                 inliers,
                                 cv::SOLVEPNP_AP3P);
    
    // 5. 결과 검증 및 시각화
    if (!ok || inliers.size() < 4) { // inlier 개수도 확인
        send_calculate_coordinate_to_swift(0, 0, 0);
        return;
    }
    
    float x = tvec.at<double>(0);
    float y = tvec.at<double>(1);
    float z = tvec.at<double>(2);
    
    // 비현실적인 거리 필터링 (5m 이상이면 무시)
    if (z < 1.0f || z > 500.0f) { // 1cm ~ 5m 범위 밖이면 비정상
        send_calculate_coordinate_to_swift(0, 0, 0);
        return;
    }
    
    // PnP 계산 성공 후, 최종적으로 사용된 'inlier' 점들만 그리기
    std::vector<cv::Point2f> inlier_points;
    std::vector<int> inlier_indices;
    for (int idx : inliers) {
        inlier_points.push_back(matched_2d_points[idx]);
        inlier_indices.push_back(matched_landmark_indices[idx]);
    }
    drawMatchedLandmarks(inlier_points, inlier_indices);

    // 6. 모든 검증을 통과한 경우에만 최종 결과 전송
    printf("[SUCCESS] PnP: x=%.1f, y=%.1f, z=%.1f | Inliers: %zu/%zu\n",
           x, y, z, inliers.size(), matched_2d_points.size());
    send_calculate_coordinate_to_swift(x, y, z);
}

// 키포인트 전부 표시하는 코드
//void performPoseEstimation() {
//    if (!obj_state.reference_initialized || !obj_state.has_intrinsics || obj_state.gray_frame.empty()) {
//        send_calculate_coordinate_to_swift(0,0,0);
//        return;
//    }
//
//    // 1. 현재 프레임에서 ORB 특징점 전체 검출
//    std::vector<cv::KeyPoint> current_keypoints;
//    cv::Mat current_descriptors;
//    obj_state.orb_detector->detectAndCompute(obj_state.gray_frame, cv::noArray(), current_keypoints, current_descriptors);
//
//    if (current_keypoints.empty() || obj_state.reference_landmark_descriptors.empty()) {
//        send_calculate_coordinate_to_swift(0, 0, 0);
//        return;
//    }
//
//    // 2. knnMatch로 매칭 후 비율 테스트
//    std::vector<std::vector<cv::DMatch>> knn_matches;
//    obj_state.matcher->knnMatch(current_descriptors, obj_state.reference_landmark_descriptors, knn_matches, 2);
//
//    std::vector<cv::DMatch> good_matches;
//    const float ratio_thresh = 0.65f; // 더 엄격하게 설정
//    for (const auto& match_pair : knn_matches) {
//        if (match_pair.size() == 2 && match_pair[0].distance < match_pair[1].distance * ratio_thresh) {
//            good_matches.push_back(match_pair[0]);
//        }
//    }
//    
//    // 3. 2D-3D 대응점 생성
//    std::vector<cv::Point3f> matched_3d_points;
//    std::vector<cv::Point2f> matched_2d_points;
//    std::vector<int> matched_landmark_indices;
//    
//    for (const auto& match : good_matches) {
//        matched_2d_points.push_back(current_keypoints[match.queryIdx].pt);
//        matched_3d_points.push_back(obj_state.object_3d_points[match.trainIdx]);
//        matched_landmark_indices.push_back(match.trainIdx);
//    }
//
//    if (matched_2d_points.size() < obj_state.MIN_MATCH_FOR_PNP) {
//        send_calculate_coordinate_to_swift(0, 0, 0);
//        return;
//    }
//    
//    // 4. **개선된 RANSAC 파라미터로 PnP 수행**
//    cv::Mat K = (cv::Mat_<double>(3,3) << obj_state.intrinsics.fx, 0, obj_state.intrinsics.cx,
//                                         0, obj_state.intrinsics.fy, obj_state.intrinsics.cy,
//                                         0, 0, 1);
//    cv::Mat dc = cv::Mat::zeros(1,5,CV_64F);
//    cv::Mat rvec, tvec;
//    std::vector<int> inliers;
//
//    // RANSAC 파라미터 개선
//    bool ok = cv::solvePnPRansac(
//        matched_3d_points, matched_2d_points, K, dc, rvec, tvec,
//        false,                    // useExtrinsicGuess
//        1000,                     // iterationsCount (더 많이)
//        3.0f,                     // reprojectionError (더 엄격하게)
//        0.8,                      // confidence
//        inliers,                  // inliers 인덱스 받기
//        cv::SOLVEPNP_ITERATIVE    // 더 정확한 방법
//    );
//    
//    if (!ok || inliers.size() < 4) { // inlier 개수도 확인
//        send_calculate_coordinate_to_swift(0, 0, 0);
//        return;
//    }
//
//    // 5. **결과 검증 추가**
//    float x = tvec.at<double>(0), y = tvec.at<double>(1), z = tvec.at<double>(2);
//    float distance = sqrt(x*x + y*y + z*z);
//    
//    // 비현실적인 거리 필터링 (예: 5m 이상이면 무시)
//    if (distance > 500.0f) { // 500cm = 5m
//        printf("[WARNING] Unrealistic distance: %.1f cm, rejecting\n", distance);
//        send_calculate_coordinate_to_swift(0, 0, 0);
//        return;
//    }
//    
//    // 6. **재투영 오차 검증**
//    std::vector<cv::Point2f> projected_points;
//    cv::projectPoints(matched_3d_points, rvec, tvec, K, dc, projected_points);
//    
//    float avg_error = 0.0f;
//    for (size_t i = 0; i < inliers.size(); ++i) {
//        int idx = inliers[i];
//        float error = cv::norm(matched_2d_points[idx] - projected_points[idx]);
//        avg_error += error;
//    }
//    avg_error /= inliers.size();
//    
//    if (avg_error > obj_state.MAX_REPROJECTION_ERROR) {
//        printf("[WARNING] High reprojection error: %.2f, rejecting\n", avg_error);
//        send_calculate_coordinate_to_swift(0, 0, 0);
//        return;
//    }
//
//    // 7. 시각화 (inlier만)
//    std::vector<cv::Point2f> inlier_2d_points;
//    std::vector<int> inlier_landmark_indices;
//    for (int idx : inliers) {
//        inlier_2d_points.push_back(matched_2d_points[idx]);
//        inlier_landmark_indices.push_back(matched_landmark_indices[idx]);
//    }
//    drawMatchedLandmarks(inlier_2d_points, inlier_landmark_indices);
//
//    printf("[SUCCESS] PnP: x=%.1f, y=%.1f, z=%.1f (cm) | Inliers: %zu/%zu | Avg Error: %.2f\n",
//           x, y, z, inliers.size(), matched_2d_points.size(), avg_error);
//    send_calculate_coordinate_to_swift(x, y, z);
//}

// ===================================================================
// C-API: Swift와 통신하는 인터페이스 함수들
// ===================================================================
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

// 이하 함수들은 현재 로직에서 사용되지 않지만, Swift와의 연결을 위해 남겨둡니다.
void receive_object_detection_info(DetectionObject_C obj) {}
void reset_pose_tracking() {}
bool is_pose_valid() { return false; }

} // extern "C"
