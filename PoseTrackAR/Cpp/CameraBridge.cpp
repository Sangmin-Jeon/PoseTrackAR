//
//  CameraBridge.cpp
//  PoseTrackAR
//
//  Created by 전상민 on 6/3/25.
//


#include "CameraBridge.h"
#include <cstdio>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <vector>
#include <algorithm>

struct ObjState {
    Intrinsics_C intrinsics;
    bool has_intrinsics = false;

    cv::Mat current_frame; // 원본 BGRA
    cv::Mat gray_frame;    // 처리용 그레이스케일
    
    // 참조 이미지 및 특징점 정보
    cv::Mat reference_image;
    cv::Mat reference_gray;
    std::vector<cv::KeyPoint> reference_keypoints;
    cv::Mat reference_descriptors;
    bool reference_loaded = false;
    
    cv::Rect last_bbox; // 바운딩 박스 정보
    bool has_detection = false;
    
    // 미리 정의된 참조 이미지의 픽셀 좌표와 3D 좌표 매칭
    std::vector<cv::Point2f> reference_pixels;  // 참조 이미지에서의 픽셀 좌표
    std::vector<cv::Point3f> object_3d_points;  // 대응하는 실측 3D 좌표
    
    // ORB 특징점 추출기
    cv::Ptr<cv::ORB> orb_detector;
    cv::Ptr<cv::BFMatcher> matcher;
    
    // PnP 결과
    cv::Mat last_rvec, last_tvec;
    bool has_valid_pose = false;
    float last_reprojection_error = 0.0f;
    
    const float MAX_REPROJECTION_ERROR = 10.0f;
    const int MIN_MATCH_COUNT = 10;
};

static ObjState obj_state;

void setupReferenceData() {
    std::printf("[C++] setupReferenceData() called\n");
    
    // 참조 데이터 초기화 - 당신이 제공한 실제 데이터 사용
    obj_state.reference_pixels.clear();
    obj_state.object_3d_points.clear();
    
    // 참조 이미지에서의 픽셀 좌표 (touch 좌표 사용)
    obj_state.reference_pixels = {
        {1496.9f, 2757.9f},  // 0: 바닥 중심
        {1481.8f, 2425.2f},  // 1: 가슴 부분
        { 889.1f, 1965.6f},  // 2: 왼쪽 눈
        {2119.8f, 2004.9f},  // 3: 오른쪽 눈
        {1221.7f, 1687.4f},  // 4: 모자 왼쪽
        {1660.2f, 1660.2f},  // 5: 모자 오른쪽
        { 964.7f, 1593.6f},  // 6: 모자 상단 왼쪽
        {1920.2f, 1418.3f}   // 7: 모자 상단 오른쪽
    };
    
    // 대응하는 실측 3D 좌표
    obj_state.object_3d_points = {
        {0.0f, 2.5f, 0.0f},   // 0: 바닥 중심
        {0.0f, 4.5f, 0.0f},   // 1: 가슴 부분
        {-4.0f, 6.6f, 0.0f},  // 2: 왼쪽 눈
        {4.0f, 6.3f, 0.0f},   // 3: 오른쪽 눈
        {-1.5f, 8.5f, 0.0f},  // 4: 모자 왼쪽
        {1.5f, 8.5f, 0.0f},   // 5: 모자 오른쪽
        {-3.5f, 9.0f, 0.0f},  // 6: 모자 상단 왼쪽
        {4.0f, 8.5f, 0.0f}    // 7: 모자 상단 오른쪽
    };
    
    std::printf("[C++] Reference data initialized: %zu point pairs\n",
                obj_state.reference_pixels.size());
    
    // 초기화 확인
    for (size_t i = 0; i < obj_state.reference_pixels.size(); ++i) {
        std::printf("[C++] Initialized ref_pixel[%zu]: (%.1f, %.1f) -> 3D(%.1f, %.1f, %.1f)\n",
                   i, obj_state.reference_pixels[i].x, obj_state.reference_pixels[i].y,
                   obj_state.object_3d_points[i].x, obj_state.object_3d_points[i].y, obj_state.object_3d_points[i].z);
    }
}

void setupFeatureDetector() {
    // 1) ORB를 한 번에 원하는 파라미터로 생성
    //    nfeatures=1000, scaleFactor=1.2, nlevels=8, edgeThreshold=31,
    //    firstLevel=0, WTA_K=2, scoreType=HARRIS_SCORE, patchSize=31, fastThreshold=20
    obj_state.orb_detector = cv::ORB::create(
        1000,         // 최대 추출할 키포인트 개수
        1.2f,         // 스케일 팩터
        8,            // 피라미드 레벨
        31,           // 엣지 임계값
        0,            // 첫 번째 레벨 인덱스
        2,            // WTA_K (여기서 반드시 2,3,4 중 하나)
        cv::ORB::HARRIS_SCORE,
        31,           // 패치 크기
        20            // FAST 임계값
    );

    // 2) BFMatcher 생성 (Hamming 거리 + 교차 검사)
    obj_state.matcher = cv::BFMatcher::create(cv::NORM_HAMMING, true);

    std::printf("[C++] ORB created with WTA_K=2 and BFMatcher initialized\n");
}

bool loadReferenceImage(const std::string& imagePath) {
    obj_state.reference_image = cv::imread(imagePath, cv::IMREAD_COLOR);
    if (obj_state.reference_image.empty()) {
        std::printf("[C++] Failed to load reference image: %s\n", imagePath.c_str());
        return false;
    }
    
    cv::cvtColor(obj_state.reference_image, obj_state.reference_gray, cv::COLOR_BGR2GRAY);
    
    // ORB 검출기가 초기화되지 않았다면 초기화
    if (!obj_state.orb_detector) {
        setupFeatureDetector();
    }
    
    // 참조 데이터가 초기화되지 않았다면 초기화
    if (obj_state.reference_pixels.empty()) {
        setupReferenceData();
    }
    
    // 참조 이미지에서 특징점 추출
    obj_state.orb_detector->detectAndCompute(
        obj_state.reference_gray,
        cv::noArray(),
        obj_state.reference_keypoints,
        obj_state.reference_descriptors
    );
    
    obj_state.reference_loaded = true;
    
    std::printf("[C++] Reference image loaded: %dx%d, %저희 keypoints\n",
                obj_state.reference_image.cols, obj_state.reference_image.rows,
                obj_state.reference_keypoints.size());
    
    // 참조 데이터 검증
    std::printf("[C++] Reference pixels count: %zu\n", obj_state.reference_pixels.size());
    for (size_t i = 0; i < obj_state.reference_pixels.size(); ++i) {
        std::printf("[C++] ref_pixel[%zu]: (%.1f, %.1f)\n",
                   i, obj_state.reference_pixels[i].x, obj_state.reference_pixels[i].y);
    }
    
    return true;
}

void receive_intrinsics(Intrinsics_C intr) {
    obj_state.intrinsics = intr;
    obj_state.has_intrinsics = true;

    std::printf("[C++] Intrinsics received: "
                "fx=%.2f fy=%.2f cx=%.2f cy=%.2f w=%d h=%d\n",
                intr.fx, intr.fy, intr.cx, intr.cy, intr.width, intr.height);

    // 참조 데이터 초기화 (항상 실행)
    // setupReferenceData();
    
    // 특징점 검출기 초기화 (항상 실행)
    // setupFeatureDetector();
    
    std::printf("[C++] Initialization completed in receive_intrinsics\n");
}

std::vector<cv::Point2f> findCorrespondingPoints(
    const std::vector<cv::DMatch>& good_matches,
    const std::vector<cv::KeyPoint>& current_keypoints,
    std::vector<cv::Point3f>& matched_3d_points
) {
    std::vector<cv::Point2f> matched_2d_points;
    matched_3d_points.clear();
    
    const float PROXIMITY_THRESHOLD = 30.0f; // 증가된 임계값
    
    // 디버깅: reference_pixels 상태 확인
    std::printf("[C++] === DEBUG INFO ===\n");
    std::printf("[C++] reference_pixels.size(): %zu\n", obj_state.reference_pixels.size());
    std::printf("[C++] object_3d_points.size(): %zu\n", obj_state.object_3d_points.size());
    
    if (obj_state.reference_pixels.empty()) {
        std::printf("[C++] ERROR: reference_pixels is empty!\n");
        return matched_2d_points;
    }
    
    // reference_pixels 내용 출력 (첫 번째 매치에서만)
    static bool debug_printed = false;
    if (!debug_printed) {
        for (size_t i = 0; i < obj_state.reference_pixels.size(); ++i) {
            std::printf("[C++] reference_pixels[%zu]: (%.1f, %.1f)\n",
                       i, obj_state.reference_pixels[i].x, obj_state.reference_pixels[i].y);
        }
        debug_printed = true;
    }
    std::printf("[C++] ==================\n");
    
    for (const auto& match : good_matches) {
        // 현재 프레임의 매칭된 키포인트
        cv::Point2f current_pt = current_keypoints[match.queryIdx].pt;
        
        // 참조 이미지의 매칭된 키포인트
        cv::Point2f ref_pt = obj_state.reference_keypoints[match.trainIdx].pt;
        
        // 참조 이미지의 키포인트와 가장 가까운 사전 정의된 포인트 찾기
        float min_dist = std::numeric_limits<float>::max();
        int best_idx = -1;
        
        for (size_t i = 0; i < obj_state.reference_pixels.size(); ++i) {
            float dist = cv::norm(ref_pt - obj_state.reference_pixels[i]);
            if (dist < min_dist) {
                min_dist = dist;
                best_idx = static_cast<int>(i);
            }
        }
        
        if (best_idx >= 0 && min_dist < PROXIMITY_THRESHOLD) {
            matched_2d_points.push_back(current_pt);
            matched_3d_points.push_back(obj_state.object_3d_points[best_idx]);
            
            std::printf("[C++] ✓ Match found: ref(%.1f,%.1f) -> current(%.1f,%.1f) -> 3D(%d) dist=%.2f\n",
                       ref_pt.x, ref_pt.y, current_pt.x, current_pt.y, best_idx, min_dist);
        }
    }

    std::printf("[C++] Final matched points: %zu\n", matched_2d_points.size());
    return matched_2d_points;
}

void drawMatchedKeypoints(cv::Mat& image,
                          const std::vector<cv::KeyPoint>& keypoints,
                          const std::vector<cv::DMatch>& matches)
{
    std::vector<cv::KeyPoint> matched_keypoints;
    for (const auto& match : matches) {
        matched_keypoints.push_back(keypoints[match.queryIdx]);
    }
    cv::drawKeypoints(image, matched_keypoints, image, cv::Scalar(0, 0, 255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
}

void performFeatureMatching() {
    if (!obj_state.reference_loaded) {
        std::printf("[C++] ERROR: reference image not loaded!\n");
        send_calculate_coordinate_to_swift(0,0,0);
        return;
    }
    
    if (!obj_state.has_intrinsics) {
        std::printf("[C++] ERROR: intrinsics not received!\n");
        send_calculate_coordinate_to_swift(0, 0, 0);
        return;
    }
    
    // ORB 검출기 초기화 확인
    if (!obj_state.orb_detector) {
        std::printf("[C++] ERROR: Feature detector not initialized! Trying to reinitialize...\n");
        setupFeatureDetector();
        
        if (!obj_state.orb_detector) {
            std::printf("[C++] ERROR: Failed to initialize feature detector!\n");
            send_calculate_coordinate_to_swift(0,0,0);
            return;
        }
    }
    
    // gray_frame 유효성 검사
    if (obj_state.gray_frame.empty()) {
        std::printf("[C++] ERROR: gray_frame is empty!\n");
        send_calculate_coordinate_to_swift(0,0,0);
        return;
    }
    
    if (obj_state.gray_frame.type() != CV_8UC1) {
        std::printf("[C++] ERROR: gray_frame has wrong type: %d (expected CV_8UC1=%d)\n",
                   obj_state.gray_frame.type(), CV_8UC1);
        send_calculate_coordinate_to_swift(0,0,0);
        return;
    }
    
    std::printf("[C++] gray_frame info: %dx%d, type=%d, channels=%d\n",
                obj_state.gray_frame.cols, obj_state.gray_frame.rows,
                obj_state.gray_frame.type(), obj_state.gray_frame.channels());
    
    // 현재 프레임에서 특징점 추출
    std::vector<cv::KeyPoint> current_keypoints;
    cv::Mat current_descriptors;
    
    try {
        obj_state.orb_detector->detectAndCompute(
            obj_state.gray_frame,
            cv::noArray(),
            current_keypoints,
            current_descriptors
        );
        
        std::printf("[C++] Feature detection successful: %zu keypoints\n", current_keypoints.size());
        
    } catch (const cv::Exception& e) {
        std::printf("[C++] ERROR in detectAndCompute: %s\n", e.what());
        
        // 대안: 검출과 계산을 분리해서 시도
        try {
            std::printf("[C++] Trying separate detect and compute...\n");
            obj_state.orb_detector->detect(obj_state.gray_frame, current_keypoints);
            obj_state.orb_detector->compute(obj_state.gray_frame, current_keypoints, current_descriptors);
            std::printf("[C++] Separate detect/compute successful: %zu keypoints\n", current_keypoints.size());
        } catch (const cv::Exception& e2) {
            std::printf("[C++] ERROR in separate detect/compute: %s\n", e2.what());
            send_calculate_coordinate_to_swift(0,0,0);
            return;
        }
    }
    
    if (current_descriptors.empty() || obj_state.reference_descriptors.empty()) {
        std::printf("[C++] No descriptors found (current: %s, reference: %s)\n",
                   current_descriptors.empty() ? "empty" : "ok",
                   obj_state.reference_descriptors.empty() ? "empty" : "ok");
        send_calculate_coordinate_to_swift(0, 0, 0);
        return;
    }
    
    // 특징점 매칭
    std::vector<cv::DMatch> matches;
    try {
        obj_state.matcher->match(current_descriptors, obj_state.reference_descriptors, matches);
    } catch (const cv::Exception& e) {
        std::printf("[C++] ERROR in matching: %s\n", e.what());
        send_calculate_coordinate_to_swift(0, 0, 0);
        return;
    }
    
    if (matches.size() < obj_state.MIN_MATCH_COUNT) {
        std::printf("[C++] Insufficient matches: %zu < %d\n", matches.size(), obj_state.MIN_MATCH_COUNT);
        send_calculate_coordinate_to_swift(0, 0, 0);
        return;
    }
    
    // 나머지 코드는 동일...
    // 좋은 매칭만 필터링 (거리 기반)
    std::sort(matches.begin(), matches.end());
    std::vector<cv::DMatch> good_matches;
    
    const float ratio_threshold = 0.7f;
    const int max_matches = static_cast<int>(matches.size() * ratio_threshold);
    
    for (int i = 0; i < std::min(max_matches, static_cast<int>(matches.size())); ++i) {
        good_matches.push_back(matches[i]);
    }
    
    // 매칭된 키포인트 시각화
    drawMatchedKeypoints(obj_state.current_frame, current_keypoints, good_matches);
    
    std::printf("[C++] Good matches: %zu/%zu\n", good_matches.size(), matches.size());
    
    // 대응점 찾기
    std::vector<cv::Point3f> matched_3d_points;
    std::vector<cv::Point2f> matched_2d_points = findCorrespondingPoints(
        good_matches, current_keypoints, matched_3d_points);
    
    if (matched_2d_points.size() < 4) {
        std::printf("[C++] Insufficient corresponding points: %zu\n", matched_2d_points.size());
        send_calculate_coordinate_to_swift(0, 0, 0);
        return;
    }
    
    // PnP 실행
    cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) <<
        obj_state.intrinsics.fx, 0.0, obj_state.intrinsics.cx,
        0.0, obj_state.intrinsics.fy, obj_state.intrinsics.cy,
        0.0, 0.0, 1.0);
    cv::Mat distCoeffs = cv::Mat::zeros(1, 5, CV_64F);
    
    cv::Mat rvec, tvec;
    bool success = false;
    
    try {
        if (matched_2d_points.size() >= 6) {
            // RANSAC 사용
            success = cv::solvePnPRansac(
                matched_3d_points, matched_2d_points,
                cameraMatrix, distCoeffs,
                rvec, tvec, false, 100, 8.0f, 0.99
            );
        } else {
            // 일반 PnP
            success = cv::solvePnP(
                matched_3d_points, matched_2d_points,
                cameraMatrix, distCoeffs,
                rvec, tvec, false, cv::SOLVEPNP_ITERATIVE
            );
        }
    } catch (const cv::Exception& e) {
        std::printf("[C++] ERROR in PnP: %s\n", e.what());
        send_calculate_coordinate_to_swift(0, 0, 0);
        return;
    }
    
    if (!success) {
        std::printf("[C++] PnP failed\n");
        send_calculate_coordinate_to_swift(0, 0, 0);
        return;
    }
    
    // 재투영 에러 계산
    std::vector<cv::Point2f> projected_points;
    cv::projectPoints(matched_3d_points, rvec, tvec, cameraMatrix, distCoeffs, projected_points);
    
    float total_error = 0.0f;
    for (size_t i = 0; i < matched_2d_points.size(); ++i) {
        float error = cv::norm(projected_points[i] - matched_2d_points[i]);
        total_error += error;
    }
    float avg_error = total_error / matched_2d_points.size();
    
    if (avg_error > obj_state.MAX_REPROJECTION_ERROR) {
        std::printf("[C++] High reprojection error: %.2f\n", avg_error);
        send_calculate_coordinate_to_swift(0, 0, 0);
        return;
    }
    
    // 결과 저장 및 전송
    obj_state.last_rvec = rvec.clone();
    obj_state.last_tvec = tvec.clone();
    obj_state.has_valid_pose = true;
    obj_state.last_reprojection_error = avg_error;
    
    float x_cm = static_cast<float>(tvec.at<double>(0));
    float y_cm = static_cast<float>(tvec.at<double>(1));
    float z_cm = static_cast<float>(tvec.at<double>(2));
    
    std::printf("[C++] PnP SUCCESS: x=%.1fcm y=%.1fcm z=%.1fcm (error=%.2fpx, matches=%zu)\n",
               x_cm, y_cm, z_cm, avg_error, matched_2d_points.size());
    
    send_calculate_coordinate_to_swift(x_cm, y_cm, z_cm);
}

void receive_camera_frame(void *baseAddress, int width, int height, int bytesPerRow,
                          const KeyPoints_C *points, int pointCount) {
    // 프레임 처리
    cv::Mat frameBGRA(height, width, CV_8UC4, baseAddress, bytesPerRow);
    obj_state.current_frame = frameBGRA.clone();
    cv::cvtColor(obj_state.current_frame, obj_state.gray_frame, cv::COLOR_BGRA2GRAY);
    
    // 특징점 매칭 및 PnP 수행
    performFeatureMatching();
    
    // 처리된 프레임 전송
    send_processed_frame_to_swift(
        obj_state.current_frame.data,
        obj_state.current_frame.cols,
        obj_state.current_frame.rows,
        static_cast<int>(obj_state.current_frame.step)
    );
}

void receive_object_detection_info(struct DetectionObject_C obj) {
    std::cout << "[C++] 객체 정보: " << obj.label << std::endl;
    if (!obj_state.has_intrinsics) return;

    // 내부 파라미터가 설정되어 있어야 진행 가능
    if (!obj_state.has_intrinsics) {
        std::cerr << "[C++] Intrinsics not received yet.\n";
        return;
    }
    
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

// 참조 이미지 로드 함수 (Swift에서 호출)
void load_reference_image(const char* imagePath) {
    std::string path(imagePath);
    if (loadReferenceImage(path)) {
        std::printf("[C++] Reference image loaded successfully\n");
    } else {
        std::printf("[C++] Failed to load reference image\n");
    }
}

void reset_pose_tracking() {
    obj_state.has_valid_pose = false;
    obj_state.last_reprojection_error = 0.0f;
    std::printf("[C++] Pose tracking reset\n");
}

bool is_pose_valid() {
    return obj_state.has_valid_pose &&
           obj_state.last_reprojection_error < obj_state.MAX_REPROJECTION_ERROR;
}

