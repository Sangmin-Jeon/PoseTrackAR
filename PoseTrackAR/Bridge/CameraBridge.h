//
//  CameraBridge.h
//  PoseTrackAR
//
//  Created by 전상민 on 6/3/25.
//

#ifndef CameraBridge_h
#define CameraBridge_h

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// C에서 사용할 구조체
struct Intrinsics_C {
    float fx;
    float fy;
    float cx;
    float cy;
    int width;
    int height;
};

// C++ 호환 구조체
struct DetectionObject_C {
    char label[32];    // 라벨 문자열 (최대 길이 제한)
    float confidence;  // 신뢰도
    float bbox_x;      // 바운딩 박스 좌상단 x (정규화 0-1)
    float bbox_y;      // 바운딩 박스 좌상단 y (정규화 0-1)
    float bbox_width;  // 바운딩 박스 너비 (정규화 0-1)
    float bbox_height; // 바운딩 박스 높이 (정규화 0-1)
};

typedef struct {
    float x; // 정규화된 좌표 (0-1) 또는 픽셀 좌표
    float y; // 정규화된 좌표 (0-1) 또는 픽셀 좌표
} KeyPoints_C;

// 카메라 내부 파라미터 전달
void receive_intrinsics(struct Intrinsics_C intr);

// 카메라 프레임 + 터치 포인트 전달
void receive_camera_frame(
    void *baseAddress,        // BGRA 버퍼 시작 주소
    int width,               // 프레임 너비
    int height,              // 프레임 높이
    int bytesPerRow,         // 행당 바이트 수
    const KeyPoints_C *points, // 터치점 배열 (NULL 가능)
    int pointCount           // 배열 길이 (0 가능)
);

// 객체 탐지 정보 전달 (선택사항)
void receive_object_detection_info(struct DetectionObject_C obj);

// 계산된 객체 좌표를 Swift로 전달
void send_calculate_coordinate_to_swift(float x, float y, float z);

// 처리된 프레임을 Swift로 전달
void send_processed_frame_to_swift(
    void *baseAddress,
    int width,
    int height,
    int bytesPerRow
);


// 참조 이미지 로드 (앱 번들의 이미지 파일 경로)
void load_reference_image(const char* imagePath);

// 포즈 추적 리셋
void reset_pose_tracking(void);

// 포즈 유효성 확인
bool is_pose_valid(void);

#ifdef __cplusplus
}
#endif

#endif /* CameraBridge_h */
