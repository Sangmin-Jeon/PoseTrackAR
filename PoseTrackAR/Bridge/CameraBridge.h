//
//  CameraBridge.h
//  PoseTrackAR
//
//  Created by 전상민 on 6/3/25.
//

#ifndef CameraBridge_h
#define CameraBridge_h

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
    char label[32];       // 라벨 문자열 (최대 길이 제한)
    float confidence;     // 신뢰도
    float bbox_x;         // 바운딩 박스 좌상단 x (정규화 or 픽셀)
    float bbox_y;         // 바운딩 박스 좌상단 y
    float bbox_width;     // 바운딩 박스 너비
    float bbox_height;    // 바운딩 박스 높이
};

typedef struct {
    float x;
    float y;
} KeyPoints_C;


// 내부 파라미터 전달 함수
void receive_intrinsics(struct Intrinsics_C intr);

// 카메라 이미지 프레임 전달 함수 (iOS BGRA 데이터)
void receive_camera_frame(
    void*               baseAddress,
    int                 width,
    int                 height,
    int                 bytesPerRow,
    const KeyPoints_C*    points,     // 터치점 배열 (nil 가능)
    int                 pointCount  // 배열 길이 (0 가능)
);

// 객체 정보
void receive_object_detection_info(struct DetectionObject_C obj);

// 계산한 객체 좌표 (Swift로 전달)
void send_calculate_coordinate_to_swift(float x, float y, float z);

void send_processed_frame_to_swift(void* baseAddress, int width, int height, int bytesPerRow);

#ifdef __cplusplus
}
#endif

#endif /* CameraBridge_h */
