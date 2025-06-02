//
//  Untitled.swift
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


// 내부 파라미터 전달 함수
void receive_intrinsics(struct Intrinsics_C intr);

// 카메라 이미지 프레임 전달 함수 (iOS BGRA 데이터)
void receive_camera_frame(void* baseAddress, int width, int height, int bytesPerRow);

void send_processed_frame_to_swift(void* baseAddress, int width, int height, int bytesPerRow);


#ifdef __cplusplus
}
#endif

#endif /* CameraBridge_h */
