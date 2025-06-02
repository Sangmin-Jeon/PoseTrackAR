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

// C 함수 선언
void receive_intrinsics(struct Intrinsics_C intr);

#ifdef __cplusplus
}
#endif

#endif /* CameraBridge_h */
