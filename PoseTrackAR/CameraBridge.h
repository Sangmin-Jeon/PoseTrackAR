//
//  Untitled.swift
//  PoseTrackAR
//
//  Created by 전상민 on 6/3/25.
//


// CameraBridge.h

#ifndef CameraBridge_h
#define CameraBridge_h

struct Intrinsics {
    float fx;
    float fy;
    float cx;
    float cy;
    
    int32 width;
    int32 height;
};

#ifdef __cplusplus
extern "C" {
#endif

void receiveIntrinsics(struct Intrinsics intr);

#ifdef __cplusplus
}
#endif

#endif /* CameraBridge_h */
