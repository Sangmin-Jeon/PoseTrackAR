//
//  CameraBridge.cpp
//  PoseTrackAR
//
//  Created by 전상민 on 6/3/25.
//

#include <stdio.h>
#include "CameraBridge.h"

#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>


void receive_intrinsics(struct Intrinsics_C intr) {
    printf("[C++] fx: %.2f, fy: %.2f, cx: %.2f, cy: %.2f, width: %d, height: %d\n",
           intr.fx, intr.fy, intr.cx, intr.cy, intr.width, intr.height);
    
}

void receive_camera_frame(void* baseAddress, int width, int height, int bytesPerRow) {
    /// 입력: BGRA 포맷
    cv::Mat matBGRA(height, width, CV_8UC4, baseAddress, bytesPerRow);

    // BGRA → GRAY
    cv::Mat matGray;
    cv::cvtColor(matBGRA, matGray, cv::COLOR_BGRA2GRAY);

    // GRAY → BGRA (R=G=B=gray, A=255)
    cv::Mat matGrayBGRA;
    cv::cvtColor(matGray, matGrayBGRA, cv::COLOR_GRAY2BGRA);

    // 회전
    cv::Mat matRotated;
    cv::rotate(matGrayBGRA, matRotated, cv::ROTATE_90_CLOCKWISE);

    // Swift로 전달
    send_processed_frame_to_swift(
        matRotated.data,
        matRotated.cols,
        matRotated.rows,
        static_cast<int>(matRotated.step)
    );
    
}
