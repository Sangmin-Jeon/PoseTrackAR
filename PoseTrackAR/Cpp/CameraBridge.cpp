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
    // BGRA 포맷 입력
    cv::Mat matBGRA(height, width, CV_8UC4, baseAddress, bytesPerRow);

    // BGRA → GRAY
    cv::Mat matGray;
    cv::cvtColor(matBGRA, matGray, cv::COLOR_BGRA2GRAY);

    // GRAY → RGB (BGR이 아닌 RGB로 변환)
    cv::Mat matGrayRGB;
    cv::cvtColor(matGray, matGrayRGB, cv::COLOR_GRAY2RGB);
    
    // RGB → RGBA (알파 채널 추가)
    cv::Mat matRGBA;
    cv::cvtColor(matGrayRGB, matRGBA, cv::COLOR_RGB2RGBA);

    // 회전
    cv::Mat matRotated;
    cv::rotate(matRGBA, matRotated, cv::ROTATE_90_CLOCKWISE);

    // Swift로 전달
    send_processed_frame_to_swift(
        matRotated.data,
        matRotated.cols,
        matRotated.rows,
        static_cast<int>(matRotated.step)
    );
}
