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
