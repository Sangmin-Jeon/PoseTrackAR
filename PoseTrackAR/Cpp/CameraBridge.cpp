//
//  CameraBridge.cpp
//  PoseTrackAR
//
//  Created by 전상민 on 6/3/25.
//

#include "CameraBridge.h"
#include <stdio.h>

void receiveIntrinsics(struct IntrinsicsC intr) {
    printf("[C++] fx: %.2f, fy: %.2f, cx: %.2f, cy: %.2f, width: %d, height: %d\n",
           intr.fx, intr.fy, intr.cx, intr.cy, intr.width, intr.height);
}
