//
//  ContentView.swift
//  PoseTrackAR
//
//  Created by 전상민 on 5/31/25.
//

import SwiftUI

let screenWidth = UIScreen.main.bounds.width
let screenHeight = UIScreen.main.bounds.height

struct ContentView: View {
    @StateObject private var ins = ARSessionManager()
    @State private var show_keypoints: [CGPoint] = []

    var body: some View {
        ZStack {
            // 1) 카메라 + 키포인트
            if let image = ins.processedImage {
                GeometryReader { geo in
                    Image(uiImage: image)
                        .resizable()
                        //.rotationEffect(.degrees(90))
                        .aspectRatio(contentMode: .fit)
                        .frame(maxWidth: screenWidth, maxHeight: screenHeight)
                        .contentShape(Rectangle())
                        .onTapGesture { location in
                            // 뷰 상의 터치 좌표
                            let x_view = location.x
                            let y_view = location.y

                            // 뷰 전체 크기
                            let viewW = geo.size.width
                            let viewH = geo.size.height

                            // AR 카메라 원본 해상도
                            let camW = CGFloat(ins.intrinsics?.width ?? 1)
                            let camH = CGFloat(ins.intrinsics?.height ?? 1)

                            // 이미지와 뷰의 종횡비
                            let imgAspect  = camW / camH
                            let viewAspect = viewW / viewH

                            // 화면에 실제 그려진 이미지 크기 계산
                            let displayedWidth:  CGFloat
                            let displayedHeight: CGFloat
                            if imgAspect > viewAspect {
                                displayedWidth  = viewW
                                displayedHeight = viewW / imgAspect
                            }
                            else {
                                displayedHeight = viewH
                                displayedWidth  = viewH * imgAspect
                            }

                            // 이미지가 가운데 정렬되면서 생긴 여백(offset)
                            let xOffset = (viewW  - displayedWidth ) * 0.5
                            let yOffset = (viewH  - displayedHeight) * 0.5

                            // 뷰 좌표 → 이미지 영역 내부 좌표로 변환
                            let x_inImage = (x_view - xOffset) * (camW / displayedWidth)
                            let y_inImage = (y_view - yOffset) * (camH / displayedHeight)

                            // 화면 표시용
                            show_keypoints.append(CGPoint(x: x_view, y: y_view))
                            // PnP에 넘길 실제 카메라 픽셀 좌표
                            ins.keyPoints.append(CGPoint(x: x_inImage, y: y_inImage))
                        }
                        .overlay(
                            ForEach(Array(show_keypoints.enumerated()), id: \.offset) { _, point in
                                Circle()
                                    .fill(Color.red)
                                    .frame(width: 12, height: 12)
                                    .position(point)
                            }
                        )
                }
                .frame(maxWidth: screenWidth, maxHeight: screenHeight)
            } else {
                // 로딩 뷰
                VStack(spacing: 16) {
                    ProgressView()
                        .progressViewStyle(CircularProgressViewStyle(tint: .blue))
                        .scaleEffect(2.0)
                    Text("Processing camera feed...")
                        .font(.headline)
                        .foregroundColor(.gray)
                    Text("Please wait while we prepare the frame.")
                        .font(.subheadline)
                        .foregroundColor(.secondary)
                }
                .frame(maxWidth: .infinity, maxHeight: .infinity)
                .background(Color.black.opacity(0.05))
            }

            // pose 텍스트: 전체 ZStack 위에, 독립적으로 좌상단 배치
            if let pose = ins.pose {
                VStack(alignment: .leading, spacing: 4) {
                    Text(String(format: "x: %.1f cm", pose.x))
                    Text(String(format: "y: %.1f cm", pose.y))
                    Text(String(format: "z: %.1f cm", pose.z))
                    Text(String(format: "dist: %.1f cm", pose.distance))
                }
                .font(.system(size: 16, weight: .semibold, design: .monospaced))
                .padding(8)
                .background(Color.black.opacity(0.5))
                .foregroundColor(.white)
                .cornerRadius(8)
                .frame(maxWidth: .infinity, maxHeight: .infinity, alignment: .topLeading)
                .padding(.top, 20)
                .padding(.leading, 20)
            }
        }
        .edgesIgnoringSafeArea(.all)
    }
}
