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
            if let image = ins.processedImage {
                // Wrap in GeometryReader to convert tap coords
                GeometryReader { geo in
                    Image(uiImage: image)
                        .resizable()
                        //.rotationEffect(.degrees(90))
                        .aspectRatio(contentMode: .fit)
                        .frame(maxWidth: screenWidth, maxHeight: screenHeight)
                        .contentShape(Rectangle()) // make full area tappable
                        .onTapGesture { location in
                            // 뷰 좌표
                            let x_view = location.x
                            let y_view = location.y
                            
                            // 화면에 그려진 이미지 크기
                            let viewW = geo.size.width
                            let viewH = geo.size.height
                            
                            // AR 카메라 원본 해상도
                            let camW = CGFloat(ins.intrinsics?.width ?? 1)
                            let camH = CGFloat(ins.intrinsics?.height ?? 1)
                            
                            // 뷰 → 카메라 해상도로 스케일 변환
                            let scaleX = camW / viewW
                            let scaleY = camH / viewH
                            
                            // 변환된 카메라 픽셀 좌표
                            let x_cam = x_view * scaleX
                            let y_cam = y_view * scaleY
                            
                            // 화면 표시용
                            show_keypoints.append(CGPoint(x: x_view, y: y_view))
                            // C++ 로 넘길 실제 카메라 좌표
                            ins.keyPoints.append(CGPoint(x: x_cam, y: y_cam))
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
        }
    }
}
