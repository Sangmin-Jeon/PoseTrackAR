//
//  PoseEstimateView.swift
//  PoseTrackAR
//
//  Created by 전상민 on 6/16/25.
//

import SwiftUI

struct PoseEstimateView: View {
    @StateObject private var sessionManager = ARSessionManager()
    
    // UI 상태를 관리하는 변수들
    @State private var isPlacingPoints = false // 포인트 찍기 모드 활성화 여부
    
    var body: some View {
        ZStack {
            // MARK: - AR 카메라 뷰
            // ARSessionManager가 C++로부터 받은 최종 이미지를 표시
            if let image = sessionManager.processedImage {
                CameraFrameView(
                    image: image,
                    isPlacingPoints: $isPlacingPoints,
                    keyPoints: $sessionManager.keyPoints
                )
            } else {
                // 로딩 중 표시
                ProgressView("카메라 준비 중...")
                    .progressViewStyle(CircularProgressViewStyle(tint: .white))
                    .foregroundColor(.white)
                    .frame(maxWidth: .infinity, maxHeight: .infinity)
                    .background(Color.black)
            }
            
            // MARK: - UI 오버레이
            VStack {
                // 상단 정보창 (Pose 결과)
                HStack {
                    if let pose = sessionManager.pose {
                        PoseInfoView(pose: pose)
                    }
                    Spacer()
                }
                .padding(.leading)
                .padding(.top, 60)
                
                Spacer()
                
                // 하단 컨트롤 버튼
                HStack(spacing: 20) {
                    // 포인트 찍기 모드 토글 버튼
                    ControlButton(
                        iconName: isPlacingPoints ? "hand.point.up.left.fill" : "hand.point.up.left",
                        color: isPlacingPoints ? .blue : .white,
                        action: { isPlacingPoints.toggle() }
                    )
                    
                    // 포인트 초기화 버튼
                    ControlButton(iconName: "trash", action: {
                        sessionManager.keyPoints.removeAll()
                    })
                }
                .padding(.bottom, 50)
            }
        }
        .edgesIgnoringSafeArea(.all)
        .animation(.easeInOut, value: isPlacingPoints)
        .animation(.easeInOut, value: sessionManager.keyPoints.count)
    }
}

// MARK: - 카메라 프레임과 터치 로직을 담당하는 뷰
private struct CameraFrameView: View {
    let image: UIImage
    @Binding var isPlacingPoints: Bool
    @Binding var keyPoints: [CGPoint] // ARSessionManager의 keyPoints와 바인딩
    
    var body: some View {
        GeometryReader { geo in
            Image(uiImage: image)
                .resizable()
                .aspectRatio(contentMode: .fit)
                .frame(width: geo.size.width, height: geo.size.height)
                .contentShape(Rectangle())
                .onTapGesture { location in
                    guard isPlacingPoints else { return } // 포인트 찍기 모드일 때만 동작
                    
                    // 뷰 터치 좌표 -> 원본 이미지 픽셀 좌표 변환 로직
                    let viewSize = geo.size
                    let imageSize = image.size
                    
                    let scaleX = imageSize.width / viewSize.width
                    let scaleY = imageSize.height / viewSize.height
                    
                    let imageX = location.x * scaleX
                    let imageY = location.y * scaleY
                    
                    // 변환된 픽셀 좌표를 keyPoints 배열에 추가
                    keyPoints.append(CGPoint(x: imageX, y: imageY))
                }
                .overlay(
                    // 현재 찍힌 포인트들을 화면에 표시
                    ForEach(Array(keyPoints.enumerated()), id: \.offset) { index, point in
                        // 이미지 픽셀 좌표 -> 뷰 좌표로 다시 변환하여 표시
                        let viewX = (point.x / image.size.width) * geo.size.width
                        let viewY = (point.y / image.size.height) * geo.size.height
                        
                        KeypointMarkerView(number: index + 1)
                            .position(x: viewX, y: viewY)
                    }
                )
        }
    }
}


// MARK: - 재사용 가능한 UI 컴포넌트들

private struct PoseInfoView: View {
    let pose: Pose
    
    var body: some View {
        VStack(alignment: .leading, spacing: 5) {
            HStack {
                Image(systemName: "move.3d")
                Text("Pose Estimation")
                    .fontWeight(.bold)
            }
            .font(.headline)
            .padding(.bottom, 5)

            Text(String(format: "x: %8.1f cm", pose.x))
            Text(String(format: "y: %8.1f cm", pose.y))
            Text(String(format: "z: %8.1f cm", pose.z))
            Divider().background(Color.white)
            Text(String(format: "dist: %6.1f cm", pose.distance))
                .fontWeight(.semibold)
        }
        .font(.system(.body, design: .monospaced))
        .padding()
        .background(.ultraThinMaterial, in: RoundedRectangle(cornerRadius: 16))
        .foregroundColor(.white)
    }
}

private struct KeypointMarkerView: View {
    let number: Int
    
    var body: some View {
        ZStack {
            Circle()
                .fill(Color.red)
                .frame(width: 24, height: 24)
            Circle()
                .stroke(Color.white, lineWidth: 2)
                .frame(width: 24, height: 24)
            Text("\(number)")
                .font(.system(size: 14, weight: .bold))
                .foregroundColor(.white)
        }
    }
}

private struct ControlButton: View {
    let iconName: String
    var color: Color = .white
    let action: () -> Void
    
    @State private var triggerHaptic: Int = 0

    var body: some View {
        Button(action: {
            action()
            triggerHaptic += 1
        }) {
            Image(systemName: iconName)
                .font(.title)
                .padding()
                .background(.black.opacity(0.5), in: Circle())
                .foregroundColor(color)
        }
        .sensoryFeedback(.impact(weight: .light), trigger: triggerHaptic)
    }
}


// MARK:  터치 기능 추가시 사용
//    .onTapGesture { location in
//                        // 뷰 상의 터치 좌표
//                        let x_view = location.x
//                        let y_view = location.y
//
//                        // 뷰 전체 크기
//                        let viewW = geo.size.width
//                        let viewH = geo.size.height
//
//                        // AR 카메라 원본 해상도
//                        let camW = CGFloat(ins.intrinsics?.width ?? 1)
//                        let camH = CGFloat(ins.intrinsics?.height ?? 1)
//
//                        // 이미지와 뷰의 종횡비
//                        let imgAspect  = camW / camH
//                        let viewAspect = viewW / viewH
//
//                        // 화면에 실제 그려진 이미지 크기 계산
//                        let displayedWidth:  CGFloat
//                        let displayedHeight: CGFloat
//                        if imgAspect > viewAspect {
//                            displayedWidth  = viewW
//                            displayedHeight = viewW / imgAspect
//                        }
//                        else {
//                            displayedHeight = viewH
//                            displayedWidth  = viewH * imgAspect
//                        }
//
//                        // 이미지가 가운데 정렬되면서 생긴 여백(offset)
//                        let xOffset = (viewW  - displayedWidth ) * 0.5
//                        let yOffset = (viewH  - displayedHeight) * 0.5
//
//                        // 뷰 좌표 → 이미지 영역 내부 좌표로 변환
//                        let x_inImage = (x_view - xOffset) * (camW / displayedWidth)
//                        let y_inImage = (y_view - yOffset) * (camH / displayedHeight)
//
//                        // 화면 표시용
//                        show_keypoints.append(CGPoint(x: x_view, y: y_view))
//                        // PnP에 넘길 실제 카메라 픽셀 좌표
//                        ins.keyPoints.append(CGPoint(x: x_inImage, y: y_inImage))
//                    }
//                    .overlay(
//                        ForEach(Array(show_keypoints.enumerated()), id: \.offset) { _, point in
//                            Circle()
//                                .fill(Color.red)
//                                .frame(width: 12, height: 12)
//                                .position(point)
//                        }
//                    )
