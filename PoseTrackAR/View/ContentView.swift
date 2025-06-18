//
//  ContentView.swift
//  PoseTrackAR
//
//  Created by 전상민 on 5/31/25.
//

import SwiftUI


let screenWidth = UIScreen.main.bounds.width
let screenHeight = UIScreen.main.bounds.height

enum ActiveView: Equatable {
    case none
    case pnpView
    case lidarView
}

struct ContentView: View {

    @State private var activeView: ActiveView = .none

    var body: some View {
        ZStack {
            // --- AR 뷰 배경 ---
            switch activeView {
            case .pnpView:
                PoseEstimateView()
                    .transition(.opacity)
            case .lidarView:
                ARObjectDistanceView()
                    .transition(.opacity)
            case .none:
                // LiDAR 뷰를 배경으로 사용하여 기술적인 느낌을 줌
                ARObjectDistanceView()
                    .blur(radius: 10)
                    .saturation(0.5)
                    .overlay(Color.black.opacity(0.6))
            }

            // --- UI 오버레이 ---
            if activeView == .none {
                WelcomeView(activeView: $activeView)
                    .transition(.opacity)
            } else {
                InSessionView(activeView: $activeView)
                    .transition(.opacity)
            }
        }
        .edgesIgnoringSafeArea(.all)
        .animation(.easeInOut(duration: 0.4), value: activeView)
    }
}


// MARK: - 시작 화면 컴포넌트
private struct WelcomeView: View {
    @Binding var activeView: ActiveView
    
    var body: some View {
        VStack(spacing: 20) {
            Spacer()
            VStack {
                Image(systemName: "cube.transparent.fill")
                    .font(.system(size: 60)).foregroundColor(.white.opacity(0.8))
                Text("AR Lab")
                    .font(.largeTitle).fontWeight(.bold).foregroundColor(.white)
            }
            .padding(.bottom, 60)
            
            Text("실행할 모드를 선택하세요")
                .font(.headline).foregroundColor(.white.opacity(0.7))

            HStack(spacing: 20) {
                ModeButton(title: "PnP 자세 추정", iconName: "move.3d", action: { activeView = .pnpView })
                ModeButton(title: "LiDAR 거리 측정", iconName: "ruler.fill", action: { activeView = .lidarView })
            }
            Spacer()
            Spacer()
        }
        .padding()
    }
}

// MARK: - AR 세션 중 화면 컴포넌트
private struct InSessionView: View {
    @Binding var activeView: ActiveView
    
    // (이 뷰의 나머지 부분은 예시 UI이므로 그대로 유지)
    // 실제 데이터는 각 AR 뷰 내부에서 처리됩니다.
    var body: some View {
        ZStack {
            // 상단 거리 표시 영역 (LiDAR 모드일 때만)
            VStack {
                if activeView == .lidarView {
                    // 이 부분은 ARObjectDistanceView가 자체적으로 표시해야 합니다.
                    // 여기서는 UI 레이아웃 예시로 남겨둡니다.
                }
                Spacer()
            }
            
            // 하단 컨트롤 버튼 영역
            VStack {
                Spacer()
                HStack {
                    ControlButton(iconName: "xmark", action: { activeView = .none })
                    Spacer()
                    ControlButton(iconName: "arrow.left.arrow.right.circle.fill", action: {
                        activeView = (activeView == .pnpView) ? .lidarView : .pnpView
                    })
                }
                .padding(.horizontal, 30)
                .padding(.bottom, 50)
            }
        }
    }
}


// MARK: - 재사용 가능한 UI 컴포넌트들 (오류 수정됨)

private struct ModeButton: View {
    let title: String
    let iconName: String
    let action: () -> Void
    
    // ★★★ Equatable 오류 수정 1: 햅틱 피드백을 위한 트리거 변수 추가 ★★★
    @State private var triggerHaptic: Int = 0

    var body: some View {
        Button(action: {
            action()
            triggerHaptic += 1 // 버튼 누를 때마다 값 변경
        }) {
            HStack {
                Image(systemName: iconName)
                Text(title).fontWeight(.semibold)
            }
            .padding().frame(maxWidth: .infinity)
            .background(.ultraThinMaterial, in: RoundedRectangle(cornerRadius: 16))
            .foregroundColor(.white)
        }
        // ★★★ Equatable 오류 수정 2: trigger 값으로 action 대신 숫자 변수 사용 ★★★
        .sensoryFeedback(.impact(weight: .medium), trigger: triggerHaptic)
    }
}

private struct ControlButton: View {
    let iconName: String
    let action: () -> Void
    
    // ★★★ Equatable 오류 수정 1: 햅틱 피드백을 위한 트리거 변수 추가 ★★★
    @State private var triggerHaptic: Int = 0
    
    var body: some View {
        Button(action: {
            action()
            triggerHaptic += 1 // 버튼 누를 때마다 값 변경
        }) {
            Image(systemName: iconName)
                .font(.title).padding()
                .background(.black.opacity(0.5), in: Circle())
                .foregroundColor(.white)
        }
        // ★★★ Equatable 오류 수정 2: trigger 값으로 action 대신 숫자 변수 사용 ★★★
        .sensoryFeedback(.impact(weight: .light), trigger: triggerHaptic)
    }
}
