//
//  ContentView.swift
//  PoseTrackAR
//
//  Created by 전상민 on 5/31/25.
//

import SwiftUI
import YOLO

struct ContentView: View {
    @StateObject private var ins = ARSessionManager()
    
    var body: some View {
        ZStack {
            if let image = ins.processedImage {
                Image(uiImage: image)
                    .resizable()
                    .aspectRatio(contentMode: .fit)
                    .frame(maxWidth: .infinity, maxHeight: .infinity)
            }
            else {
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
        .onAppear {
            Task {
                do {
                    let yolo = try YOLO("yolo11n", task: .detect)
                    guard let url = URL(string: "https://ultralytics.com/images/bus.jpg") else { return }
                    
                    let (data, _) = try await URLSession.shared.data(from: url)
                    if let image = UIImage(data: data) {
                        let result = yolo(image)
                        print("Result: \(result)")
                    } else {
                        print("이미지 변환 실패")
                    }
                } catch {
                    print("모델 로딩 또는 추론 실패: \(error)")
                }
            }
        }
        
    }
}
