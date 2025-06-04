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
    
    var body: some View {
        ZStack {
            if let image = ins.processedImage {
                Image(uiImage: image)
                    .resizable()
                    .rotationEffect(.degrees(90))
                    .aspectRatio(contentMode: .fill)
                    .frame(maxWidth: screenWidth, maxHeight: screenHeight)
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
        
    }
}
