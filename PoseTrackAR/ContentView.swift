//
//  ContentView.swift
//  PoseTrackAR
//
//  Created by 전상민 on 5/31/25.
//

import SwiftUI

struct ContentView: View {
    @StateObject private var ins = ARSessionManager()
    
    var body: some View {
        ZStack {
            if let image = ins.processedImage {
                Image(uiImage: image)
                    .resizable()
                    .aspectRatio(contentMode: .fit)
                    .frame(maxWidth: .infinity, maxHeight: .infinity)
            } else {
                Text("처리된 이미지 없음")
                    .foregroundColor(.gray)
            }
        }
    }
}
