//
//  ContentView.swift
//  PoseTrackAR
//
//  Created by 전상민 on 5/31/25.
//

import SwiftUI
import ARKit
import ReplayKit

let screenWidth = UIScreen.main.bounds.width
let screenHeight = UIScreen.main.bounds.height

struct ContentView: View {

    var body: some View {
        ZStack {
            PoseEstimateView()
            // ARObjectScanAndDetectView()
        }
        .edgesIgnoringSafeArea(.all)
    }
}
