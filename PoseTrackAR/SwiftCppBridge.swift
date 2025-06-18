//
//  SwiftCppBridge.swift
//  PoseTrackAR
//
//  Created by 전상민 on 6/11/25.
//

import Combine
import CoreImage
import UIKit


@_cdecl("send_processed_frame_to_swift")
public func send_processed_frame_to_swift(
    _ baseAddress: UnsafeRawPointer?,
    _ width: Int32,
    _ height: Int32,
    _ bytesPerRow: Int32
) {
    guard let baseAddress = baseAddress else { return }

    let bufferSize = Int(bytesPerRow) * Int(height)
    let data = Data(bytes: baseAddress, count: bufferSize)

    guard let provider = CGDataProvider(data: data as CFData),
          let cgImage = CGImage(
            width: Int(width),
            height: Int(height),
            bitsPerComponent: 8,
            bitsPerPixel: 32,
            bytesPerRow: Int(bytesPerRow),
            space: CGColorSpaceCreateDeviceRGB(),
            bitmapInfo: CGBitmapInfo(
                rawValue: CGImageAlphaInfo.premultipliedFirst.rawValue | CGBitmapInfo.byteOrder32Little.rawValue
            ), // BGRA 포맷
            provider: provider,
            decode: nil,
            shouldInterpolate: false,
            intent: .defaultIntent
          )
    else {
        print("CGImage 생성 실패")
        return
    }
    
    let uiImage = UIImage(
        cgImage: cgImage,
        scale: UIScreen.main.scale,
        orientation: .right
    )
    processedImageSubject.send(uiImage)
}


public struct Pose {
    public let x: Float
    public let y: Float
    public let z: Float
    public let distance: Float
}

// --- 이동 평균 필터를 위한 전역 변수 ---
private var poseSamples: [(x: Float, y: Float, z: Float)] = []
private let sampleCount = 10 // 평균을 계산할 샘플 개수

// C++에서 좌표를 받을 때마다 호출되는 함수
@_cdecl("send_calculate_coordinate_to_swift")
public func send_calculate_coordinate_to_swift(
    _ x: Float,
    _ y: Float,
    _ z: Float
) {
    processAndAveragePose(x: x, y: y, z: z)
}

// 받은 좌표를 처리하고, 평균을 계산하여 최종 Pose를 전송하는 함수
private func processAndAveragePose(x: Float, y: Float, z: Float) {
    let distance = sqrt(x * x + y * y + z * z)
    
    // 유효성 검사: 비정상적인 값은 샘플에 추가하지 않고 무시합니다.
    // 0, nan, inf 또는 2m(200cm)를 초과하는 값은 버립니다.
    if distance.isZero || distance.isNaN || distance.isInfinite || distance > 200 {
        // C++에서 넘어온 원본 값을 그대로 출력하여 디버깅에 활용할 수 있습니다.
        print(String(format: "[swift] Invalid value received and rejected: x=%.1f, y=%.1f, z=%.1f", x, y, z))
        return
    }

    // 유효한 샘플을 배열에 추가
    poseSamples.append((x: x, y: y, z: z))

    // 배열 크기를 일정하게 유지 (오래된 데이터는 삭제)
    if poseSamples.count > sampleCount {
        poseSamples.removeFirst()
    }
    
    // 충분한 샘플이 모였을 때만 평균 계산을 진행 (최소 10개 이상)
    // 이 조건을 만족하지 않으면 UI는 이전 값을 그대로 유지하게 됩니다.
    guard poseSamples.count >= 5 else {
        return
    }

    // 모든 샘플의 평균 좌표 계산
    var avgX: Float = 0
    var avgY: Float = 0
    var avgZ: Float = 0
    
    for sample in poseSamples {
        avgX += sample.x
        avgY += sample.y
        avgZ += sample.z
    }
    
    let count = Float(poseSamples.count)
    avgX /= count
    avgY /= count
    avgZ /= count
    
    let avgDistance = sqrt(avgX * avgX + avgY * avgY + avgZ * avgZ)

    // 최종적으로 계산된 '안정적인' 평균값을 Pose 객체로 만들어 UI에 전달
    let averagedPose = Pose(x: avgX, y: avgY, z: avgZ, distance: avgDistance)
    poseSubject.send(averagedPose)
    
    // 디버깅을 위한 최종 결과 출력
    print(String(format: "[swift-smooth] Smoothed Pose (cm): x=%.1f, y=%.1f, z=%.1f → distance=%.1f cm | Samples: %d",
                 avgX, avgY, avgZ, avgDistance, poseSamples.count))
}
