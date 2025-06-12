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


@_cdecl("send_calculate_coordinate_to_swift")
public func send_calculate_coordinate_to_swift(
    _ x: Float,
    _ y: Float,
    _ z: Float
) {
    let distance = sqrt(x * x + y * y + z * z)
    print(String(format: "[swift] obj coordinate (cm): x=%.1f, y=%.1f, z=%.1f → distance=%.1f cm",
                 x, y, z, distance))
     
}
