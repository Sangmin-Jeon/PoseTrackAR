//
//  Untitled.swift
//  PoseTrackAR
//
//  Created by 전상민 on 6/2/25.
//

import ARKit
import Combine

import CoreImage
import UIKit
import YOLO

struct Intrinsics {
    let fx: Float
    let fy: Float
    let cx: Float
    let cy: Float
    
    let width: Int32
    let height: Int32
}

let processedImageSubject = PassthroughSubject<UIImage, Never>()

@_cdecl("send_calculate_coordinate_to_swift")
public func send_calculate_coordinate_to_swift(
    _ x: Float,
    _ y: Float,
    _ z: Float
) {
    print("[swift] obj coordinate: \(x), \(y), \(z)")
    
    
}

class ARSessionManager: NSObject, ObservableObject {
    private let session = ARSession()
    private let detector = ObjectDetector()

    /*
     intrinsics.columns.0 = [f_x, 0,     0]   // X축 방향 열
     intrinsics.columns.1 = [s,   f_y,   0]   // Y축 방향 열
     intrinsics.columns.2 = [c_x, c_y,   1]   // 주점과 1
     */
    @Published var intrinsics: Intrinsics?
    @Published var res: CGSize = .zero
    
    @Published var processedImage: UIImage?

    private var cancellable: AnyCancellable?

    override init() {
        super.init()
        session.delegate = self

        let cfg = ARWorldTrackingConfiguration()
        cfg.frameSemantics = []          // 필요 옵션만
        cfg.videoFormat = ARWorldTrackingConfiguration.supportedVideoFormats
                .filter { $0.framesPerSecond == 30 }  // 30fps로 제한
                .first ?? ARWorldTrackingConfiguration.supportedVideoFormats[0]
        session.run(cfg)
        
        cancellable = processedImageSubject
            .receive(on: DispatchQueue.main)
            .sink { [weak self] image in
                guard let self = self else { return }
                DispatchQueue.main.async {
                    self.processedImage = image
                }
            }
    }


}

extension ARSessionManager: ARSessionDelegate {
    func session(_ session: ARSession, didUpdate frame: ARFrame) {
        let ins: simd_float3x3 = frame.camera.intrinsics
        let res = frame.camera.imageResolution
        let pixelBuf = frame.capturedImage
        
        self.receiveIntrinsicsToCpp(ins: ins, res: res)
        self.receivePixelBufToCpp(pixelBuf)
        
        DispatchQueue.global(qos: .userInitiated).async {
            // image 변환 등 시간 걸리는 작업은 여기서 처리
            guard let image = self.convertPixelBufferToUIImage(pixelBuf) else { return }
            
            self.detector.detect(image: image)
        }
    }
    
}

extension ARSessionManager {
    private func receiveIntrinsicsToCpp(ins: simd_float3x3, res: CGSize) {
        let _intrinsics = Intrinsics_C(
            fx: ins.columns.0.x, fy: ins.columns.1.y,
            cx: ins.columns.2.x, cy: ins.columns.2.y,
            width: Int32(res.width), height: Int32(res.height)
        )
        receive_intrinsics(_intrinsics)
    }
    
    
    func convertPixelBufferToUIImage(_ pixelBuffer: CVPixelBuffer) -> UIImage? {
        let ciImage = CIImage(cvPixelBuffer: pixelBuffer)
        let context = CIContext()
        
        guard let cgImage = context.createCGImage(ciImage, from: ciImage.extent) else {
            print("CGImage 변환 실패")
            return nil
        }

        return UIImage(cgImage: cgImage)
    }
    
    
    private func receivePixelBufToCpp(_ pixelBuffer: CVPixelBuffer) {
        CVPixelBufferLockBaseAddress(pixelBuffer, .readOnly)
        defer {
            CVPixelBufferUnlockBaseAddress(pixelBuffer, .readOnly)
        }
        
        DispatchQueue.global(qos: .userInitiated).async {
            if let (dataPtr, width, height) = self.pixelBufferToBGRAData(pixelBuffer),
               let dataPtr = dataPtr {
                receive_camera_frame(
                    UnsafeMutableRawPointer(mutating: dataPtr),
                    Int32(width), Int32(height), Int32(width * 4)
                )
                dataPtr.deallocate()
            }
        }
    }
    
    
    private func pixelBufferToBGRAData(_ pixelBuffer: CVPixelBuffer) -> (UnsafeRawPointer?, Int, Int)? {
        let ciImage = CIImage(cvPixelBuffer: pixelBuffer)
        let context = CIContext(options: nil)
        
        let width = CVPixelBufferGetWidth(pixelBuffer)
        let height = CVPixelBufferGetHeight(pixelBuffer)
        let rect = CGRect(x: 0, y: 0, width: width, height: height)
        
        guard let cgImage = context.createCGImage(ciImage, from: rect) else { return nil }
        
        let bytesPerPixel = 4
        let bytesPerRow = width * bytesPerPixel
        let bufferSize = bytesPerRow * height
        
        let rawData = UnsafeMutablePointer<UInt8>.allocate(capacity: bufferSize)
        let colorSpace = CGColorSpaceCreateDeviceRGB()
        
        guard let contextRef = CGContext(
            data: rawData, width: width, height: height,
            bitsPerComponent: 8, bytesPerRow: bytesPerRow, space: colorSpace,
            bitmapInfo: CGImageAlphaInfo.premultipliedLast.rawValue | CGBitmapInfo.byteOrder32Little.rawValue
        )
        else {
            rawData.deallocate()
            return nil
        }
        
        contextRef.draw(cgImage, in: rect)
        return (UnsafeRawPointer(rawData), width, height)
        
    }
    
}
