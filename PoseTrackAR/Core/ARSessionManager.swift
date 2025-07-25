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

struct KeyPoints {
    let x: Float
    let y: Float
}


let processedImageSubject = PassthroughSubject<UIImage, Never>()
let poseSubject = PassthroughSubject<Pose, Never>()

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
    @Published var keyPoints: [CGPoint] = []
    @Published var pose: Pose?

    private var cancellable = Set<AnyCancellable>()

    override init() {
        super.init()
        session.delegate = self
        
        if let imagePath = Bundle.main.path(forResource: "ref_img", ofType: "jpeg") {
            load_reference_image(imagePath)
        }

        let cfg = ARWorldTrackingConfiguration()
        cfg.frameSemantics = []          // 필요 옵션만
        cfg.videoFormat = ARWorldTrackingConfiguration.supportedVideoFormats
                .filter { $0.framesPerSecond == 30 }  // 30fps로 제한
                .first ?? ARWorldTrackingConfiguration.supportedVideoFormats[0]
        
        // 자동 포커스 끄기
        // cfg.isAutoFocusEnabled = false

        session.run(cfg)
        
        // C++에서 보낸 이미지를 받아서 @Published 프로퍼티 업데이트
        processedImageSubject
            .receive(on: DispatchQueue.main)
            .sink { [weak self] image in
                self?.processedImage = image
            }
            .store(in: &cancellable)
        
        // C++에서 보낸 Pose 정보를 받아서 @Published 프로퍼티 업데이트
        poseSubject
            .receive(on: DispatchQueue.main)
            .sink { [weak self] _pose in
                self?.pose = _pose
            }
            .store(in: &cancellable)
    }
    
}

extension ARSessionManager: ARSessionDelegate {
    func session(_ session: ARSession, didUpdate frame: ARFrame) {
        Task {
            guard let uiImage = self.convertPixelBufferToUIImage(frame.capturedImage),
                  let _ = await self.detector.detect(image: uiImage) else { return }

            // detection_obj로 last_bbox, has_detection 셋업 끝난 뒤 C++에 프레임 넘기기
            self.receiveIntrinsicsToCpp(ins: frame.camera.intrinsics,
                                        res: frame.camera.imageResolution)
            self.receivePixelBufToCpp(frame.capturedImage)
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
        var ciImage = CIImage(cvPixelBuffer: pixelBuffer)
        ciImage = ciImage.oriented(.right)
        let context = CIContext()
        
        guard let cgImage = context.createCGImage(ciImage, from: ciImage.extent) else {
            print("CGImage 변환 실패")
            return nil
        }

        return UIImage(cgImage: cgImage)
    }
    
    
    private func receivePixelBufToCpp(_ pixelBuffer: CVPixelBuffer) {
        CVPixelBufferLockBaseAddress(pixelBuffer, .readOnly)
        defer { CVPixelBufferUnlockBaseAddress(pixelBuffer, .readOnly) }

        // Swift의 [CGPoint]를 C 호환 KeyPoints 배열로 변환
        let kpArr = keyPoints.map { KeyPoints(x: Float($0.x), y: Float($0.y)) }
        
        DispatchQueue.global(qos: .userInitiated).async {
            guard let (dataPtr, width, height) = self.pixelBufferToBGRAData(pixelBuffer),
                  let dataPtr = dataPtr else { return }

            // 항상 호출하되, 터치가 없으면 buf.count == 0, buf.baseAddress == nil
            kpArr.withUnsafeBufferPointer { buf in
                buf.baseAddress?.withMemoryRebound(
                    to: KeyPoints_C.self,
                    capacity: buf.count
                ) { cPtr in
                    receive_camera_frame(
                        UnsafeMutableRawPointer(mutating: dataPtr),
                        Int32(width),
                        Int32(height),
                        Int32(width * 4),
                        cPtr,
                        Int32(buf.count)
                    )
                }
                // 터치 포인트가 없으면 buf.baseAddress는 nil → C엔 NULL, pointCount=0 으로 넘어감
            }

            dataPtr.deallocate()
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
            bitmapInfo: CGImageAlphaInfo.premultipliedFirst.rawValue | CGBitmapInfo.byteOrder32Little.rawValue
            // 핵심 변경: premultipliedLast → premultipliedFirst (RGBA → BGRA)
        )
        else {
            rawData.deallocate()
            return nil
        }
        
        contextRef.draw(cgImage, in: rect)
        return (UnsafeRawPointer(rawData), width, height)
    }
    
}
