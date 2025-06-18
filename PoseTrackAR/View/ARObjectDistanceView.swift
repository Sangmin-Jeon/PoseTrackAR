import SwiftUI
import ARKit
import RealityKit
import simd

// 메인 콘텐츠 뷰
struct ARObjectDistanceView: View {
    // 측정된 거리를 저장할 상태 변수 (미터 단위)
    @State private var distance: Float?
    // YOLO가 감지한 바운딩 박스를 저장할 상태 변수
    @State private var detectedBox: CGRect?

    var body: some View {
        ZStack {
            // AR 뷰 및 로직
            LidarDistanceView(distance: $distance, detectedBox: $detectedBox)
                .edgesIgnoringSafeArea(.all)

            // UI 오버레이
            VStack {
                // 측정된 거리 값을 표시
                if let distance = distance {
                    Text(String(format: "%.0f cm", distance * 100))
                            .font(.largeTitle).fontWeight(.bold)
                            .foregroundColor(.white)
                            .padding()
                            .background(Color.black.opacity(0.6))
                            .cornerRadius(10)
                }
                else {
                    Text("객체를 찾는 중...")
                        .font(.headline).foregroundColor(.white)
                        .padding().background(Color.black.opacity(0.6))
                        .cornerRadius(10)
                }
                
                Spacer() // UI를 상단에 고정
            }
            .padding(.top, 50)
            
            // 감지된 바운딩 박스를 화면에 표시
            if let box = detectedBox {
                Rectangle()
                    .stroke(Color.red, lineWidth: 4)
                    .frame(width: box.width, height: box.height)
                    .position(x: box.midX, y: box.midY)
            }
        }
    }
}

// ARKit + RealityKit 뷰
fileprivate struct LidarDistanceView: UIViewRepresentable {
    @Binding var distance: Float?
    @Binding var detectedBox: CGRect?
    
    func makeUIView(context: Context) -> ARView {
        let arView = ARView(frame: .zero)
        arView.session.delegate = context.coordinator
        
        let config = ARWorldTrackingConfiguration()
        // sceneDepth 활성화
        config.frameSemantics.insert(.sceneDepth)
        if ARWorldTrackingConfiguration.supportsSceneReconstruction(.mesh) {
            config.sceneReconstruction = .mesh
        }
        config.planeDetection = [.horizontal, .vertical]
        
        arView.session.run(config, options: [.resetTracking, .removeExistingAnchors])
        context.coordinator.arView = arView
        return arView
    }
    
    func updateUIView(_ uiView: ARView, context: Context) {}
    
    func makeCoordinator() -> Coordinator {
        Coordinator(distance: $distance, detectedBox: $detectedBox)
    }
    
    class Coordinator: NSObject, ARSessionDelegate {
        @Binding var distance: Float?
        @Binding var detectedBox: CGRect?
        weak var arView: ARView?
        private let objectDetectorActor = ObjectDetector()
        private var isProcessingFrame = false
        
        init(distance: Binding<Float?>, detectedBox: Binding<CGRect?>) {
            _distance = distance
            _detectedBox = detectedBox
        }
        
        func session(_ session: ARSession, didUpdate frame: ARFrame) {
            guard !isProcessingFrame else { return }
            self.isProcessingFrame = true
            
            Task {
                var ciImage = CIImage(cvPixelBuffer: frame.capturedImage)
                ciImage = ciImage.oriented(.right)
                
                let context = CIContext()
                guard let cgImage = context.createCGImage(ciImage, from: ciImage.extent) else {
                    await MainActor.run { self.isProcessingFrame = false }
                    return
                }
                let uiImage = UIImage(cgImage: cgImage)
                
                let detection = await objectDetectorActor.detect(image: uiImage)
                
                await MainActor.run {
                    if let detectedObject = detection {
                        self.processDetections(detectedObject)
                    } else {
                        self.distance = nil
                        self.detectedBox = nil
                    }
                    self.isProcessingFrame = false
                }
            }
        }
        
        private func processDetections(_ detections: DetectionObject) {
            guard let arView = self.arView else { return }
            
            let viewRect = arView.bounds
            let pixelBoundingBox = CGRect(
                x: detections.boundingBox.origin.x * viewRect.width,
                y: detections.boundingBox.origin.y * viewRect.height,
                width: detections.boundingBox.size.width * viewRect.width,
                height: detections.boundingBox.size.height * viewRect.height
            )
            
            self.detectedBox = pixelBoundingBox
            self.distance = self.getDistance(to: pixelBoundingBox, in: arView)
        }
        
        /// 바운딩 박스 중심의 Raw Depth 맵을 읽어 거리를 계산하도록 변경
        private func getDistance(to boundingBox: CGRect, in arView: ARView) -> Float? {
            guard
                let frame = arView.session.currentFrame,
                let sceneDepth = frame.sceneDepth
            else {
                return nil
            }
            
            let depthMap = sceneDepth.depthMap
            CVPixelBufferLockBaseAddress(depthMap, .readOnly)
            defer { CVPixelBufferUnlockBaseAddress(depthMap, .readOnly) }
            
            let width = CVPixelBufferGetWidth(depthMap)
            let height = CVPixelBufferGetHeight(depthMap)
            let bytesPerRow = CVPixelBufferGetBytesPerRow(depthMap)
            
            // 뎁스 맵 좌표로 변환
            let cx = Int((boundingBox.midX / arView.bounds.width) * CGFloat(width))
            let cy = Int((boundingBox.midY / arView.bounds.height) * CGFloat(height))
            
            // Float32 데이터 포인터
            let rowStride = bytesPerRow / MemoryLayout<Float32>.size
            guard let base = CVPixelBufferGetBaseAddress(depthMap) else { return nil }
            let floatBuffer = base.assumingMemoryBound(to: Float32.self)
            
            // 해당 픽셀의 depth (미터 단위)
            let depthAtCenter = floatBuffer[cy * rowStride + cx]
            return depthAtCenter
        }
    }
}
