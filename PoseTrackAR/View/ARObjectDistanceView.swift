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
            // MARK: - AR 뷰 (백그라운드)
            LidarDistanceView(distance: $distance, detectedBox: $detectedBox)
                .edgesIgnoringSafeArea(.all)

            // MARK: - UI 오버레이
            VStack {
                // 상단 거리 정보 표시
                if let dist = distance {
                    InfoPillView(
                        text: String(format: "%.0f cm", dist * 100),
                        iconName: "ruler.fill"
                    )
                } else {
                    InfoPillView(
                        text: "객체를 찾는 중...",
                        iconName: "magnifyingglass"
                    )
                }
                Spacer()
            }
            .padding(.top, 60)
            .animation(.easeInOut, value: distance)

            // 감지된 바운딩 박스 표시
            if let box = detectedBox {
                BoundingBoxView(box: box)
                    .animation(.bouncy, value: box)
            }
        }
    }
}

// MARK: - ARKit + RealityKit 뷰 (UIViewRepresentable)
fileprivate struct LidarDistanceView: UIViewRepresentable {
    @Binding var distance: Float?
    @Binding var detectedBox: CGRect?
    
    // (makeUIView, updateUIView, makeCoordinator 함수는 그대로 유지)
    func makeUIView(context: Context) -> ARView {
        let arView = ARView(frame: .zero)
        arView.session.delegate = context.coordinator
        let config = ARWorldTrackingConfiguration()
        if ARWorldTrackingConfiguration.supportsSceneReconstruction(.mesh) {
            config.sceneReconstruction = .mesh
        }
        config.frameSemantics.insert(.sceneDepth) // sceneDepth 사용을 위해 추가
        arView.session.run(config, options: [.resetTracking, .removeExistingAnchors])
        context.coordinator.arView = arView
        return arView
    }

    func updateUIView(_ uiView: ARView, context: Context) {}

    func makeCoordinator() -> Coordinator {
        Coordinator(distance: $distance, detectedBox: $detectedBox)
    }

    // MARK: - Coordinator
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
                        self.processDetections(detectedObject, in: frame)
                    } else {
                        self.distance = nil
                        self.detectedBox = nil
                    }
                    self.isProcessingFrame = false
                }
            }
        }
        
        private func processDetections(_ detection: DetectionObject, in frame: ARFrame) {
            guard let arView = self.arView else { return }
            
            let viewRect = arView.bounds
            let pixelBoundingBox = CGRect(
                x: detection.boundingBox.origin.x * viewRect.width,
                y: detection.boundingBox.origin.y * viewRect.height,
                width: detection.boundingBox.size.width * viewRect.width,
                height: detection.boundingBox.size.height * viewRect.height
            )
            
            self.detectedBox = pixelBoundingBox
            self.distance = self.getDistance(to: pixelBoundingBox, in: frame, arView: arView)
        }
        
        // ★★★ Depth Map을 사용하도록 최적화된 getDistance 함수 ★★★
        private func getDistance(to boundingBox: CGRect, in frame: ARFrame, arView: ARView) -> Float? {
            guard let sceneDepth = frame.sceneDepth else { return nil }
            
            let depthMap = sceneDepth.depthMap
            CVPixelBufferLockBaseAddress(depthMap, .readOnly)
            defer { CVPixelBufferUnlockBaseAddress(depthMap, .readOnly) }
            
            let depthWidth = CVPixelBufferGetWidth(depthMap)
            let depthHeight = CVPixelBufferGetHeight(depthMap)
            
            // 뷰 좌표 -> 뎁스맵 좌표로 변환
            let viewSize = arView.bounds.size
            let depthPoint = CGPoint(
                x: (boundingBox.midX / viewSize.width) * CGFloat(depthWidth),
                y: (boundingBox.midY / viewSize.height) * CGFloat(depthHeight)
            )
            
            // 뎁스맵의 해당 픽셀에서 깊이 값(미터) 읽기
            let rowStride = CVPixelBufferGetBytesPerRow(depthMap) / MemoryLayout<Float32>.size
            guard let baseAddress = CVPixelBufferGetBaseAddress(depthMap) else { return nil }
            let floatBuffer = baseAddress.assumingMemoryBound(to: Float32.self)
            
            let x = Int(depthPoint.x)
            let y = Int(depthPoint.y)
            
            // 좌표가 뎁스맵 범위 내에 있는지 확인
            guard x >= 0, x < depthWidth, y >= 0, y < depthHeight else { return nil }
            
            let depthAtCenter = floatBuffer[y * rowStride + x]
            
            // 유효한 깊이 값인지 확인 (0이면 보통 유효하지 않음)
            return depthAtCenter > 0 ? depthAtCenter : nil
        }
    }
}


// MARK: - 재사용 가능한 UI 컴포넌트들

private struct InfoPillView: View {
    let text: String
    let iconName: String
    
    var body: some View {
        HStack(spacing: 8) {
            Image(systemName: iconName)
            Text(text)
                .fontWeight(.bold)
        }
        .font(.system(.headline, design: .rounded))
        .padding(.horizontal, 20)
        .padding(.vertical, 12)
        .background(.ultraThinMaterial, in: Capsule())
        .foregroundColor(.white)
        .transition(.opacity.combined(with: .move(edge: .top)))
    }
}

private struct BoundingBoxView: View {
    let box: CGRect
    private let cornerSize: CGFloat = 20
    private let cornerLineWidth: CGFloat = 4
    
    var body: some View {
        // 모서리만 강조하는 형태로 변경
        Path { path in
            // Top-left corner
            path.move(to: CGPoint(x: box.minX, y: box.minY + cornerSize))
            path.addLine(to: CGPoint(x: box.minX, y: box.minY))
            path.addLine(to: CGPoint(x: box.minX + cornerSize, y: box.minY))
            
            // Top-right corner
            path.move(to: CGPoint(x: box.maxX - cornerSize, y: box.minY))
            path.addLine(to: CGPoint(x: box.maxX, y: box.minY))
            path.addLine(to: CGPoint(x: box.maxX, y: box.minY + cornerSize))
            
            // Bottom-left corner
            path.move(to: CGPoint(x: box.minX, y: box.maxY - cornerSize))
            path.addLine(to: CGPoint(x: box.minX, y: box.maxY))
            path.addLine(to: CGPoint(x: box.minX + cornerSize, y: box.maxY))
            
            // Bottom-right corner
            path.move(to: CGPoint(x: box.maxX - cornerSize, y: box.maxY))
            path.addLine(to: CGPoint(x: box.maxX, y: box.maxY))
            path.addLine(to: CGPoint(x: box.maxX, y: box.maxY - cornerSize))
        }
        .stroke(Color.red, style: StrokeStyle(lineWidth: cornerLineWidth, lineCap: .round, lineJoin: .round))
        .shadow(color: .red.opacity(0.8), radius: 5, x: 0, y: 0)
    }
}
