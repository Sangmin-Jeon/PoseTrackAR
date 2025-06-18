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
                    Text(String(format: "%.2f m", distance))
                        .font(.largeTitle).fontWeight(.bold)
                        .foregroundColor(.white)
                        .padding()
                        .background(Color.black.opacity(0.6))
                        .cornerRadius(10)
                } else {
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

    // ARKit 델리게이트와 로직 처리
    class Coordinator: NSObject, ARSessionDelegate {
        @Binding var distance: Float?
        @Binding var detectedBox: CGRect?
        weak var arView: ARView?
        
        let objectDetector = ObjectDetector()
        private var isProcessingFrame = false // 프레임 중복 처리 방지 플래그

        init(distance: Binding<Float?>, detectedBox: Binding<CGRect?>) {
            _distance = distance
            _detectedBox = detectedBox
        }
        
        // 매 프레임 업데이트될 때 호출
        func session(_ session: ARSession, didUpdate frame: ARFrame) {
            // 이미 다른 프레임을 처리 중이면 건너뛰기
            guard !isProcessingFrame else { return }
            
            self.isProcessingFrame = true
            
            // 현재 프레임의 이미지를 UIImage로 변환
            let pixelBuffer = frame.capturedImage
            var ciImage = CIImage(cvPixelBuffer: pixelBuffer)
            ciImage = ciImage.oriented(.right)
            let uiImage = UIImage(ciImage: ciImage)
            
            // ObjectDetector로 객체 탐지 실행
            objectDetector.detect(image: uiImage)
            guard let arView = self.arView else {
                self.isProcessingFrame = false
                return
            }
            arView.debugOptions.insert(.showSceneUnderstanding)
            
            if let obj = objectDetector.detection_obj {
                // 정규화된 바운딩 박스를 ARView의 픽셀 좌표로 변환
                let viewRect = arView.bounds
                let pixelBoundingBox = CGRect(
                    x: obj.boundingBox.origin.x * viewRect.width,
                    y: obj.boundingBox.origin.y * viewRect.height,
                    width: obj.boundingBox.size.width * viewRect.width,
                    height: obj.boundingBox.size.height * viewRect.height
                )
                
                // 거리 계산
                let calculatedDistance = self.getDistance(to: pixelBoundingBox, in: arView)
                
                // UI 업데이트 (메인 스레드에서)
                DispatchQueue.main.async {
                    self.distance = calculatedDistance
                    self.detectedBox = pixelBoundingBox
                }
                // 프레임 처리 완료
                self.isProcessingFrame = false
                
            }
            else {
                // 감지된 객체가 없으면 UI 초기화
                DispatchQueue.main.async {
                    self.distance = nil
                    self.detectedBox = nil
                    self.isProcessingFrame = false
                }
            }
            
        }
        
        /// 특정 바운딩 박스의 중심까지의 거리를 계산하는 핵심 함수
        private func getDistance(to boundingBox: CGRect, in arView: ARView) -> Float? {
            let centerPoint = CGPoint(x: boundingBox.midX, y: boundingBox.midY)

            guard let raycastQuery = arView.makeRaycastQuery(from: centerPoint, allowing: .existingPlaneGeometry, alignment: .any) else {
                return nil
            }
            
            let results = arView.session.raycast(raycastQuery)
            guard let result = results.first else {
                print("거리 결과 없음 ~~~")
                return nil
            }

            let cameraPosition = arView.cameraTransform.translation
            let hitPosition = simd_make_float3(result.worldTransform.columns.3)
            
            return simd_distance(cameraPosition, hitPosition)
        }
    }
}
