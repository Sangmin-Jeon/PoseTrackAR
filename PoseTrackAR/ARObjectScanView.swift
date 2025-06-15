//
//  123.swift
//  PoseTrackAR
//
//  Created by 전상민 on 6/16/25.
//

import SwiftUI
import ARKit
import RealityKit

struct ARObjectScanAndDetectView: View {
    @State private var scannedObject: ARReferenceObject? = nil
    @State private var featurePointCount: Int = 0

    /// 자동으로 스캔 끝낼 피처 포인트 목표치
    private let targetPointCount = 5_000

    var body: some View {
        ZStack {
            if let reference = scannedObject {
                ObjectDetectorView(referenceObject: reference)
                    .edgesIgnoringSafeArea(.all)
            } else {
                ObjectScannerView(
                    featurePointCount: $featurePointCount,
                    targetCount: targetPointCount
                ) { obj in
                    scannedObject = obj
                }
                .edgesIgnoringSafeArea(.all)

                VStack {
                    Spacer()
                    Text("피처 포인트: \(featurePointCount)")
                        .font(.headline)
                        .foregroundColor(.white)
                    ProgressView(value: Double(featurePointCount),
                                 total: Double(targetPointCount))
                        .padding(.horizontal, 40)
                        .accentColor(.green)
                    Button("스캔 완료") {
                        // 수동으로 스캔 끝내기
                        NotificationCenter.default.post(name: .finishScanning, object: nil)
                    }
                    .padding(.top, 8)
                    .foregroundColor(.white)
                    .padding(.horizontal, 16)
                    .padding(.vertical, 8)
                    .background(Color.black.opacity(0.5))
                    .cornerRadius(8)
                    .padding(.bottom, 30)
                }
            }
        }
    }
}

extension Notification.Name {
    static let finishScanning = Notification.Name("finishScanning")
}

struct ObjectScannerView: UIViewRepresentable {
    @Binding var featurePointCount: Int
    let targetCount: Int
    let onFinish: (ARReferenceObject) -> Void

    func makeUIView(context: Context) -> ARSCNView {
        let view = ARSCNView(frame: .zero)
        view.autoenablesDefaultLighting = true
        view.debugOptions = [.showFeaturePoints, .showWorldOrigin]
        view.session.delegate = context.coordinator
        view.delegate = context.coordinator

        // Coaching Overlay
        let coaching = ARCoachingOverlayView()
        coaching.session = view.session
        coaching.goal = .horizontalPlane
        coaching.activatesAutomatically = true
        coaching.delegate = context.coordinator
        coaching.translatesAutoresizingMaskIntoConstraints = false
        view.addSubview(coaching)
        NSLayoutConstraint.activate([
            coaching.topAnchor.constraint(equalTo: view.topAnchor),
            coaching.bottomAnchor.constraint(equalTo: view.bottomAnchor),
            coaching.leadingAnchor.constraint(equalTo: view.leadingAnchor),
            coaching.trailingAnchor.constraint(equalTo: view.trailingAnchor),
        ])
        context.coordinator.coaching = coaching

        // 한 번만 실행
        let config = ARObjectScanningConfiguration()
        config.planeDetection = [.horizontal, .vertical]
        view.session.run(config, options: [.resetTracking, .removeExistingAnchors])

        // 수동 종료 알림 구독
        NotificationCenter.default.addObserver(
            context.coordinator,
            selector: #selector(Coordinator.finishManually),
            name: .finishScanning,
            object: nil
        )

        return view
    }

    func updateUIView(_ uiView: ARSCNView, context: Context) {
        // NO-OP: makeUIView 에서만 run
    }

    func makeCoordinator() -> Coordinator { Coordinator(onFinish: onFinish, targetCount: targetCount) }

    class Coordinator: NSObject, ARSessionDelegate, ARSCNViewDelegate, ARCoachingOverlayViewDelegate {
        var onFinish: (ARReferenceObject) -> Void
        var targetCount: Int
        weak var coaching: ARCoachingOverlayView?
        private var finished = false

        init(onFinish: @escaping (ARReferenceObject) -> Void, targetCount: Int) {
            self.onFinish = onFinish
            self.targetCount = targetCount
        }

        func session(_ session: ARSession, didUpdate frame: ARFrame) {
            guard !finished else { return }
            let count = frame.rawFeaturePoints?.points.count ?? 0
            DispatchQueue.main.async {
                self.featurePointCount = count
            }
            if count >= targetCount {
                finishScanning(session: session)
            }
        }

        func renderer(_ renderer: SCNSceneRenderer,
                      didAdd node: SCNNode,
                      for anchor: ARAnchor) {
            guard !finished, let objAnchor = anchor as? ARObjectAnchor else { return }
            finishScanning(session: (renderer as! ARSCNView).session, with: objAnchor.referenceObject)
        }

        // CoachingOverlay가 비활성화될 때 호출
        func coachingOverlayViewWillActivate(_ coachingOverlayView: ARCoachingOverlayView) {
            // do nothing
        }
        func coachingOverlayViewDidDeactivate(_ coachingOverlayView: ARCoachingOverlayView) {
            // 코칭이 꺼지면(Plane 잡혔을 때) 추가 액션 필요 없음
        }
        func coachingOverlayViewSessionReset(_ coachingOverlayView: ARCoachingOverlayView) {
            // do nothing
        }

        @objc func finishManually() {
            if let session = coaching?.session {
                finishScanning(session: session)
            }
        }

        private func finishScanning(session: ARSession) {
            finishScanning(session: session, with: nil)
        }
        private func finishScanning(session: ARSession, with refObj: ARReferenceObject?) {
            finished = true
            session.pause()
            coaching?.setActive(false, animated: true)
            if let obj = refObj {
                DispatchQueue.main.async {
                    self.onFinish(obj)
                }
            }
        }

        // 바인딩 업데이트
        var featurePointCount: Int = 0 {
            didSet { DispatchQueue.main.async { self.parentBinding?(self.featurePointCount) } }
        }
        private var parentBinding: ((Int)->Void)?
        func bindFeaturePointCount(_ update: @escaping (Int)->Void) {
            parentBinding = update
        }
    }
}

struct ObjectDetectorView: UIViewRepresentable {
    var referenceObject: ARReferenceObject

    func makeUIView(context: Context) -> ARView {
        let arView = ARView(frame: .zero)
        arView.automaticallyConfigureSession = false
        arView.session.delegate = context.coordinator
        return arView
    }

    func updateUIView(_ uiView: ARView, context: Context) {
        var config = ARWorldTrackingConfiguration()
        config.detectionObjects = [referenceObject]
        config.planeDetection = [.horizontal]
        uiView.session.run(config,
                           options: [.resetTracking, .removeExistingAnchors])
    }

    func makeCoordinator() -> Coordinator { Coordinator() }

    class Coordinator: NSObject, ARSessionDelegate {
        func session(_ session: ARSession, didAdd anchors: [ARAnchor]) {
            for anchor in anchors {
                if let objAnchor = anchor as? ARObjectAnchor {
                    print("[SwiftUI] Detected object at transform:\n\(objAnchor.transform)")
                }
            }
        }
    }
}
