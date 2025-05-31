//
//  Untitled.swift
//  PoseTrackAR
//
//  Created by 전상민 on 6/2/25.
//

import ARKit
import Combine


struct Intrinsics {
    let fx: Float
    let fy: Float
    let cx: Float
    let cy: Float
    
    let width: Int32
    let height: Int32
}

class ARSessionManager: NSObject, ObservableObject {

    private let session = ARSession()

    /*
     intrinsics.columns.0 = [f_x, 0,     0]   // X축 방향 열
     intrinsics.columns.1 = [s,   f_y,   0]   // Y축 방향 열
     intrinsics.columns.2 = [c_x, c_y,   1]   // 주점과 1
     */
    @Published var intrinsics: Intrinsics?
    @Published var res: CGSize = .zero

    override init() {
        super.init()
        session.delegate = self

        let cfg = ARWorldTrackingConfiguration()
        cfg.frameSemantics = []          // 필요 옵션만
        session.run(cfg)
    }


}

extension CameraManager: ARSessionDelegate {
    func session(_ session: ARSession, didUpdate frame: ARFrame) {
        let ins: simd_float3x3 = frame.camera.intrinsics
        let res = frame.camera.imageResolution
        
        intrinsics = Intrinsics(
            fx: ins.columns.0.x, fy: ins.columns.1.y,
            cx: ins.columns.2.x, cy: ins.columns.2.y,
            width: Int32(res.width), height: Int32(res.height)
        )
        
    }
}
