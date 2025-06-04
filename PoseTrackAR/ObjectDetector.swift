//
//  ObjectDetector.swift
//  PoseTrackAR
//
//  Created by 전상민 on 6/4/25.
//

import Foundation
import UIKit
import CoreML
import YOLO


// YOLO 모델 사이즈
fileprivate let modelName = String("yolo11n")
// YOLO 입력 사이즈
fileprivate let yolo_input_size: CGSize = CGSize(width: 640, height: 384)


class ObjectDetector {
    private var detector: YOLO?
    
    init() {
        let _ = YOLO(modelName, task: .detect) { [weak self] result in
            guard let self = self else { return }
            switch result {
            case .success(let yolo):
                self.detector = yolo
                print("모델 로딩 성공")
            case .failure(let error):
                print("모델 로딩 실패: \(error)")
            }
        }
    }
    
    func detect(image: UIImage) {
        guard let detector = self.detector else {
            print("모델이 아직 로드되지 않았습니다.")
            return
        }
        
        let resized = image.resized(to: yolo_input_size)
        let result = detector(resized)
        print("YOLO 추론 결과: \(result)")
    }
    
    
}


extension UIImage {
    func resized(to targetSize: CGSize) -> UIImage {
        let renderer = UIGraphicsImageRenderer(size: targetSize)
        return renderer.image { _ in
            self.draw(in: CGRect(origin: .zero, size: targetSize))
        }
    }
}
