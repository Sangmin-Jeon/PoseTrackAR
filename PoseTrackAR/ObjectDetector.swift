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



struct DetectionObject: Identifiable {
    let id = UUID()
    let label: String
    let confidence: Float
    let boundingBox: CGRect  // normalized (0~1) 기준 or 실제 pixel 기준
    
    func boundingBoxRect(for imageSize: CGSize) -> CGRect {
        return CGRect(
            x: CGFloat(boundingBox.origin.x) * imageSize.width,
            y: CGFloat(boundingBox.origin.y) * imageSize.height,
            width: CGFloat(boundingBox.width) * imageSize.width,
            height: CGFloat(boundingBox.height) * imageSize.height
        )
    }
}

// YOLO 모델 사이즈
fileprivate let modelName = String("best")
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
        
        let detections = result.boxes.map {
            DetectionObject(
                label: $0.cls,
                confidence: $0.conf,
                boundingBox: $0.xywhn
            )
        }
        
        print(detections)
        
        if !detections.isEmpty {
            receive_object_detection_info(self.convertToCStruct(from: detections[0]))
        }
        

        // 원본 이미지 크기에 맞게 시각화
        // let imageWithBoxes = image.drawDetections(detections)
        // TODO: YOLO 객체 추적 이미지
        // processedImageSubject.send(imageWithBoxes)
        
        
    }
    
    func convertToCStruct(from object: DetectionObject) -> DetectionObject_C {
        var cObject = DetectionObject_C()
        
        // 문자열 변환
        let labelCString = (object.label as NSString).utf8String
        strncpy(&cObject.label.0, labelCString, 31)
        
        // 바운딩 박스 변환
        cObject.bbox_x = Float(object.boundingBox.origin.x)
        cObject.bbox_y = Float(object.boundingBox.origin.y)
        cObject.bbox_width = Float(object.boundingBox.size.width)
        cObject.bbox_height = Float(object.boundingBox.size.height)

        cObject.confidence = object.confidence
        
        return cObject
    }
    
}


extension UIImage {
    func resized(to targetSize: CGSize) -> UIImage {
        let renderer = UIGraphicsImageRenderer(size: targetSize)
        return renderer.image { _ in
            self.draw(in: CGRect(origin: .zero, size: targetSize))
        }
    }
    
    func drawDetections(_ detections: [DetectionObject]) -> UIImage {
        let format = UIGraphicsImageRendererFormat.default()
        format.scale = self.scale
        let renderer = UIGraphicsImageRenderer(size: self.size, format: format)
        
        return renderer.image { context in
            self.draw(at: .zero)
            
            for detection in detections {
                let rect = detection.boundingBoxRect(for: self.size)
                
                // Draw box
                context.cgContext.setStrokeColor(UIColor.red.cgColor)
                context.cgContext.setLineWidth(2)
                context.cgContext.stroke(rect)
                
                // Draw label
                let label = "\(detection.label) \(Int(detection.confidence * 100))%"
                let attrs: [NSAttributedString.Key: Any] = [
                    .font: UIFont.systemFont(ofSize: 14),
                    .foregroundColor: UIColor.red,
                    .backgroundColor: UIColor.white.withAlphaComponent(0.7)
                ]
                let textSize = label.size(withAttributes: attrs)
                let textRect = CGRect(x: rect.origin.x, y: rect.origin.y - textSize.height, width: textSize.width, height: textSize.height)
                label.draw(in: textRect, withAttributes: attrs)
            }
        }
    }
}
