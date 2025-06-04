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


class ObjectDetector {

    init(modelName: String = "yolo11n") {
        let _ = YOLO(modelName, task: .detect) { [weak self] result in
            guard let self = self else { return }
            switch result {
            case .success(let yolo):
                self.detect(with: yolo)
                print("모델 로딩 성공")
                
            case .failure(let error):
                print("모델 로딩 실패: \(error)")
            }
        }
    }

    private func detect(with detector: YOLO) {
        guard let url = URL(string: "https://ultralytics.com/images/bus.jpg") else {
            print("URL이 유효하지 않습니다.")
            return
        }

        URLSession.shared.dataTask(with: url) { data, _, error in
            if let error = error {
                print("네트워크 오류: \(error.localizedDescription)")
                return
            }

            guard let data = data, let image = UIImage(data: data) else {
                print("이미지 변환 실패")
                return
            }

            let result = detector(image)
            print("Result: \(result)")
            
        }.resume()
    }
}
