# ArUco検出の実装 (src/aruco_detect/src/aruco_detect.cpp)

## 概要

このファイルは、OpenCVのArUcoモジュールを使用したArUcoマーカー検出ノードのコア実装を提供します。カメラトピックを購読し、マーカーを検出し、6自由度姿勢を推定し、結果を配信します。

## クラス: Aruco_detect

### コンストラクタ

**位置:** 5-40行目

**機能:**
- ROS2パラメータの宣言と取得
- 画像とカメラ情報のサブスクリプション作成
- 姿勢配列パブリッシャーの初期化
- マーカー変換用のTF2ブロードキャスターのセットアップ
- OpenCV可視化ウィンドウの作成

**主要パラメータ:**
- `desired_aruco_marker_id`: 検出するマーカー(デフォルト: 0)
- `marker_width`: マーカーの物理的なサイズ(メートル単位、デフォルト: 0.05m)
- `camera_topic`: 画像入力トピック
- `camera_info`: カメラキャリブレーショントピック

**依存関係:**
```cpp
#include "aruco_detect/aruco_detect.h"
```

## カメラキャリブレーションコールバック

### cameraInfoCallback()

**位置:** 43-80行目

**目的:** 姿勢推定用のカメラ内部パラメータを保存

**プロセス:**
1. キャリブレーション受信済みかチェック(一度のみの操作)
2. frame_idと画像サイズを抽出
3. 9要素配列からカメラマトリックス(K)を解析
4. 5要素配列から歪み係数(D)を解析
5. 補正マトリックス(R)を解析
6. 投影マトリックス(P)を解析
7. `cam_info_received`フラグを設定

**保存形式:**
```cpp
camera_matrix = (cv::Mat_<double>(3, 3) <<
    K[0], K[1], K[2],
    K[3], K[4], K[5],
    K[6], K[7], K[8]);
```

## 画像処理コールバック

### imageCallback()

**位置:** 83-104行目

**目的:** 受信したカメラ画像をマーカー検出用に処理

**プロセス:**
1. カメラキャリブレーションの受信を待機
2. パラメータの更新(実行時再設定を許可)
3. ROS画像をOpenCV形式(MONO8)に変換
4. cv_bridge例外の処理
5. マーカー検出関数の呼び出し

**エラー処理:**
```cpp
try {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
} catch (cv_bridge::Exception& e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return;
}
```

## マーカー検出と姿勢推定

### detectArucoMarkers()

**位置:** 106-243行目

**目的:** メイン検出および姿勢推定アルゴリズム

### アルゴリズムステップ

#### 1. 初期化 (108-114行目)
```cpp
std::vector<int> markerIds;
std::vector<std::vector<cv::Point2f>> markerCorners;
std::vector<cv::Vec3d> rvecs, tvecs;
```

#### 2. パラメータ検証 (121-125行目)
カメラマトリックスと歪み係数が初期化されていることを確認

#### 3. マーカー検出 (128行目)
```cpp
cv::aruco::detectMarkers(image, dictionary, markerCorners, markerIds);
```

事前定義されたArUco辞書(DICT_4X4_50)を使用

#### 4. 姿勢推定 (134行目)
```cpp
cv::aruco::estimatePoseSingleMarkers(
    markerCorners,    // 検出されたコーナー
    marker_width,     // マーカーの物理的なサイズ
    camera_matrix,    // 内部マトリックス
    dist_coeffs,      // 歪み係数
    rvecs,           // 出力: 回転ベクトル
    tvecs            // 出力: 並進ベクトル
);
```

#### 5. マーカーフィルタリング (137-139行目)
`desired_aruco_marker_id`に一致するマーカーのみを処理

#### 6. 座標変換 (145-184行目)

**OpenCVからROSへの変換:**

**位置マッピング:**
```cpp
pose.position.x = tvecs[i][2];   // 前方 (OpenCV Z → ROS X)
pose.position.y = -tvecs[i][0];  // 左 (OpenCV -X → ROS Y)
pose.position.z = tvecs[i][1];   // 上 (OpenCV Y → ROS Z)
```

**回転変換:**
```cpp
// ステップ 1: ロドリゲス回転ベクトル → 回転マトリックス
cv::Rodrigues(rvecs[i], rotation_matrix);

// ステップ 2: Eigen形式に変換
Eigen::Matrix3d rot = ... // rotation_matrixをコピー

// ステップ 3: 座標系変換を適用
Eigen::Matrix3d cv_to_ros;
cv_to_ros << 0,  0, 1,
            -1,  0, 0,
             0, -1, 0;
rot = cv_to_ros * rot * cv_to_ros.transpose();

// ステップ 4: クォータニオンに変換
Eigen::Quaterniond eigen_quat(rot);
tf2::Quaternion quat(eigen_quat.x(), eigen_quat.y(),
                     eigen_quat.z(), eigen_quat.w());

// ステップ 5: 180°ピッチオフセットを適用
tf2::Quaternion offset;
offset.setRPY(0, M_PI, 0);
quat = quat * offset;
quat.normalize();
```

#### 7. 可視化 (186-195行目)
画像にマーカー情報を表示:
```cpp
cv::putText(image, position_text,
            cv::Point(10, 80),
            cv::FONT_HERSHEY_SIMPLEX,
            0.5, cv::Scalar(255, 0, 0), 1);
```

#### 8. TF配信 (200-212行目)
```cpp
transformStamped.header.stamp = current_time;
transformStamped.header.frame_id = frame_id;  // カメラフレーム
transformStamped.child_frame_id = "aruco_marker_" + std::to_string(markerIds[i]);
// ... 並進と回転を設定
tf_broadcaster->sendTransform(transformStamped);
```

#### 9. 姿勢配列の配信 (213-218行目)
姿勢を配列に追加して配信

#### 10. FPS計算 (225-238行目)
加重移動平均:
```cpp
float duration = (duration_fps.seconds() +
                  (previous_duration * (weighted_avg-1))) / weighted_avg;
fps = 1.0f / duration;
```

## メイン関数

**位置:** 247-253行目

**機能:**
```cpp
int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<aruco_detect::Aruco_detect>());
    rclcpp::shutdown();
    return 0;
}
```

## 座標系規約

### OpenCVカメラフレーム
- **X軸:** 右
- **Y軸:** 下
- **Z軸:** 前方(光軸)

### ROSカメラフレーム
- **X軸:** 前方
- **Y軸:** 左
- **Z軸:** 上

### 変換マトリックス
```
[0   0  1]   [X_cv]   [Z_cv]      [X_ros]
[-1  0  0] × [Y_cv] = [-X_cv]  =  [Y_ros]
[0  -1  0]   [Z_cv]   [-Y_cv]     [Z_ros]
```

## 依存関係

### ROS2
- `rclcpp`: ノード管理
- `sensor_msgs`: ImageとCameraInfoメッセージ
- `geometry_msgs`: PoseとTransformメッセージ
- `tf2_ros`: 変換の配信
- `cv_bridge`: ROS-OpenCV変換

### 外部ライブラリ
- **OpenCV 4.x:** ArUco検出
- **Eigen3:** 行列演算
- **TF2:** クォータニオン演算

## パフォーマンスに関する考慮事項

- **FPS表示:** リアルタイムパフォーマンス監視
- **一度のみのキャリブレーション:** カメラパラメータは一度のみロード
- **選択的検出:** 目的のマーカーIDのみを処理
- **効率的な変換:** 可能な限り直接メモリマッピングを使用

## エラー処理

1. **キャリブレーション未受信:** imageCallbackから早期リターン
2. **cv_bridge失敗:** エラーをログして戻る
3. **マーカー未検出:** サイレント(ログスパムを避ける)
4. **パラメータ未初期化:** 使用前にチェック

## 今後の改善

- [ ] 複数マーカー追跡
- [ ] マーカー姿勢フィルタリング(カルマンフィルター)
- [ ] 動的マーカーサイズパラメータ
- [ ] 複数のArUco辞書のサポート
- [ ] マーカー検出信頼度のしきい値設定
