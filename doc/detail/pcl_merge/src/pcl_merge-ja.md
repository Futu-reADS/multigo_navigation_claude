# ポイントクラウド統合実装 (src/pcl_merge/src/pcl_merge.cpp)

## 概要

このファイルは、TF2変換とボクセルグリッドダウンサンプリングを使用したマルチセンサーポイントクラウド融合を実装します。複数のポイントクラウドトピックを購読し、それらを共通フレームに変換し、統合されダウンサンプリングされたクラウドを配信します。

## クラス: PCLMergeNode

### コンストラクタ

**場所:** 3-40行目

**主な操作:**

1. **パラメータ宣言** (8-12行目)
```cpp
this->declare_parameter("output_frame", "base_link");
this->declare_parameter("output_topic", "pcl_merged");
this->declare_parameter("input_topics",
    std::vector<std::string>({"scan_pointcloud",
                               "/camera_depth_top/camera_depth/points"}));
```

2. **動的購読** (26-29行目)
```cpp
for (size_t i = 0; i < input_topics_.size(); i++) {
    auto callback = [this, i](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        cloud_callback(i, msg);
    };
    subscribers_.push_back(this->create_subscription<sensor_msgs::msg::PointCloud2>(
        input_topics_[i], 10, callback));
}
```

**ラムダキャプチャ:** クラウド保存用に各コールバックにインデックス `i` をバインド

3. **TF2設定** (35-36行目)
```cpp
tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
```

4. **タイマー設定** (39行目)
```cpp
timer_ = this->create_wall_timer(std::chrono::milliseconds(33),
                                  std::bind(&PCLMergeNode::timer_callback, this));
```
約30 Hz (33ms周期)で配信

## ポイントクラウドコールバック

### cloud_callback()

**場所:** 42-116行目

**目的:** 個々のセンサーからポイントクラウドを受信、変換、保存

### 処理パイプライン

#### 1. インデックス検証 (43-44行目)
```cpp
if (index >= clouds_.size())
    return;
```

#### 2. TF変換ルックアップ (47-56行目)
```cpp
transform = tf_buffer_->lookupTransform(
    output_frame_,              // ターゲット: base_link
    msg->header.frame_id,       // ソース: センサーフレーム
    rclcpp::Time(0)             // 利用可能な最新
);
```

**エラー処理:** 変換が利用できない場合は警告を出してクラウドを破棄

#### 3. ポイントタイプ変換 (58-100行目)

**戦略:** 直接PointXYZI変換を試行、失敗時にはPointXYZRGBへフォールバック

**直接変換の試行:**
```cpp
try {
    pcl::fromROSMsg(*msg, *pcl_cloud);
    conversion_success = true;
} catch (std::exception &e) {
    // RGB変換を試行
}
```

**RGBから強度への変換:**
```cpp
pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_cloud(...);
pcl::fromROSMsg(*msg, *rgb_cloud);

for (size_t i = 0; i < rgb_cloud->points.size(); ++i) {
    // RGB値を抽出
    uint32_t rgb = *reinterpret_cast<const int*>(&pt.rgb);
    uint8_t r = (rgb >> 16) & 0x0000ff;
    uint8_t g = (rgb >> 8)  & 0x0000ff;
    uint8_t b = (rgb)       & 0x0000ff;

    // 輝度計算式: I = 0.299*R + 0.587*G + 0.114*B
    pt_i.intensity = 0.299 * r + 0.587 * g + 0.114 * b;
}
```

**根拠:** 知覚的輝度のためのITU-R BT.601輝度係数

#### 4. 空間変換 (103-112行目)
```cpp
pcl_ros::transformPointCloud(*pcl_cloud, *pcl_cloud_transformed, transform);
```

すべてのクラウドを `output_frame` に整列させるためにTF2変換を適用

#### 5. 保存 (115行目)
```cpp
clouds_[index] = pcl_cloud_transformed;
```

各センサーの最新クラウドを個別に保存

## 統合と配信

### timer_callback()

**場所:** 119-156行目

**目的:** 定期的にすべてのクラウドを統合し、ダウンサンプリングされた結果を配信

### 処理ステップ

#### 1. 有効性チェック (121-130行目)
```cpp
bool any_valid = false;
for (const auto &cloud : clouds_) {
    if (cloud != nullptr) {
        any_valid = true;
        break;
    }
}
if (!any_valid) return;
```

少なくとも1つのクラウドが利用可能な場合のみ続行

#### 2. クラウドの統合 (133-138行目)
```cpp
pcl::PointCloud<pcl::PointXYZI>::Ptr merged_cloud(...);
for (const auto& cloud : clouds_) {
    if (cloud) {
        *merged_cloud += *cloud;  // PCLクラウドの連結
    }
}
```

**演算子オーバーロード:** PCLの `operator+=` がポイントクラウドを連結

#### 3. ボクセルグリッドダウンサンプリング (141-146行目)
```cpp
pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;
voxel_filter.setInputCloud(merged_cloud);
voxel_filter.setLeafSize(0.05f, 0.05f, 0.05f);  // 5cm立方体
voxel_filter.filter(*downsampled_cloud);
```

**効果:** 各5cm³ボクセル内のすべてのポイントをその重心に置き換え

**利点:**
- ポイント数を削減(通常5-10倍)
- 重複するセンサーからの重複ポイントを削除
- ポイント間隔を正規化

#### 4. ROSメッセージ変換 (149-151行目)
```cpp
sensor_msgs::msg::PointCloud2 output_msg;
pcl::toROSMsg(*downsampled_cloud, output_msg);
output_msg.header.frame_id = output_frame_;
```

#### 5. 配信 (154行目)
```cpp
pub_->publish(output_msg);
RCLCPP_INFO_ONCE(...);  // 最初の配信のみログ
```

## メイン関数

**場所:** 159-166行目

```cpp
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PCLMergeNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

## データフロー図

```
センサー 1 → cloud_callback(0) → 変換 → Store[0] ─┐
センサー 2 → cloud_callback(1) → 変換 → Store[1] ─┤
センサー N → cloud_callback(N) → 変換 → Store[N] ─┴→ timer_callback()
                                                         ↓
                                                    クラウドを統合
                                                         ↓
                                                    ボクセルダウンサンプリング
                                                         ↓
                                                    統合クラウドを配信
```

## メモリ管理

- **クラウド保存:** `std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>`
- **上書きポリシー:** 各センサーからの最新クラウドが以前のものを置き換え
- **一時クラウド:** タイマーサイクルごとに作成され、自動的に破棄
- **スマートポインタ:** RAIIが適切なクリーンアップを保証

## パフォーマンス最適化

### 効率的なタイプ処理
- 可能な場合は直接PointXYZI変換
- フォールバックとしてのみRGB変換
- 不必要なメモリコピーを回避

### ボクセルグリッドの利点
- **入力:** 潜在的に数百万のポイント
- **出力:** 通常10-100kポイント
- **速度:** O(n)フィルタリング
- **メモリ:** 大幅な削減

### タイマーベースの配信
- 入力レートから出力レートを分離
- センサーレートに関係なく一定の30 Hz出力
- 非同期処理

## エラーシナリオ

| エラー | 処理 | 影響 |
|-------|------|------|
| TF変換が見つからない | 警告して破棄 | このサイクルではクラウドが統合されない |
| 無効なポイントタイプ | RGB変換を試行 | 適切なフォールバック |
| RGB変換失敗 | 警告して破棄 | センサーが除外される |
| 有効なクラウドなし | サイレントリターン | 出力が配信されない |

## 設定の推奨事項

### ボクセルリーフサイズの調整
- **0.01m:** 高詳細、処理が遅い
- **0.05m:** バランス型(デフォルト、pcl_merge.cpp:144行目)
- **0.10m:** 高速、詳細度低い

**ボクセルサイズのトレードオフ:** デフォルトの0.05m (5cm)ボクセルサイズは、ポイントクラウドの詳細度と処理性能の間の良好なバランスを提供します。この設定:
- ポイント数を大幅に削減(典型的な削減率: シーンの複雑さに応じて70-90%)
- ナビゲーションと障害物検出のための十分な詳細を維持
- 処理レイテンシをリアルタイム制約内に保つ(典型的な統合クラウドで約10-20ms)
- 5cmより大きい幾何学的特徴を保持

より細かい詳細が必要なアプリケーション(例: 精密ドッキング、小物体検出)の場合は、0.01-0.03mに削減してください。要求の少ないシナリオでパフォーマンスを向上させる場合は、0.08-0.10mに増やしてください。

### 出力フレームの選択
- **base_link:** ロボット中心(推奨)
- **map:** グローバルフレーム
- **odom:** フィルタリングされたオドメトリ用

## 依存関係

- **PCL:** ポイントクラウド構造とフィルター
- **PCL Conversions:** ROS ↔ PCL変換
- **PCL ROS:** TF統合
- **TF2:** 変換管理
- **Eigen3:** 行列演算(PCL経由)
