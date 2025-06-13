# feetech\_controller

## 概要

**feetech\_controller**は、FeetechサーボをROS2 Humbleでガンガン動かすためのC++ノードだ！
ジョイパッド入力でパン・チルト・ロール・ロック・発射まで全部制御できる、爆速ロボットコントローラ！

---

## ファイル構成

* `pan_tilt_node.cpp`
  メインノード。ジョイからサーボ全部コントロール！
* `feetech_handler.hpp`
  Feetechサーボの操作クラス
* `pan_tilt_ros_if.hpp`
  ROS2インターフェース
* `serial_port_handler.hpp`
  シリアル通信用

---

## 依存

* ROS2 Humble
* rclcpp
* sensor\_msgs
* std\_msgs

---

## ビルド方法

```bash
colcon build --packages-select feetech_controller
source install/setup.bash
```

---

## 起動方法

```bash
ros2 run feetech_controller pan_tilt_node
```

---

## ジョイパッド割り当て一覧

| 機能     | Joy軸/ボタン            | 動作・説明              |
| ------ | ------------------- | ------------------ |
| 前後移動   | axes\[1]            | 前進・後退              |
| 旋回     | axes\[2]            | 左右旋回               |
| ロール    | axes\[5] / axes\[4] | ロール・アンロール          |
| 発射ON   | Bボタン(1)             | 発射ON（サーボ23: 150）   |
| 発射OFF  | Xボタン(2)             | 発射OFF（サーボ23: 1050） |
| アンロック  | ↑(11)               | アンロック（サーボ24: 250）  |
| ロック    | ↓(12)               | ロック（サーボ24: 700）    |
| トルクOFF | LB(9)               | 全サーボのトルクOFF        |
| トルクON  | RB(10)              | 全サーボのトルクON         |

* サーボIDは 20:右パン, 21:左パン, 22:ロール, 23:発射, 24:ロック で固定！
* ボタン番号はコントローラーによって異なる場合があるから注意！

---

## 実装ポイント

* ジョイ入力は`onJoyReceived`でガッツリ受け取り
* 各サーボへ`setCommand`/`setCommandCustomPos`/`setTorqueEnable`で一括制御
* ジョイント状態もpublishしてるので可視化(RVizなど)もOK
* 異常時はエラー出して止まる安心設計
* 速度・ガードタイムも調整済み

---

## カスタマイズ

* サーボIDや速度スケールは`pan_tilt_node.cpp`を編集して調整可能！
* ジョイパッドのボタン割り当ても自由に変更できる！

---

## 注意点

* サーボID、配線、通信ポートは**事前確認必須！**
* 通信失敗時はノードが起動しません

---

## 問い合わせ・貢献

質問・改善案は**Issue**や**Pull Request**で待ってるぜ！！
一緒にロボットを進化させよう🔥

---

## ライセンス

自由に使ってOK！（自己責任で！）

---

## クイックスタート

```bash
ros2 run feetech_controller pan_tilt_node
```

---

**爆速でパンチルトぶん回せ！！！**

---

他に足したい情報・Q\&Aや図解とか、要望があればどんどん言って！
