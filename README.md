# feetech\_controller README

## 概要

**feetech\_controller**は、FeetechサーボをROS2 Humbleで制御できる最強ノードや！
C++製で、ジョイパッド操作やロボットからパンチルト・ロール・発射・ロックまで全部動かせる！

---

## ファイル構成

* `pan_tilt_node.cpp`
  メインノード
* `feetech_handler.hpp`
  Feetechサーボの制御クラス
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

## ビルド

```bash
colcon build --packages-select feetech_controller
source install/setup.bash
```

---

## 実行

```bash
ros2 run feetech_controller pan_tilt_node
```

---

## 主な関数・役割

### setCommand

```cpp
void setCommand(const int id, const float value);
```

* **概要**：
  指定したサーボIDに速度コマンドを送信する関数。
  valueは-1.0～1.0の範囲で、内部でスケーリングしてサーボに伝える。

* **引数**

  * id : サーボID（例：20, 21, 22など）
  * value : サーボ速度（-1.0:最大逆回転、0:停止、1.0:最大正回転）

* **特徴**

  * コマンド送信後、1ms(usleep)のガードで暴走防止！

---

### setCommandCustomPos

```cpp
void setCommandCustomPos(const int id, const int value);
```

* **概要**：
  指定したサーボIDに任意のポジション値を直接送る関数。
  例：発射アクションやロック動作に使う！

* **引数**

  * id : サーボID（23, 24など）
  * value : サーボ目標位置値

* **特徴**

  * コマンド送信後、100msのガードタイム

---

### setTorqueEnable

```cpp
void setTorqueEnable(const int id, const bool state);
```

* **概要**：
  指定したサーボIDのトルク（駆動ON/OFF）を切り替える関数。

* **引数**

  * id : サーボID
  * state : true(1) でトルクON, false(0) でトルクOFF

* **特徴**

  * コマンド送信後、100msのガードタイム

---

### その他のポイント

* **JointState publish**
  現在の各サーボの状態（位置・速度）を/joint\_statesでパブリッシュ
  RVizで可視化もできる！

* **シリアル初期化**
  サーボの初期化に失敗するとノードが強制停止

* **ジョイ入力をonJoyReceivedで全部拾って多機能動作可能**
  動作例はコード内コメントを参照

---

## カスタマイズ

* サーボID・スケール値・コマンド内容は`pan_tilt_node.cpp`内で編集可能！
* 新しい動作追加やジョイ割当の変更も簡単！

---

## 注意

* サーボIDや配線、通信ポートを事前にしっかり確認してね！
* 通信エラー時はノードが即終了するよ

---

## お問い合わせ・貢献

質問・改善案は**Issue**や**Pull Request**で！
一緒にFeetechサーボぶん回そうぜ！

---

## クイックスタート

```bash
ros2 run feetech_controller pan_tilt_node
```

---

**さぁ、パンチルト人生をスタートしよう！爆速で！**

---

もし追加で説明したい関数や、サーボID割り当ての表、よくあるトラブルシューティングとか要望あったらすぐ言ってな！
