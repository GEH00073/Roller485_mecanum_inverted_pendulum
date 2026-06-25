# 配線

## AtomS3R/AtomS3 と Roller485

AtomS3R または AtomS3 の PORT.A に、4台の Roller485 を Grove 4線ケーブルで並列接続します。

AtomS3R/AtomS3 の PORT.A 信号:

- G7
- G8
- 5V
- GND

4台の Roller485 は同じ I2C バスを共有し、I2C アドレスで識別します。

## Roller485 の電源

7.2 V バッテリーから、各 Roller485 の PWR485 入力へ 7.2 V と GND を供給します。

## Roller485 の配置と I2C アドレス

- LEFT = 左外側、アドレス 0x64
- LEFT2 = 左内側、アドレス 0x65
- RIGHT2 = 右内側、アドレス 0x66
- RIGHT = 右外側、アドレス 0x67

## コントローラーとの通信

Atom JoyStick コントローラーと本体側 AtomS3R/AtomS3 は、ESP-NOW のチャンネル 3 で無線接続します。
