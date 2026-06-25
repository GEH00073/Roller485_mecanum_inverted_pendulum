# ESP-NOW パケット仕様

コントローラーは、チャンネル 3 で固定長 25 バイトの ESP-NOW ブロードキャストパケットを送信します。

```text
パケットサイズ: 25 バイト
byte 0-2: 宛先 MAC アドレス下位 3 バイト、またはブロードキャスト用の FF FF FF
byte 3-6: rudder / Psi、float
byte 7-10: throttle、float
byte 11-14: aileron / Phi、float
byte 15-18: elevator / Theta、float
byte 19: arm ボタン
byte 20: flip ボタン
byte 21: mode ボタン
byte 22: option ボタン
byte 23: proactive フラグ
byte 24: チェックサム、byte 0-23 の合計
```

現在公開しているファームウェアでは、byte 0-2 に `FF FF FF` を入れてブロードキャスト動作させています。本体側は、パケット長、宛先、チェックサムがすべて有効な場合だけ、最終受信時刻を更新します。
