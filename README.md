# balancer

現時点での動作確認方法(動作未確認)

1.まず、[pololu-rpi-slave-arduino-library-balboa](https://github.com/oguran/pololu-rpi-slave-arduino-library-balboa)リポジトリ以下の
pololu-rpi-slave-arduino-library-balboa/examples/BalboaRPiSlaveDemo/BalboaRPiSlaveDemo.ino
をArduinoに書き込む。

2.rasp以下のコマンドを実効する。
`rosrun script/balance_main.py`
