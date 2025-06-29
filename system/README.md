# Ubuntu 24.04 LTSのドライバ周りのシステム設定

* 99-Balancer2.rules
USBにBalacer2が接続されたとき，最初に接続されるHIDのドライバを切断して自動的に専用ドライバにつながるようにする設定．
/etc/udev/rules.dに格納する．

* balancer-rebind.sh
99-Balancer2.rulesから起動されるスクリプト．/etc/udev/rules.dに格納する．

* copyfirm
copyfirm.shを実行するときにroot passを必要としないようにする設定

* myBeuatoDriver.service
専用ドライバを起動時にinsmodするための設定．/etc/systemd/systemに格納して
sudo systemctl daemon-reload
sudo systemctl enable myBeuatoDriver.service
sudo systemctl start myBeuatoDriver.service
しておく．

