# Balancer2

Balancer2 は、VStone 製の USB 接続マイコンロボットです。このリポジトリには、
PC 側から Balancer2 と通信するための Python スクリプトと、マイコン側で実行する
ファームウェアの MCUXpresso プロジェクトが含まれています。

以下は簡単な説明です。詳細なセットアップ手順や課題の進め方については、
[詳細手順](https://share.evernote.com/note/10070c28-ae6f-467a-1fde-5d3a872984fb)を参照してください。

## Python スクリプトの使い分け

OS によって使用する Python フォルダが異なります。

- Windows/Linux では `Python.Linux/` のスクリプトを使用します。
- macOS では `Python.mac/` のスクリプトを使用します。

macOS 版は、専用カーネルドライバを使わず、HID/libusb 経由でユーザランドから
Balancer2 と通信します。たとえば `Python.mac/graph_mem.py` は、実機のメモリマップ
をリアルタイム表示し、書き込み可能な項目を変更するためのツールです。

## ファームウェアの場所

ファームウェアは MCUXpresso IDE 上でビルドします。

最初の練習に使う標準ファームは、次の場所にあります。

```text
MCUxpresso/Default_Balancer/Balancer2_firm_Simple/
```

課題用・開発用テンプレートとして使うファームは、次の場所にあります。

```text
MCUxpresso/Balancer_2025/
```

MCUXpresso でビルドすると、実機へ転送する `.bin` ファイルは各プロジェクトの
`Debug/` フォルダに生成されます。たとえば Balancer_2025 の場合は次のファイルです。

```text
MCUxpresso/Balancer_2025/Debug/Balancer_2025.bin
```

## 基本手順

1. MCUXpresso IDE と必要な SDK をインストールします。
2. 標準ファーム `Balancer2_firm_Simple` を MCUXpresso でビルドします。

   ```text
   MCUxpresso/Default_Balancer/Balancer2_firm_Simple/
   ```
3. Windows を使う場合は、標準の BeuatoBalancer2 プログラマキットをダウンロードして
   インストールします。
4. Balancer2 実機を USB で接続し、生成された `.bin` ファイルをスクリプトで転送します。

   macOS では、ファームウェア書き込みモードで接続した Balancer2 が NXP LPC13xx IFLASH
   USB Mass Storage デバイスとして見えます。Finder でファイルコピーするのではなく、
   `Python.mac/flash` を使って転送します。

   ```sh
   cd Python.mac
   ./flash ../MCUxpresso/Default_Balancer/Balancer2_firm_Simple/Debug/Balancer2_firm_Simple.bin
   ```
   このデバイスでは、実際の Flash イメージ領域は LBA 4 から始まります。LBA 0 から
   書き込むと FAT メタデータを壊し、ファームウェアが起動しなくなることがあります。
   上記の macOS 用スクリプトは、内部で `tools/flash_lpc13xx_iflash.py` を呼び出し、
   デフォルトで LBA 4 から書き込みます。
5. 実機を通常モードで起動し、ファームウェアが動作することを確認します。
6. テレメトリを確認します。

   macOS では次のツールを使用します。

   ```sh
   cd Python.mac
   python graph_mem.py
   ```
   Windows/Linux では `Python.Linux/` フォルダ内の対応するスクリプトを使用します。
7. 標準ファームで動作確認ができたら、`Balancer_2025` をビルドして課題に取り組みます。

   ```text
   MCUxpresso/Balancer_2025/
   ```
   `Balancer_2025` では、`GAIN_OPTION1` から `GAIN_OPTION5` などのメモリマップ項目を
   `graph_mem.py` から変更しながら、制御パラメータの調整やテレメトリ確認を行えます。
