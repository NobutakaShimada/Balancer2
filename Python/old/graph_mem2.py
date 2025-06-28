#!/usr/bin/env python3
import tkinter as tk
from tkinter import ttk, filedialog, messagebox, simpledialog
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import struct
import time
from collections import deque
import matplotlib.colors as mcolors
import csv
from datetime import datetime
import ast

from allmemquery import read_all_memory
from BeuatoMemMap import memory_map
from BeuatoMemMap import TYPE_FMT

# 更新間隔（ms）
UPDATE_INTERVAL = 10
# グラフ表示点数（この点数を常に表示）
GRAPH_POINTS = 100
# プロット用の時系列データ数（更新間隔に応じて自動計算）
WINDOW_SIZE = GRAPH_POINTS  # 表示点数と同じに設定
# デバッグモード
DEBUG_MODE = False  # True にするとデバッグ情報を出力

# driver debug log
IOCTL_DEBUG = 0x40044200
IOCTL_READ_MODE = 0x40044201

# automatic string to numerical conversion (from interactive.py)
convert = lambda s: ast.literal_eval(s) if s.replace('.','').replace('-','').replace('e','').replace('E','').replace('+','').isdigit() else s

def make_write_command(name, data) -> bytes:
    """
    変数名と書き込みデータから書き込みコマンドを生成
    interactive.pyの形式に合わせて修正
    """
    try:
        addr, length, vartype = memory_map[name]
    except KeyError:
        raise ValueError(f"Unknown variable: {name!r}")
    
    # データをバイナリ形式に変換
    try:
        converted_data = convert(str(data))
        # interactive.pyと同じ形式でバイナリデータを作成
        bdata = struct.pack(TYPE_FMT[vartype], converted_data)
        print(f"Debug: name={name}, addr={addr}, length={length}, vartype={vartype}")
        print(f"Debug: converted_data={converted_data}, bdata={bdata.hex()}")
    except (ValueError, struct.error) as e:
        raise ValueError(f"Invalid data format for {vartype}: {data}")
    
    # interactive.pyの形式に合わせてASCIIコマンドを作成
    addr_hex = format(addr, 'x')
    len_hex = format(length, 'x')
    cmd = f"w {addr_hex} {len_hex} {bdata.hex(' ')}"
    print(f"Debug: ASCII command = '{cmd}'")
    
    return cmd.encode('ascii')

def send_write_command(dev, command):
    """
    書き込みコマンドを送信
    interactive.pyの形式に合わせて修正
    """
    try:
        print(f"Debug: Sending command: {command}")
        dev.write(command)
        
        # 応答を読み取り（書き込み完了確認）
        response = dev.read(256)
        print(f"Debug: Write response: {response}")
        
        # 応答の解析（必要に応じて）
        if response:
            try:
                response_str = response.decode('ascii', errors='ignore').strip()
                print(f"Debug: Response string: '{response_str}'")
                # 書き込み成功の判定ロジックをここに追加
                return True
            except Exception as e:
                print(f"Debug: Response decode error: {e}")
                return False
        else:
            print("Debug: No response received")
            return False
            
    except Exception as e:
        print(f"Write command failed: {e}")
        import traceback
        traceback.print_exc()
        return False


class MemoryViewerApp:
    def __init__(self, root):
        self.root = root
        self.device_path = "/dev/BeuatoCtrl0"
        root.title("Memory Map Viewer")
        
        # 初期ウィンドウサイズを調整（グラフ領域拡大に合わせて）
        root.geometry("1200x800")  # 幅を1200に拡大

        # デバイスをオープン (バイナリ、ノンバッファ)
        if DEBUG_MODE:
            print("デバイスをオープン中...")
        try:
            self.dev = open(self.device_path, "r+b", buffering=0)
            if DEBUG_MODE:
                print("デバイスオープン成功")
            DRIVER_DEBUG = 0 # 0: no debug 1: debug
            import fcntl
            fcntl.ioctl(self.dev, IOCTL_DEBUG, struct.pack("I",DRIVER_DEBUG)) # debug dmesg
            if DRIVER_DEBUG:
                print("driver debug log on.")
            else :
                print("driver debug log off.")

        except Exception as e:
            print(f"デバイスオープンエラー: {e}")
            self.dev = None

        # 選択状態管理
        self.selected_fields = set()  # 選択されたフィールド名のセット
        
        # 記録モード関連
        self.is_recording = False
        self.recorded_data = []  # 記録データ: [{'timestamp': datetime, 'field1': val1, 'field2': val2, ...}, ...]
        self.recording_start_time = None
        
        # 書き込み待ちキュー（メインループで処理）
        self.write_queue = []  # [(variable_name, value), ...]
        
        # プロット用の色リスト（複数グラフ用）
        self.colors = list(mcolors.TABLEAU_COLORS.values())

        # 左側：メモリマップのテーブル
        # カスタムスタイルで選択時の背景色を無効化
        style = ttk.Style()
        
        # 新しいカスタムスタイルを作成
        style.element_create("Custom.Treeheading.border", "from", "default")
        style.layout("Custom.Treeview", [
            ('Custom.Treeview.treearea', {'sticky': 'nswe'})
        ])
        
        # 選択時の色を完全に無効化
        style.configure("Custom.Treeview", 
                       selectbackground='',
                       selectforeground='black')
        style.map("Custom.Treeview",
                  background=[('selected', ''), ('active', '')],
                  foreground=[('selected', 'black'), ('active', 'black')])
        
        self.tree = ttk.Treeview(root, columns=('value',), show='tree headings', height=30, style="Custom.Treeview")
        self.tree.heading('#0', text='Field')
        self.tree.column('#0', width=200, anchor='w')
        self.tree.heading('value', text='Value')
        self.tree.column('value', width=120, anchor='e')
        self.tree.pack(side='left', fill='y', padx=5, pady=5)

        # フィールドを追加（クリックイベントをバインド）
        for name in memory_map:
            self.tree.insert('', 'end', iid=name, text=name, values=('',))
        
        # ツリーのクリックイベントをバインド
        self.tree.bind('<Button-1>', self._on_tree_click)

        # 右側全体のフレーム
        right_frame = ttk.Frame(root)
        right_frame.pack(side='right', fill='both', expand=True, padx=5, pady=5)
        
        # 右側上部：記録コントロールボタン
        control_frame = ttk.Frame(right_frame)
        control_frame.pack(side='top', fill='x', pady=(0, 5))
        
        # 記録開始/停止ボタン
        self.record_button = ttk.Button(control_frame, text="記録開始", command=self._toggle_recording)
        self.record_button.pack(side='left', padx=5)
        
        # 保存ボタン
        self.save_button = ttk.Button(control_frame, text="CSV保存", command=self._save_csv, state='disabled')
        self.save_button.pack(side='left', padx=5)
        
        # 記録状態表示ラベル
        self.status_label = ttk.Label(control_frame, text="記録停止中", foreground='red')
        self.status_label.pack(side='left', padx=10)
        
        # 記録データ数表示
        self.data_count_label = ttk.Label(control_frame, text="データ数: 0")
        self.data_count_label.pack(side='left', padx=10)

        # 右側下部：Matplotlib プロット領域
        if DEBUG_MODE:
            print("Matplotlib初期化開始...")
        try:
            self.fig, self.ax = plt.subplots(figsize=(12, 8))  # 高さを少し調整
            if DEBUG_MODE:
                print("matplotlib図作成完了")
            
            self.canvas = FigureCanvasTkAgg(self.fig, master=right_frame)
            if DEBUG_MODE:
                print("matplotlib canvas作成完了")
            
            self.canvas.get_tk_widget().pack(side='top', fill='both', expand=True)
            if DEBUG_MODE:
                print("matplotlib widget配置完了")
            
        except Exception as e:
            print(f"Matplotlib初期化エラー: {e}")
            import traceback
            traceback.print_exc()

        # 時系列データ保持用
        self.series = {name: deque(maxlen=WINDOW_SIZE) for name in memory_map}

        # 初期プロット設定
        self.ax.set_title("Select fields from left panel")
        self.ax.set_xlabel("Sample")
        self.ax.set_ylabel("Value")
        self.canvas.draw()

        # 定期更新開始
        root.after(0, self.update)

    def _on_tree_click(self, event):
        """ツリーアイテムクリック時の処理"""
        if DEBUG_MODE:
            print(f"=== ツリークリック イベント発生 ===")
        
        # クリック位置がどの領域かを判定
        region = self.tree.identify_region(event.x, event.y)
        item = self.tree.identify('item', event.x, event.y)
        
        if DEBUG_MODE:
            print(f"クリック位置: region={region}, item={item}")
        
        if item and item in memory_map:
            # クリック位置のX座標でカラムを判定
            # フィールド名列の幅は200px、値列の開始位置はそれ以降
            field_column_width = 200
            
            if event.x > field_column_width:  # Value列（数値表示領域）がクリックされた
                if DEBUG_MODE:
                    print("数値表示領域クリック")
                self._on_value_click(item)
            else:  # Field名列がクリックされた
                if DEBUG_MODE:
                    print("フィールド名領域クリック")
                self._on_field_click(item)
        
        # 標準選択を強制的にクリア
        self.root.after_idle(lambda: self.tree.selection_remove(self.tree.selection()))
        
        if DEBUG_MODE:
            print("=== ツリークリック 処理完了 ===\n")

    def _on_field_click(self, item):
        """フィールド名クリック時の処理（従来のトグル選択）"""
        if DEBUG_MODE:
            print(f"フィールド名クリック: {item}")
        
        # トグル処理
        if item in self.selected_fields:
            # 選択解除
            self.selected_fields.remove(item)
            if DEBUG_MODE:
                print(f"選択解除: {item}")
        else:
            # 選択追加
            self.selected_fields.add(item)
            if DEBUG_MODE:
                print(f"選択追加: {item}")
        
        if DEBUG_MODE:
            print(f"現在の選択フィールド: {self.selected_fields}")
        
        # 選択状態の視覚的表示を更新
        self._update_tree_colors()
        
        # 選択状態が変わったときに即座に再描画
        if DEBUG_MODE:
            print("プロット更新を実行...")
        self._update_plot()

    def _on_value_click(self, item):
        """数値表示領域クリック時の処理（書き込みモード）"""
        if DEBUG_MODE:
            print(f"数値領域クリック: {item}")
        
        if not self.dev:
            messagebox.showerror("エラー", "デバイスが接続されていません")
            return
        
        # 現在の値を取得
        current_value = self.tree.item(item)['values'][0]
        
        # 入力ダイアログを表示
        new_value = simpledialog.askstring(
            "値を書き込み",
            f"変数 '{item}' に書き込む値を入力してください:\n現在値: {current_value}",
            initialvalue=str(current_value)
        )
        
        if new_value is not None:  # キャンセルされていない場合
            try:
                # 値の妥当性をチェック
                converted_data = convert(str(new_value))
                addr, length, vartype = memory_map[item]
                
                # 型チェック
                struct.pack(TYPE_FMT[vartype], converted_data)
                
                # 書き込みキューに追加（メインループで処理される）
                self.write_queue.append((item, new_value))
                
                if DEBUG_MODE:
                    print(f"書き込みキューに追加: {item} = {new_value}")
                    
            except ValueError as e:
                messagebox.showerror("エラー", f"無効な値です: {str(e)}")
            except Exception as e:
                messagebox.showerror("エラー", f"値の検証中にエラーが発生しました: {str(e)}")

    def _process_write_command(self, variable_name, value):
        """
        書き込みコマンドを処理（メインループ内で呼び出される）
        """
        try:
            # 書き込みコマンドを生成
            command = make_write_command(variable_name, value)
            if DEBUG_MODE:
                print(f"書き込みコマンド実行: {variable_name} = {value}")
                print(f"コマンド: {command}")
            
            # デバイスに送信
            success = send_write_command(self.dev, command)
            
            if success:
                if DEBUG_MODE:
                    print(f"書き込み成功: {variable_name} = {value}")
                return True
            else:
                print(f"書き込み失敗: {variable_name} = {value}")
                return False
                    
        except Exception as e:
            print(f"書き込み処理エラー: {variable_name} = {value}, エラー: {str(e)}")
            import traceback
            traceback.print_exc()
            return False

    def _update_tree_colors(self):
        """ツリーの選択状態を視覚的に表示"""
        # 全てのアイテムに対して選択状態をチェック
        for item in self.tree.get_children():
            if item in self.selected_fields:
                # 選択されている場合は背景色を変更
                self.tree.item(item, tags=('selected',))
            else:
                # 選択されていない場合はタグをクリア（白背景に戻る）
                self.tree.item(item, tags=())
        
        # 選択された項目のスタイルを設定
        self.tree.tag_configure('selected', background='lightblue')
        
        # 強制的にツリーを更新
        self.tree.update_idletasks()
        
        if DEBUG_MODE:
            print(f"色更新完了: 選択中={self.selected_fields}")

    def _update_plot(self):
        """プロットを更新"""
        if DEBUG_MODE:
            print("=== _update_plot() 関数開始 ===")
            print(f"選択フィールド: {self.selected_fields}")
        
        # 最小限のテスト描画
        self.ax.clear()
        
        if not self.selected_fields:
            if DEBUG_MODE:
                print("選択フィールドなし")
            self.ax.set_title("Select fields from left panel")
            self.ax.set_xlabel("Sample")
            self.ax.set_ylabel("Value")
            self.canvas.draw()
            if DEBUG_MODE:
                print("初期状態描画完了")
            return
        
        if DEBUG_MODE:
            print("選択フィールドあり - グラフ描画開始")
        
        plot_count = 0
        colors = ['blue', 'red', 'green', 'orange', 'purple', 'brown', 'pink', 'gray']
        
        for i, field_name in enumerate(self.selected_fields):
            if DEBUG_MODE:
                print(f"処理中: {field_name}")
            if field_name in self.series:
                data_queue = self.series[field_name]
                if DEBUG_MODE:
                    print(f"  データ数: {len(data_queue)}")
                
                if len(data_queue) > 0:
                    y_data = list(data_queue)
                    x_data = list(range(len(y_data)))
                    color = colors[i % len(colors)]
                    
                    if DEBUG_MODE:
                        print(f"  プロット: {len(y_data)} ポイント, 色: {color}")
                        print(f"  Y範囲: {min(y_data):.6f} ~ {max(y_data):.6f}")
                    
                    self.ax.plot(x_data, y_data, color=color, label=field_name, 
                               linewidth=2, marker='o', markersize=3)
                    plot_count += 1
                else:
                    if DEBUG_MODE:
                        print(f"  データなし")
            else:
                if DEBUG_MODE:
                    print(f"  シリーズに存在しません")
        
        if plot_count > 0:
            self.ax.legend()
            # 時間幅を計算して表示
            time_span_ms = GRAPH_POINTS * UPDATE_INTERVAL
            if time_span_ms >= 1000:
                time_span_str = f"{time_span_ms/1000:.1f}s"
            else:
                time_span_str = f"{time_span_ms}ms"
            self.ax.set_title(f"Plotting {plot_count} fields ({time_span_str} span)")
            if DEBUG_MODE:
                print(f"グラフ作成完了: {plot_count} 系列")
        else:
            self.ax.set_title("Waiting for data...")
            if DEBUG_MODE:
                print("データ待ち状態")
        
        # X軸を時間表示に変更
        if plot_count > 0:
            # 最新の時刻を0として、過去にさかのぼる表示
            max_samples = max(len(self.series[field]) for field in self.selected_fields if field in self.series)
            if max_samples > 0:
                # X軸の目盛りを時間（秒）で表示
                x_ticks = []
                x_labels = []
                for i in range(0, max_samples, max(1, max_samples//5)):  # 5つの目盛り
                    time_ago_ms = (max_samples - i) * UPDATE_INTERVAL
                    if time_ago_ms >= 1000:
                        x_labels.append(f"-{time_ago_ms/1000:.1f}s")
                    else:
                        x_labels.append(f"-{time_ago_ms}ms")
                    x_ticks.append(i)
                self.ax.set_xticks(x_ticks)
                self.ax.set_xticklabels(x_labels)
        
        self.ax.set_xlabel("Time (ago)")
        self.ax.set_ylabel("Value")
        self.ax.grid(True, alpha=0.3)
        
        if DEBUG_MODE:
            print("canvas.draw() 実行")
        self.canvas.draw()
        if DEBUG_MODE:
            print("=== _update_plot() 関数終了 ===\n")

    def _toggle_recording(self):
        """記録の開始/停止を切り替え"""
        if not self.is_recording:
            # 記録開始
            self.is_recording = True
            self.recording_start_time = datetime.now()
            self.recorded_data.clear()
            self.record_button.config(text="記録停止")
            self.status_label.config(text="記録中", foreground='green')
            self.save_button.config(state='disabled')
            if DEBUG_MODE:
                print(f"記録開始: {self.recording_start_time}")
        else:
            # 記録停止
            self.is_recording = False
            self.record_button.config(text="記録開始")
            self.status_label.config(text="記録停止中", foreground='red')
            if self.recorded_data:
                self.save_button.config(state='normal')
            if DEBUG_MODE:
                print(f"記録停止: データ数 {len(self.recorded_data)}")

    def _save_csv(self):
        """CSVファイルに記録データを保存"""
        if not self.recorded_data:
            messagebox.showwarning("警告", "保存するデータがありません。")
            return
        
        # ファイルダイアログでファイル名を取得
        filename = filedialog.asksaveasfilename(
            defaultextension=".csv",
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")],
            title="記録データを保存"
        )
        
        if not filename:
            return
        
        try:
            # CSVファイルに保存
            with open(filename, 'w', newline='', encoding='utf-8') as csvfile:
                if self.recorded_data:
                    # ヘッダーを作成（タイムスタンプ + 全フィールド名）
                    fieldnames = ['timestamp'] + list(memory_map.keys())
                    writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                    
                    # ヘッダー行を書き込み
                    writer.writeheader()
                    
                    # データ行を書き込み
                    for record in self.recorded_data:
                        # タイムスタンプを文字列に変換
                        row = {'timestamp': record['timestamp'].strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}
                        # 各フィールドの値を追加
                        for field_name in memory_map.keys():
                            row[field_name] = record.get(field_name, '')
                        writer.writerow(row)
            
            messagebox.showinfo("保存完了", f"データを保存しました:\n{filename}\n記録数: {len(self.recorded_data)}")
            
        except Exception as e:
            messagebox.showerror("エラー", f"保存中にエラーが発生しました:\n{str(e)}")

    def update(self):
        """定期更新処理"""
        if DEBUG_MODE:
            print("=== update() 開始 ===")
        
        # 書き込みキューの処理（読み出し前に実行）
        if self.write_queue:
            variable_name, value = self.write_queue.pop(0)
            if DEBUG_MODE:
                print(f"書き込み処理実行: {variable_name} = {value}")
            
            success = self._process_write_command(variable_name, value)
            if not success:
                # 書き込み失敗をユーザーに通知（次回更新時に表示）
                self.root.after_idle(
                    lambda: messagebox.showerror("書き込みエラー", f"{variable_name} への書き込みに失敗しました")
                )
        
        try:
            # 全メモリを一括読み出し
            mem = read_all_memory(self.dev)
            if DEBUG_MODE:
                print(f"メモリ読み出し完了: {len(mem)} bytes")
        except Exception as e:
            print(f"メモリ読み出しエラー: {e}")
            self.root.after(UPDATE_INTERVAL, self.update)
            return
        
        # デコードしてテーブル更新
        data = {}
        numeric_count = 0
        for name, (addr, length, typ) in memory_map.items():
            raw = mem[addr:addr+length]
            #if typ == "c":
            #    val = raw.rstrip(b"\x00").decode("ascii", errors="ignore")
            #else:
            fmt = TYPE_FMT.get(typ)
            if fmt:
                val = struct.unpack_from(fmt, raw)[0]
                if isinstance(val, (int, float)):
                    numeric_count += 1
            else:
                val = "N/A"

            data[name] = val
            # テーブルセルを更新
            self.tree.set(name, 'value', val)

        if DEBUG_MODE:
            print(f"データデコード完了: {len(data)} フィールド, {numeric_count} 数値フィールド")

        # 記録モード中の場合、データを記録
        if self.is_recording:
            record = {
                'timestamp': datetime.now(),
                **data  # 全フィールドのデータを追加
            }
            self.recorded_data.append(record)
            # データ数表示を更新
            self.data_count_label.config(text=f"データ数: {len(self.recorded_data)}")

        # 全フィールドの時系列データを更新
        added_count = 0
        for field_name in memory_map.keys():
            if field_name in data:
                val = data[field_name]
                # 数値データのみプロット対象
                if isinstance(val, (int, float)):
                    self.series[field_name].append(val)
                    added_count += 1
                    if DEBUG_MODE and field_name in self.selected_fields:
                        print(f"選択フィールドにデータ追加: {field_name}={val}")

        if DEBUG_MODE:
            print(f"時系列データ追加完了: {added_count} フィールド")
            print(f"現在の選択フィールド: {self.selected_fields}")

        # プロット更新（選択されたフィールドがある場合のみ）
        if self.selected_fields:
            if DEBUG_MODE:
                print("*** プロット更新を実行開始 ***")
            try:
                if DEBUG_MODE:
                    print("_update_plot() 呼び出し直前")
                result = self._update_plot()
                if DEBUG_MODE:
                    print(f"_update_plot() 呼び出し完了, 戻り値: {result}")
            except Exception as e:
                print(f"プロット更新でエラー発生: {e}")
                print(f"エラータイプ: {type(e)}")
                import traceback
                traceback.print_exc()
        else:
            if DEBUG_MODE:
                print("選択フィールドなし、プロット更新スキップ")

        # 次回更新予約
        self.root.after(UPDATE_INTERVAL, self.update)
        if DEBUG_MODE:
            print("=== update() 終了 ===\n")

    def on_close(self):
        """終了時にデバイスをクローズ"""
        try:
            self.dev.close()
        except:
            pass
        self.root.quit()


def main():
    root = tk.Tk()
    app = MemoryViewerApp(root)
    root.protocol("WM_DELETE_WINDOW", app.on_close)
    root.mainloop()

if __name__ == "__main__":
    main()

