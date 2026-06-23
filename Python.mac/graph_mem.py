#!/usr/bin/env python3
import os
import tkinter as tk
from tkinter import ttk, filedialog, messagebox
import struct
import time
from collections import deque
import csv
from datetime import datetime
import ast

from allmemquery import read_all_memory
from BeuatoMemMap import memory_map
from BeuatoMemMap import TYPE_FMT
from beuato_transport import (
    BEUATO_MODE_BINARY,
    open_beuato_device,
    set_beuato_debug,
    set_beuato_read_mode,
)

# 更新間隔（ms）
UPDATE_INTERVAL = 10
# グラフ表示点数（この点数を常に表示）
GRAPH_POINTS = 100
# プロット用の時系列データ数（更新間隔に応じて自動計算）
WINDOW_SIZE = GRAPH_POINTS  # 表示点数と同じに設定
# デバッグモード
DEBUG_MODE = False  # True にするとデバッグ情報を出力

def print_debug(*args, **kwargs):
    """デバッグモードの時のみ出力する関数"""
    if DEBUG_MODE:
        print(*args, **kwargs)


class TkRealtimePlot:
    def __init__(self, master):
        self.canvas = tk.Canvas(master, background="white", highlightthickness=1, highlightbackground="#c0c0c0")
        self.canvas.bind("<Configure>", lambda _event: self._redraw())
        self.title = "Select fields from left panel"
        self.series = {}
        self.colors = {}
        self._redraw()

    def get_tk_widget(self):
        return self.canvas

    def draw_empty(self, title):
        self.title = title
        self.series = {}
        self.colors = {}
        self._redraw()

    def draw_series(self, title, series, colors):
        self.title = title
        self.series = {name: list(values) for name, values in series.items() if values}
        self.colors = colors
        self._redraw()

    def _redraw(self):
        c = self.canvas
        c.delete("all")
        width = max(c.winfo_width(), 400)
        height = max(c.winfo_height(), 300)
        left, top, right, bottom = 58, 34, 18, 44
        plot_w = max(width - left - right, 1)
        plot_h = max(height - top - bottom, 1)

        c.create_text(width // 2, 16, text=self.title, fill="#222")
        c.create_rectangle(left, top, left + plot_w, top + plot_h, outline="#808080")

        all_values = [v for values in self.series.values() for v in values if isinstance(v, (int, float))]
        if not all_values:
            c.create_text(left + plot_w // 2, top + plot_h // 2, text="Waiting for data...", fill="#606060")
            return

        ymin = min(all_values)
        ymax = max(all_values)
        if ymin == ymax:
            ymin -= 1
            ymax += 1

        for i in range(6):
            y = top + plot_h * i / 5
            value = ymax - (ymax - ymin) * i / 5
            c.create_line(left, y, left + plot_w, y, fill="#e8e8e8")
            c.create_text(left - 6, y, text=f"{value:.3g}", anchor="e", fill="#555", font=("", 10))

        for index, (name, values) in enumerate(self.series.items()):
            if len(values) < 2:
                continue
            color = self.colors.get(name, "blue")
            points = []
            span = max(len(values) - 1, 1)
            for i, value in enumerate(values):
                x = left + plot_w * i / span
                y = top + plot_h - (float(value) - ymin) * plot_h / (ymax - ymin)
                points.extend((x, y))
            c.create_line(*points, fill=color, width=2)
            legend_y = top + 16 * index
            c.create_line(left + plot_w - 150, legend_y, left + plot_w - 128, legend_y, fill=color, width=3)
            c.create_text(left + plot_w - 122, legend_y, text=name, anchor="w", fill="#222", font=("", 10))

        c.create_text(left + plot_w // 2, height - 16, text="Time (ago)", fill="#444")
        c.create_text(16, top + plot_h // 2, text="Value", fill="#444", angle=90)


# driver debug log
IOCTL_DEBUG = 0x40044200
IOCTL_READ_MODE = 0x40044201

# fontsize
DEFAULT_FONTSIZE = 12
DEFAULT_COLORS = ['blue', 'red', 'green', 'orange', 'purple', 'brown', 'pink', 'gray']


# automatic string to numerical conversion (from interactive.py)
convert = lambda s: ast.literal_eval(s) if s.replace('.','').replace('-','').replace('e','').replace('E','').replace('+','').isdigit() else s

# 仕様書に基づく読み取り専用変数のリスト（「R」のみの変数）
READONLY_VARIABLES = {
    "Product_ID",           # プロダクトID
    "Version",              # ファームウェアバージョン  
    "MODE",                 # モード
    "M_CURRENT_L",          # モータ電流測定値
    "M_CURRENT_R", 
    "T_CURRENT_L",          # モータ電流指令値
    "T_CURRENT_R",
    "CURRENT_OFFSET_L",     # 電流オフセット
    "CURRENT_OFFSET_R",
    "BODY_ANGULAR_SPD",     # 本体角速度
    "BODY_ANGLE",           # 本体角度
    "BODY_ANGULAR_SPD_OFFSET", # 本体角速度オフセット
    "WHEEL_ANGULAR_SPD_L",  # ホイール角速度
    "WHEEL_ANGULAR_SPD_R",
    "WHEEL_ANGLE_L",        # ホイール角度
    "WHEEL_ANGLE_R",
    "ENC_L",                # エンコーダ
    "ENC_R",
    "GYRO_DATA",            # ジャイロ
    "ADC_C_LA",             # ADC（電流）
    "ADC_C_LB",
    "ADC_C_RA",
    "ADC_C_RB",
    "PAD_BTN",              # VS-C3
    "PAD_AN_RX",
    "PAD_AN_RY", 
    "PAD_AN_LX",
    "PAD_AN_LY",
    # 実装上書き込み不可
    "GAIN_OPTION6"          # ファームウェア未実装
}

def make_write_command(name, data) -> bytes:
    """
    変数名と書き込みデータから書き込みコマンドを生成
    allmemquery.pyと同じASCII形式
    """
    try:
        addr, length, vartype = memory_map[name]
    except KeyError:
        raise ValueError(f"Unknown variable: {name!r}")
    
    print_debug(f"Debug: {name} - addr={addr} (0x{addr:x}), length={length}, vartype={vartype}")
    
    # データをバイナリ形式に変換
    try:
        converted_data = convert(str(data))
        bdata = struct.pack(TYPE_FMT[vartype], converted_data)
        print_debug(f"Debug: converted_data={converted_data}, bdata={bdata.hex()}")
    except (ValueError, struct.error) as e:
        raise ValueError(f"Invalid data format for {vartype}: {data}")
    
    # ASCII形式でコマンド生成（allmemquery.pyと同じパターン）
    # "w {addr} {length} {data_hex} "
    addr_hex = format(addr, 'x')
    length_hex = format(length, 'x')
    data_hex = ' '.join(f'{b:02x}' for b in bdata)
    cmd = f"w {addr_hex} {length_hex} {data_hex} "
    
    print_debug(f"Debug: ASCII command = '{cmd}'")
    return cmd.encode('ascii')

def send_write_command(dev, command):
    """
    書き込みコマンドを送信
    allmemquery.pyと同じパターンでバイナリ応答を期待
    """
    try:
        print_debug(f"Debug: Sending command: {command}")
        dev.write(command)
        
        # バイナリ形式で応答を読み取り
        response = dev.read(256)
        print_debug(f"Debug: Write response raw: {response}")
        print_debug(f"Debug: Write response hex: {response.hex() if response else 'None'}")
        
        # バイナリ応答の解析（ドライバのbuild_line_binary形式）
        if response and len(response) >= 3:
            if response[0:1] == b'w':  # 書き込み応答
                # 応答フォーマット: 'w' + addr_low + length + data...
                addr = response[1]
                response_len = response[2] if len(response) > 2 else 0
                print_debug(f"Debug: Write response - command='w', addr=0x{addr:x}({addr}), length={response_len}")
                
                if len(response) >= 3 + response_len:
                    data_part = response[3:3+response_len] if response_len > 0 else b''
                    print_debug(f"Debug: Response data: {data_part.hex() if data_part else 'none'}")
                    print_debug("Debug: 書き込み成功と判定")
                    return True
                else:
                    print_debug(f"Debug: データ部分不足 - 期待{response_len}, 実際{len(response)-3}")
                    # データが不足でも、レスポンス自体があれば成功とみなす
                    if response_len == 0:
                        print_debug("Debug: データ長0だが応答ありのため成功と判定")
                        return True
                    return False
            else:
                print_debug(f"Debug: Unexpected response command: {response[0]:02x}")
                return False
        else:
            print_debug("Debug: No response or too short")
            return False
            
    except Exception as e:
        print(f"Write command failed: {e}")
        import traceback
        traceback.print_exc()
        return False


class MemoryViewerApp:
    def __init__(self, root):
        # 最初に全ての必要な属性を初期化
        self.editing_item = None
        self.edit_entry = None
        self.original_value = None
        self.editing_start_time = 0
        self.selected_fields = set()
        self.is_recording = False
        self.recorded_data = []
        self.recording_start_time = None
        self.write_queue = []
        self.colors = list(DEFAULT_COLORS)
        self.last_connect_attempt = 0
        self.default_values = {}
        self.current_values = {}
        self.default_values_captured = False
        
        self.root = root
        self.device_path = "/dev/BeuatoCtrl0"
        root.title("Memory Map Viewer")
        root.geometry("1200x800")

        self.dev = None
        self._connect_device(force=True)

        # 左側：メモリマップのテーブル
        style = ttk.Style()
        style.element_create("Custom.Treeheading.border", "from", "default")
        style.layout("Custom.Treeview", [
            ('Custom.Treeview.treearea', {'sticky': 'nswe'})
        ])
        style.configure("Custom.Treeview", 
                       selectbackground='',
                       selectforeground='black',
                       font=('', DEFAULT_FONTSIZE, 'normal'))
        style.map("Custom.Treeview",
                  background=[('selected', ''), ('active', '')],
                  foreground=[('selected', 'black'), ('active', 'black')])
        
        # メインのPanedWindow（左右分割）
        main_paned = ttk.PanedWindow(root, orient='horizontal')
        main_paned.pack(fill='both', expand=True, padx=5, pady=5)
        root.after(100, lambda: main_paned.sashpos(0,350))

        # 左ペイン（変数リスト用フレーム）
        left_frame = ttk.Frame(main_paned)
        main_paned.add(left_frame, weight=1)

        # 右ペイン（グラフ用フレーム）
        right_frame = ttk.Frame(main_paned)
        main_paned.add(right_frame, weight=40)

        left_control_frame = ttk.Frame(left_frame)
        left_control_frame.pack(side='top', fill='x', pady=(0, 4))

        self.reset_all_button = tk.Button(
            left_control_frame,
            text="全てリセット",
            command=self._reset_all_to_defaults,
            state='disabled',
            relief='raised',
            borderwidth=2,
            padx=8,
            pady=2,
        )
        self.reset_all_button.pack(side='left', padx=2)

        self.set_default_button = tk.Button(
            left_control_frame,
            text="現在値をデフォルトに設定",
            command=self._set_current_values_as_defaults,
            state='disabled',
            relief='raised',
            borderwidth=2,
            padx=8,
            pady=2,
        )
        self.set_default_button.pack(side='left', padx=2)

        #self.tree = ttk.Treeview(root, columns=('value',), show='tree headings', height=30, style="Custom.Treeview")
        self.tree = ttk.Treeview(left_frame, columns=('value', 'reset'), show='tree headings', style="Custom.Treeview")
        self.tree.heading('#0', text='Field')
        self.tree.column('#0', width=200, anchor='w')
        self.tree.heading('value', text='Value')
        self.tree.column('value', width=100, anchor='e')
        self.tree.heading('reset', text='')
        self.tree.column('reset', width=55, anchor='center', stretch=False)
        #self.tree.pack(side='left', fill='both', padx=2, pady=5)

        # フィールドを追加
        for name in memory_map:
            self.tree.insert('', 'end', iid=name, text=name, values=('', ''))

        # ツリーを左フレームに配置
        self.tree.pack(in_=left_frame, fill='both', expand=True)

        self.reset_buttons = {}
        for name in memory_map:
            if self._is_writable_variable(name):
                button = tk.Button(
                    self.tree,
                    text="Reset",
                    relief="raised",
                    borderwidth=2,
                    padx=4,
                    pady=0,
                    command=lambda item=name: self._reset_item_to_default(item),
                )
                button.place_forget()
                self.reset_buttons[name] = button

        # イベントバインド
        self.tree.bind('<Button-1>', self._on_tree_click)
        self.tree.bind('<Configure>', lambda e: self._position_reset_buttons())
        self.tree.bind('<Expose>', lambda e: self._position_reset_buttons())
        self.tree.bind('<MouseWheel>', lambda e: self.root.after_idle(self._position_reset_buttons))
        self.tree.bind('<Button-4>', lambda e: self.root.after_idle(self._position_reset_buttons))
        self.tree.bind('<Button-5>', lambda e: self.root.after_idle(self._position_reset_buttons))





        # コントロールボタン
        control_frame = ttk.Frame(right_frame)
        control_frame.pack(side='top', fill='x', pady=(0, 5))
        
        self.record_button = ttk.Button(control_frame, text="記録開始", command=self._toggle_recording)
        self.record_button.pack(side='left', padx=5)
        
        self.save_button = ttk.Button(control_frame, text="CSV保存", command=self._save_csv, state='disabled')
        self.save_button.pack(side='left', padx=5)
        
        self.status_label = ttk.Label(control_frame, text="記録停止中", foreground='red')
        self.status_label.pack(side='left', padx=10)
        
        self.data_count_label = ttk.Label(control_frame, text="データ数: 0")
        self.data_count_label.pack(side='left', padx=10)

        self.using_matplotlib = False
        try:
            if tk.TkVersion >= 9.0:
                raise RuntimeError("matplotlib TkAgg is not compatible with this Tk 9 runtime")
            else:
                os.environ.setdefault("MPLCONFIGDIR", os.path.join(os.path.dirname(__file__), ".matplotlib-cache"))
                import matplotlib.pyplot as plt
                from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

                self.fig, self.ax = plt.subplots(figsize=(12, 8))
                self.canvas = FigureCanvasTkAgg(self.fig, master=right_frame)
                self.canvas.get_tk_widget().pack(side='top', fill='both', expand=True)
                self.using_matplotlib = True
        except Exception as e:
            print(f"Matplotlib TkAggを使用しません: {e}")
            print("Tk Canvas fallbackを使用します。")
            self.plot_canvas = TkRealtimePlot(right_frame)
            self.plot_canvas.get_tk_widget().pack(side='top', fill='both', expand=True)

        # 時系列データ保持用
        self.series = {name: deque(maxlen=WINDOW_SIZE) for name in memory_map}

        # 初期プロット設定
        if self.using_matplotlib:
            self.ax.set_title("Select fields from left panel")
            self.ax.set_xlabel("Sample")
            self.ax.set_ylabel("Value")
            self.canvas.draw()
        else:
            self.plot_canvas.draw_empty("Select fields from left panel")

        # 初期表示でRead Only変数を視覚化
        self._update_tree_colors()

        # 定期更新開始
        root.after(0, self.update)

    def _connect_device(self, force=False):
        now = time.monotonic()
        if self.dev is not None:
            return True
        if not force and now - self.last_connect_attempt < 1.0:
            return False

        self.last_connect_attempt = now
        try:
            self.dev = open_beuato_device(self.device_path, read_mode=BEUATO_MODE_BINARY)
            set_beuato_debug(self.dev, 0)
            set_beuato_read_mode(self.dev, BEUATO_MODE_BINARY)
            print("Driver set to BINARY read mode.")
            if hasattr(self, "status_label") and not self.is_recording:
                self.status_label.config(text="接続済み", foreground="green")
            return True
        except Exception as e:
            print(f"デバイスオープンエラー: {e}")
            self.dev = None
            if hasattr(self, "status_label") and not self.is_recording:
                self.status_label.config(text="未接続", foreground="red")
            return False

    def _disconnect_device(self):
        if self.dev is None:
            return
        try:
            self.dev.close()
        except Exception:
            pass
        self.dev = None

    def _on_tree_click(self, event):
        """ツリーアイテムクリック時の処理"""
        print_debug("=== ツリークリック イベント発生 ===")
        
        # 編集中の場合、編集を終了してから新しいクリックを処理
        if self.editing_item:
            # 編集開始から十分時間が経過している場合は終了
            if hasattr(self, 'editing_start_time') and time.time() - self.editing_start_time > 0.2:
                print_debug("既存の編集を終了（時間経過）")
                self._end_edit(commit=False)
            else:
                print_debug("編集開始直後のため、クリックを無視")
                return
        
        region = self.tree.identify_region(event.x, event.y)
        item = self.tree.identify('item', event.x, event.y)
        
        print_debug(f"クリック位置: region={region}, item={item}, x={event.x}, y={event.y}")
        
        if item and item in memory_map:
            column = self.tree.identify_column(event.x)
            print_debug(f"クリック列: {column}")
            
            if column == '#2':
                print_debug("リセット列クリック")
                self._reset_item_to_default(item)
            elif column == '#1':
                print_debug("数値表示領域クリック")
                self._on_value_click(item)
            else:
                print_debug("フィールド名領域クリック")
                self._on_field_click(item)
        
        self.root.after_idle(lambda: self.tree.selection_remove(self.tree.selection()))
        print_debug("=== ツリークリック 処理完了 ===\n")

    def _position_reset_buttons(self):
        """Reset列に各行の実ボタンを重ねて配置する"""
        if not hasattr(self, "reset_buttons"):
            return

        for item, button in self.reset_buttons.items():
            bbox = self.tree.bbox(item, 'reset')
            if not bbox:
                button.place_forget()
                continue

            x, y, width, height = bbox
            button.place(
                in_=self.tree,
                x=x + 3,
                y=y + 1,
                width=max(1, width - 6),
                height=max(1, height - 2),
            )

    def _on_field_click(self, item):
        """フィールド名クリック時の処理（グラフ選択）"""
        if item in self.selected_fields:
            self.selected_fields.remove(item)
        else:
            self.selected_fields.add(item)
        
        self._update_tree_colors()
        self._update_plot()

    def _on_value_click(self, item):
        """数値表示領域クリック時の処理（インライン編集）"""
        print_debug(f"=== _on_value_click 開始: {item} ===")
        
        if not self.dev:
            messagebox.showerror("エラー", "デバイスが接続されていません")
            return
            
        # 読み取り専用変数のチェック
        if item in READONLY_VARIABLES:
            print_debug(f"{item} は読み取り専用です")
            messagebox.showinfo("読み取り専用", f"{item} は読み取り専用のため編集できません。\n\n仕様書で「R」(Read Only)に指定されている変数です。")
            return
        
        print_debug(f"{item} は書き込み可能です")
        
        if self.editing_item:
            print_debug(f"既存の編集を終了: {self.editing_item}")
            self._end_edit(commit=False)
        
        print_debug(f"編集開始: {item}")
        self._start_edit(item)

    def _start_edit(self, item):
        """インライン編集開始"""
        print_debug(f"=== _start_edit 開始: {item} ===")
        
        self.editing_item = item
        self.editing_start_time = time.time()
        
        current_value = self.tree.item(item)['values'][0]
        self.original_value = str(current_value)
        print_debug(f"現在の値: {current_value}")
        
        # 基本的なEntryウィジェット作成
        try:
            self.edit_entry = tk.Entry(self.tree, relief='solid', borderwidth=2)
            print_debug("基本Entryウィジェット作成完了")
        except Exception as e:
            print_debug(f"Entry作成エラー: {e}")
            return
        
        self.edit_entry.insert(0, self.original_value)
        self.edit_entry.select_range(0, tk.END)
        
        bbox = self.tree.bbox(item, 'value')
        print_debug(f"bbox: {bbox}")
        
        if bbox:
            x, y, width, height = bbox
            print_debug(f"配置位置: x={x}, y={y}, width={width}, height={height}")
            
            self.edit_entry.place(in_=self.tree, x=x, y=y, width=width, height=height)
            
            # イベントバインド
            self.edit_entry.bind('<Return>', lambda e: self._end_edit(commit=True))
            self.edit_entry.bind('<Escape>', lambda e: self._end_edit(commit=False))
            self.edit_entry.bind('<Button-3>', lambda e: self._end_edit(commit=False))
            
            # フォーカス設定を遅延実行
            self.root.after(10, self._delayed_focus)
            print_debug(f"Entryウィジェット配置完了")
        else:
            print_debug("bbox取得失敗")
            self._end_edit(commit=False)

    def _delayed_focus(self):
        """遅延フォーカス設定"""
        if self.edit_entry:
            self.edit_entry.focus_set()
            self.edit_entry.select_range(0, tk.END)
            print_debug("遅延フォーカス設定完了 - 編集可能状態")

    def _end_edit(self, commit=False):
        """インライン編集終了"""
        if not self.editing_item or not self.edit_entry:
            return
            
        print_debug(f"編集終了: {self.editing_item}, commit={commit}")
        
        if commit:
            new_value = self.edit_entry.get().strip()
            if new_value != self.original_value:
                try:
                    converted_data = convert(new_value)
                    addr, length, vartype = memory_map[self.editing_item]
                    struct.pack(TYPE_FMT[vartype], converted_data)
                    
                    self.write_queue.append((self.editing_item, new_value))
                    print_debug(f"書き込みキューに追加: {self.editing_item} = {new_value}")
                        
                except (ValueError, struct.error) as e:
                    print(f"値の検証エラー: {e}")
                    messagebox.showerror("入力エラー", f"無効な値です: {str(e)}")
                except Exception as e:
                    print(f"予期しないエラー: {e}")
                    messagebox.showerror("エラー", f"値の検証中にエラーが発生しました: {str(e)}")
        
        if self.edit_entry:
            self.edit_entry.destroy()
            self.edit_entry = None
        
        self.editing_item = None
        self.original_value = None

    def _process_write_command(self, variable_name, value):
        """書き込みコマンドを処理"""
        try:
            command = make_write_command(variable_name, value)
            success = send_write_command(self.dev, command)
            
            if success:
                print_debug(f"書き込み成功: {variable_name} = {value}")
                return True
            else:
                print_debug(f"書き込み失敗: {variable_name} = {value}")
                return False
                    
        except Exception as e:
            print(f"書き込み処理エラー: {variable_name} = {value}, エラー: {str(e)}")
            return False

    def _is_writable_variable(self, variable_name):
        return variable_name in memory_map and variable_name not in READONLY_VARIABLES

    def _capture_default_values(self, data):
        """最初に読めたメモリマップ値をリセット用デフォルトとして保存"""
        if self.default_values_captured:
            return

        self.default_values = dict(data)
        self.default_values_captured = True
        self.reset_all_button.config(state='normal')
        self.set_default_button.config(state='normal')
        print("起動時デフォルト値を記録しました。")

    def _set_current_values_as_defaults(self):
        """現在表示されている値を以後のリセット先として保存"""
        if not self.current_values:
            messagebox.showinfo("未準備", "まだ現在値を読み込めていません。")
            return

        self.default_values = dict(self.current_values)
        self.default_values_captured = True
        self.reset_all_button.config(state='normal')
        self.set_default_button.config(state='normal')
        print("現在値をリセット用デフォルトとして記録しました。")

    def _queue_default_write(self, variable_name):
        if not self.default_values_captured:
            messagebox.showinfo("未準備", "まだ起動時デフォルト値を読み込めていません。")
            return False
        if not self._is_writable_variable(variable_name):
            return False
        if variable_name not in self.default_values:
            return False

        self.write_queue.append((variable_name, repr(self.default_values[variable_name])))
        return True

    def _reset_item_to_default(self, variable_name):
        """指定項目を起動時デフォルト値へ戻す"""
        if not self._is_writable_variable(variable_name):
            return
        if self._queue_default_write(variable_name):
            print_debug(f"リセットキューに追加: {variable_name} -> {self.default_values[variable_name]!r}")

    def _reset_all_to_defaults(self):
        """書き込み可能な全項目を起動時デフォルト値へ戻す"""
        if self.editing_item:
            self._end_edit(commit=False)

        queued = 0
        for variable_name in memory_map:
            if self._queue_default_write(variable_name):
                queued += 1

        if queued == 0:
            messagebox.showinfo("リセット", "リセット対象がありません。")
        else:
            print(f"{queued}項目を起動時デフォルト値へ戻します。")

    def _update_tree_colors(self):
        """ツリーの選択状態と読み取り専用状態を視覚的に表示"""
        print_debug(f"Debug: _update_tree_colors called, readonly_variables count={len(READONLY_VARIABLES)}")
        
        for item in self.tree.get_children():
            if item in self.selected_fields:
                # 選択されている場合（グラフ表示対象）
                self.tree.item(item, tags=('selected',))
                print_debug(f"Debug: {item} -> selected")
            elif item in READONLY_VARIABLES:
                # 読み取り専用の場合
                self.tree.item(item, tags=('readonly',))
                print_debug(f"Debug: {item} -> readonly")
            else:
                # 書き込み可能な場合
                self.tree.item(item, tags=('writable',))
                print_debug(f"Debug: {item} -> writable")
        
        # 各タグのスタイルを設定
        self.tree.tag_configure('selected', background='pink', font=('',DEFAULT_FONTSIZE,'bold')) # pink with bold
        self.tree.tag_configure('readonly', background='lightblue', foreground='black')  # 薄いグレー背景、グレー文字
        self.tree.tag_configure('writable', background='white', foreground='black')       # 通常表示
        
        self.tree.update_idletasks()
        print_debug("Debug: Tree colors updated")

    def _update_plot(self):
        """プロットを更新"""
        if not self.using_matplotlib:
            self._update_tk_plot()
            return

        self.ax.clear()
        
        if not self.selected_fields:
            self.ax.set_title("Select fields from left panel")
            self.ax.set_xlabel("Sample")
            self.ax.set_ylabel("Value")
            self.canvas.draw()
            return
        
        plot_count = 0
        colors = DEFAULT_COLORS
        
        for i, field_name in enumerate(self.selected_fields):
            if field_name in self.series:
                data_queue = self.series[field_name]
                if len(data_queue) > 0:
                    y_data = list(data_queue)
                    x_data = list(range(len(y_data)))
                    color = colors[i % len(colors)]
                    
                    self.ax.plot(x_data, y_data, color=color, label=field_name, 
                               linewidth=2, marker='o', markersize=3)
                    plot_count += 1
        
        if plot_count > 0:
            self.ax.legend()
            time_span_ms = GRAPH_POINTS * UPDATE_INTERVAL
            if time_span_ms >= 1000:
                time_span_str = f"{time_span_ms/1000:.1f}s"
            else:
                time_span_str = f"{time_span_ms}ms"
            self.ax.set_title(f"Plotting {plot_count} fields ({time_span_str} span)")
            
            # X軸を時間表示に変更
            max_samples = max(len(self.series[field]) for field in self.selected_fields if field in self.series)
            if max_samples > 0:
                x_ticks = []
                x_labels = []
                for i in range(0, max_samples, max(1, max_samples//5)):
                    time_ago_ms = (max_samples - i) * UPDATE_INTERVAL
                    if time_ago_ms >= 1000:
                        x_labels.append(f"-{time_ago_ms/1000:.1f}s")
                    else:
                        x_labels.append(f"-{time_ago_ms}ms")
                    x_ticks.append(i)
                self.ax.set_xticks(x_ticks)
                self.ax.set_xticklabels(x_labels)
        else:
            self.ax.set_title("Waiting for data...")
        
        self.ax.set_xlabel("Time (ago)")
        self.ax.set_ylabel("Value")
        self.ax.grid(True, alpha=0.3)
        self.canvas.draw()

    def _update_tk_plot(self):
        if not self.selected_fields:
            self.plot_canvas.draw_empty("Select fields from left panel")
            return

        colors = DEFAULT_COLORS
        plot_series = {}
        color_map = {}
        for i, field_name in enumerate(self.selected_fields):
            data_queue = self.series.get(field_name)
            if data_queue:
                values = list(data_queue)
                if values:
                    plot_series[field_name] = values
                    color_map[field_name] = colors[i % len(colors)]

        if not plot_series:
            self.plot_canvas.draw_empty("Waiting for data...")
            return

        time_span_ms = GRAPH_POINTS * UPDATE_INTERVAL
        if time_span_ms >= 1000:
            time_span_str = f"{time_span_ms/1000:.1f}s"
        else:
            time_span_str = f"{time_span_ms}ms"
        self.plot_canvas.draw_series(
            f"Plotting {len(plot_series)} fields ({time_span_str} span)",
            plot_series,
            color_map,
        )

    def _toggle_recording(self):
        """記録の開始/停止を切り替え"""
        if not self.is_recording:
            self.is_recording = True
            self.recording_start_time = datetime.now()
            self.recorded_data.clear()
            self.record_button.config(text="記録停止")
            self.status_label.config(text="記録中", foreground='green')
            self.save_button.config(state='disabled')
        else:
            self.is_recording = False
            self.record_button.config(text="記録開始")
            self.status_label.config(text="記録停止中", foreground='red')
            if self.recorded_data:
                self.save_button.config(state='normal')

    def _save_csv(self):
        """CSVファイルに記録データを保存"""
        if not self.recorded_data:
            messagebox.showwarning("警告", "保存するデータがありません。")
            return
        
        filename = filedialog.asksaveasfilename(
            defaultextension=".csv",
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")],
            title="記録データを保存"
        )
        
        if not filename:
            return
        
        try:
            with open(filename, 'w', newline='', encoding='utf-8') as csvfile:
                if self.recorded_data:
                    fieldnames = ['timestamp'] + list(memory_map.keys())
                    writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                    writer.writeheader()
                    
                    for record in self.recorded_data:
                        row = {'timestamp': record['timestamp'].strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}
                        for field_name in memory_map.keys():
                            row[field_name] = record.get(field_name, '')
                        writer.writerow(row)
            
            messagebox.showinfo("保存完了", f"データを保存しました:\n{filename}\n記録数: {len(self.recorded_data)}")
            
        except Exception as e:
            messagebox.showerror("エラー", f"保存中にエラーが発生しました:\n{str(e)}")

    def update(self):
        """定期更新処理"""
        if not self._connect_device():
            self.root.after(UPDATE_INTERVAL, self.update)
            return

        # 書き込みキューの処理
        if self.write_queue:
            variable_name, value = self.write_queue.pop(0)
            success = self._process_write_command(variable_name, value)
            if not success:
                self.root.after_idle(
                    lambda: messagebox.showerror("書き込みエラー", f"{variable_name} への書き込みに失敗しました")
                )
        
        try:
            mem = read_all_memory(self.dev)
        except Exception as e:
            print(f"メモリ読み出しエラー: {e}")
            self._disconnect_device()
            if not self.is_recording:
                self.status_label.config(text="再接続待ち", foreground="red")
            self.root.after(UPDATE_INTERVAL, self.update)
            return
        
        # デコードしてテーブル更新
        data = {}
        for name, (addr, length, typ) in memory_map.items():
            raw = mem[addr:addr+length]
            fmt = TYPE_FMT.get(typ)
            if fmt:
                val = struct.unpack_from(fmt, raw)[0]
            else:
                val = "N/A"

            data[name] = val
            self.tree.set(name, 'value', val)

        self.current_values = dict(data)
        self._capture_default_values(data)

        # 記録モード
        if self.is_recording:
            record = {
                'timestamp': datetime.now(),
                **data
            }
            self.recorded_data.append(record)
            self.data_count_label.config(text=f"データ数: {len(self.recorded_data)}")

        # 時系列データ更新
        for field_name in memory_map.keys():
            if field_name in data:
                val = data[field_name]
                if isinstance(val, (int, float)):
                    self.series[field_name].append(val)

        # プロット更新
        if self.selected_fields:
            try:
                self._update_plot()
            except Exception as e:
                print(f"プロット更新エラー: {e}")

        self._position_reset_buttons()
        self.root.after(UPDATE_INTERVAL, self.update)

    def on_close(self):
        self.root.after_cancel(self.root.tk.call('after', 'info'))
        self.root.after(50, self._delayed_close)

    def _delayed_close(self):
        """終了時にデバイスをクローズ"""
        self._disconnect_device()
        self.root.quit()


def main():
    root = tk.Tk()
    app = MemoryViewerApp(root)
    root.protocol("WM_DELETE_WINDOW", app.on_close)
    root.mainloop()

if __name__ == "__main__":
    main()
