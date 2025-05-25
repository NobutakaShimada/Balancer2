#!/usr/bin/env python3
import tkinter as tk
from tkinter import ttk
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import struct
import time
from collections import deque

from allmemquery import read_all_memory
from BeuatoMemMap import memory_map

# 型コード → struct.unpack 用フォーマット
TYPE_FMT = {
    "us": "<H",   # unsigned short
    "uc": "<B",   # unsigned char
    "s":  "<h",   # short
    "d":  "<d",   # double
    "ll": "<q",   # long long
    "ull":"<Q",   # unsigned long long
}

# プロット用の時系列データ数
WINDOW_SIZE = 100
# 更新間隔（ms）
UPDATE_INTERVAL = 500

class MemoryViewerApp:
    def __init__(self, root):
        self.root = root
        self.device_path = "/dev/BeuatoCtrl0"
        root.title("Memory Map Viewer")

        # デバイスをオープン (バイナリ、ノンバッファ)
        self.dev = open(self.device_path, "r+b", buffering=0)

        # 左側：メモリマップのテーブル
        self.tree = ttk.Treeview(root, columns=('value',), show='headings', height=20)
        self.tree.heading('value', text='Value')
        self.tree.column('value', width=150)
        self.tree.pack(side='left', fill='y')

        for name in memory_map:
            self.tree.insert('', 'end', iid=name, values=('',))

        # 右上：フィールド選択 Combobox
        control_frame = ttk.Frame(root)
        control_frame.pack(side='top', fill='x', padx=5, pady=5)

        ttk.Label(control_frame, text="Plot field:").pack(side='left')
        self.selected_field = tk.StringVar()
        self.combo = ttk.Combobox(control_frame, textvariable=self.selected_field,
                                  values=list(memory_map.keys()), state='readonly')
        self.combo.pack(side='left', padx=5)
        self.combo.bind("<<ComboboxSelected>>", lambda e: self._on_field_change())

        # 右下：Matplotlib プロット領域
        self.fig, self.ax = plt.subplots()
        self.canvas = FigureCanvasTkAgg(self.fig, master=root)
        self.canvas.get_tk_widget().pack(side='right', fill='both', expand=True)

        # 時系列データ保持用
        self.series = {name: deque(maxlen=WINDOW_SIZE) for name in memory_map}

        # 初回フィールド選択
        self.current_field = None

        # 定期更新開始
        root.after(0, self.update)

    def _on_field_change(self):
        # Combobox で選ばれたフィールドに切り替え
        self.current_field = self.selected_field.get()
        # クリア
        self.series[self.current_field].clear()
        self.ax.clear()
        self.ax.set_title(self.current_field)
        self.canvas.draw()

    def update(self):
        # 全メモリを一括読み出し
        mem = read_all_memory(self.dev)
        # デコードしてテーブル更新
        data = {}
        for name, (addr, length, typ) in memory_map.items():
            raw = mem[addr:addr+length]
            if typ == "c":
                val = raw.rstrip(b"\x00").decode("ascii", errors="ignore")
            else:
                fmt = TYPE_FMT.get(typ)
                val = struct.unpack_from(fmt, raw)[0]
            data[name] = val
            # テーブルセルを更新
            self.tree.set(name, 'value', val)

        # 選択フィールドの時系列プロット更新
        if self.current_field:
            val = data[self.current_field]
            # 数値データのみプロット対象
            if isinstance(val, (int, float)):
                self.series[self.current_field].append(val)
                xs = list(range(len(self.series[self.current_field])))
                self.ax.clear()
                self.ax.plot(xs, self.series[self.current_field])
                self.ax.set_title(self.current_field)
                self.canvas.draw()

        # 次回更新予約
        self.root.after(UPDATE_INTERVAL, self.update)

    def on_close(self):
        # 終了時にデバイスをクローズ
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
