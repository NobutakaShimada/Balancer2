#!/usr/bin/env python3
import os, sys
sys.path.append(os.path.dirname(__file__))  # allmemquery モジュールを同ディレクトリから読み込む
import tkinter as tk
from tkinter import ttk
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import struct
import time
from collections import deque

from allmemquery import read_all_memory
from BeuatoMemMap import memory_map, TYPE_FMT


# ウィンドウサイズと更新間隔（ミリ秒）
WINDOW_SIZE     = 100
UPDATE_INTERVAL = 10


class MemoryViewerApp:
    def __init__(self, root):
        self.root = root
        root.title("Memory Map Viewer")

        # デバイスをオープン (バイナリ、ノンバッファ)
        self.dev = open("/dev/BeuatoCtrl0", "r+b", buffering=0)

        # スタイル調整：フォントを小さめに
        style = ttk.Style()
        style.configure("Treeview", font=('TkDefaultFont', 8))
        style.configure("Treeview.Heading", font=('TkDefaultFont', 9, 'bold'))

        # 左: メモリマップ表示テーブル
        # #0 列 (=ツリー列) に Field 名、'value' 列に値を表示
        self.tree = ttk.Treeview(
            root,
            columns=('value',),
            show='tree headings',
            height=20
        )
        self.tree.heading('#0', text='Field')
        self.tree.column('#0', width=150, anchor='w')
        self.tree.heading('value', text='Value')
        self.tree.column('value', width=100, anchor='e')
        self.tree.pack(side='left', fill='y', padx=(5,2), pady=5)

        # 行を追加 (Field 名を text に渡す)
        for name in memory_map.keys():
            self.tree.insert('', 'end', iid=name, text=name, values=(''))

        # 右上: フィールド選択 Combobox
        control = ttk.Frame(root)
        control.pack(side='top', fill='x', padx=5, pady=(5,0))
        ttk.Label(control, text="Plot field:").pack(side='left')
        self.selected_field = tk.StringVar()
        self.combo = ttk.Combobox(
            control,
            textvariable=self.selected_field,
            values=list(memory_map.keys()),
            state='readonly',
            font=('TkDefaultFont', 8)
        )
        self.combo.pack(side='left', padx=5)
        self.combo.bind("<<ComboboxSelected>>", lambda e: self._on_field_change())

        # 初期選択を設定しておく
        if memory_map:
            self.combo.current(0)
            self.current_field = self.combo.get()
        else:
            self.current_field = None

        # 右下: Matplotlib 描画領域
        self.fig, self.ax = plt.subplots(figsize=(5,4))
        self.canvas = FigureCanvasTkAgg(self.fig, master=root)
        self.canvas.get_tk_widget().pack(side='right', fill='both', expand=True, padx=(2,5), pady=5)

        # 時系列データ保持用
        self.series = {name: deque(maxlen=WINDOW_SIZE) for name in memory_map.keys()}

        # 初回プロット準備
        if self.current_field:
            self._on_field_change()

        # 定期更新スタート
        root.after(0, self.update)

    def _on_field_change(self):
        # 選択フィールドが変わったとき
        self.current_field = self.combo.get()
        # 既存データをクリア
        self.series[self.current_field].clear()
        # プロットもクリア
        self.ax.clear()
        self.ax.set_title(self.current_field)
        self.canvas.draw()

    def update(self):
        # 1) 全メモリを読み出し
        mem = read_all_memory(self.dev)

        # 2) デコードしてテーブルを更新
        data = {}
        for name, (addr, length, typ) in memory_map.items():
            raw = mem[addr:addr+length]
            if typ == "c":
                val = raw.rstrip(b"\x00").decode("ascii", errors="ignore")
            else:
                fmt = TYPE_FMT.get(typ)
                val = struct.unpack_from(fmt, raw)[0]
            data[name] = val
            self.tree.set(name, 'value', val)

        # 3) 選択フィールドの時系列プロット更新
        if self.current_field:
            val = data[self.current_field]
            if isinstance(val, (int, float)):
                dq = self.series[self.current_field]
                dq.append(val)
                xs = list(range(len(dq)))
                self.ax.clear()
                self.ax.plot(xs, dq, marker='o', markersize=3)
                self.ax.set_title(self.current_field)
                self.ax.set_xlabel("Sample")
                self.ax.set_ylabel("Value")
                self.canvas.draw()

        # 4) 次回更新予約
        self.root.after(UPDATE_INTERVAL, self.update)

    def on_close(self):
        # 終了時にデバイスをクローズ
        try:
            self.dev.close()
        except:
            pass
        self.root.destroy()


def main():
    root = tk.Tk()
    app = MemoryViewerApp(root)
    root.protocol("WM_DELETE_WINDOW", app.on_close)
    root.mainloop()


if __name__ == "__main__":
    main()
