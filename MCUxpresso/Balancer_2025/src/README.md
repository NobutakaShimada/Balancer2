# 立命館大学大学院情報理工学研究科　知能機械特論　倒立振子系の制御

## プログラム内容
- `bb2_control.ipynb` Python3による倒立振子のシミュレーションプログラム
  同じファイルが[Google Colab](https://colab.research.google.com/drive/1dfHT3Tn5mHuN4kQoALb1uVZLmnndbQBl?usp=sharing)にあるのでそちらから自分のColabのアカウントにコピーして実行するのがよい。
- `control.c` 制御ループの関数Control()が含まれる、ファームウェア（ビュートバランサー本体に書き込まれるプログラム）のソースコードファイル（の一部）。このファイルを主に編集することになる。
- 'main.c' ファームウェアのメインエントリ。外部からUSB経由で参照できる一部のメモリ領域の初期化なども含まれている。

## `bb2_control.ipynb`における値設定方法
- 極配置法を用いる場合は，use_LQRをFalseにする。
- 最適制御法を用いる場合は，use_LQRをTrueにして、QとRを適切に設定する。どの次元がどの状態に相当するかよく考えながら重みを与えること。
```
use_LQR = True
# ----------------------------------
if not use_LQR:
    # 極配置
    # 左2つ・右2つがそれぞれペアの共役複素数になるように設定（例：[-1+1j, -1-1j, -2+2j, -2-2j]）
    #p = [-...+...j, -...-...j, -...+...j, -...-...j]
    p = np.array([-30+1j, -30-1j, -10+2j, -10-2j])
    K = place_poles(A, B, p).gain_matrix # ゲイン
else :
    # 最適制御
    #Q = np.diag([..., ..., ..., ...])
    Q = np.diag([1.,1.,1.,1.])
    #R = np.array([...])
    R = np.array([1])
    P = linalg.solve_continuous_are(A, B, Q, R)
    K = np.dot(B.T, P)/R # ゲイン
    K = K@np.linalg.inv(C) # 出力フィードバック 実際のBBの内界センサでは車輪回転角は本体基準なので、鉛直方向基準のθ1'=θ1-θ2。これを補正する。
```

## `control.c`における値設定方法
最適制御法（もしくは極配置法）のシミュレーションで求めたゲインを用いて，K[0]~K[3]の値を設定する．
```
void Control(){
	volatile double outL = 0.0, outR = 0.0;
	// 状態量
	double x[4];
	x[0] = -(memmap.values.WHEEL_ANGLE_L + memmap.values.WHEEL_ANGLE_R)/2.0;
	x[1] = memmap.values.BODY_ANGLE;
	x[2] = -(memmap.values.WHEEL_ANGULAR_SPD_L + memmap.values.WHEEL_ANGULAR_SPD_R)/2.0;
	x[3] = memmap.values.BODY_ANGULAR_SPD;

	double u;

	//u = Control_PID(x);
	u = Control_Feedback(x);
	// u = Control_Adaptive(x);

	outL += u;
	outR += u;

	//最大指令値規制
	if(outL > 32767.0){
		outL = 32767.0;
	}
	if(outR > 32767.0){
		outR = 32767.0;
	}
	if(outL < -32767.0){
		outL = -32767.0;
	}
	if(outR < -32767.0){
		outR = -32767.0;
	}

	//電流指令値設定
	//memmap.values.T_CURRENT_L = (short)outL*1000;
	//memmap.values.T_CURRENT_R = (short)outR*1000;
	double gain = memmap.values.GAIN_OPTION5;
	memmap.values.T_CURRENT_L = (short)(outL*gain);
	memmap.values.T_CURRENT_R = (short)(outR*gain);

}
```

## 外部リンク
- [Google Colab ゲイン計算書](https://colab.research.google.com/drive/1dfHT3Tn5mHuN4kQoALb1uVZLmnndbQBl?usp=sharing)
- [ビュートバランサー2 取扱説明書](https://www.vstone.co.jp/products/beauto_balancer_2/download/BeautoBalancer2_Manual_1_05.pdf)
