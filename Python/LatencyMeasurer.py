import time
import functools
from typing import Callable, Dict, List, Any

class LatencyMeasurer:
    """
    関数の実行時間を計測するデコレータクラス
    """
    # クラス変数として統計情報を保持
    stats = {}

    def __init__(self, func: Callable):
        """
        デコレータの初期化
        """
        self.func = func
        functools.update_wrapper(self, func)  # メタデータのコピー

        # この関数の統計情報を初期化
        func_name = func.__name__
        if func_name not in self.stats:
            self.stats[func_name] = {
                'times': [],  # 実行時間のリスト
                'count': 0,   # 実行回数
            }
    
    def __call__(self, *args, **kwargs):
        """
        関数呼び出し時の処理
        """
        func_name = self.func.__name__

        # return_latency フラグを取り出し（原関数には渡さない）
        return_latency = kwargs.pop('return_latency', False)
        
        # 時間計測開始
        start_time = time.time()
        
        # 元の関数を実行
        result = self.func(*args, **kwargs)
        
        # 時間計測終了
        end_time = time.time()
        elapsed_ms = (end_time - start_time) * 1000  # ミリ秒に変換
        
        # 統計情報を更新
        self.stats[func_name]['times'].append(elapsed_ms)
        self.stats[func_name]['count'] += 1
        
        # フラグに応じて戻り値を切り替え
        if return_latency:
            return result, elapsed_ms
        else:
            return result
        #return result, {'latency_ms': elapsed_ms}
    
    @classmethod
    def print_stats(cls, function_name: str = None) -> Dict:
        """
        計測された関数のレイテンシー統計情報を表示
        """
        if function_name is not None and function_name in cls.stats:
            functions = [function_name]
        else:
            functions = list(cls.stats.keys())
        
        results = {}
        
        for func_name in functions:
            times = cls.stats[func_name]['times']
            if not times:
                continue
                
            count = cls.stats[func_name]['count']
            avg_time = sum(times) / count
            min_time = min(times)
            max_time = max(times)
            std_dev = (sum((t - avg_time) ** 2 for t in times) / count) ** 0.5
            
            print(f"\n===== {func_name} レイテンシー統計 =====")
            print(f"サンプル数: {count}")
            print(f"平均: {avg_time:.3f}ms")
            print(f"最小: {min_time:.3f}ms")
            print(f"最大: {max_time:.3f}ms")
            print(f"標準偏差: {std_dev:.3f}ms")
            
            results[func_name] = {
                'samples': count,
                'avg': avg_time,
                'min': min_time,
                'max': max_time,
                'std_dev': std_dev
            }
        
        return results
    
    @classmethod
    def get_raw_data(cls, function_name: str = None) -> Dict:
        """
        計測された生データを取得
        """
        if function_name is not None and function_name in cls.stats:
            return {function_name: cls.stats[function_name]['times']}
        else:
            return {func_name: cls.stats[func_name]['times'] for func_name in cls.stats}

