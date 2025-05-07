import time
import functools
from typing import Callable, Any, Dict, List

# レイテンシー計測用デコレータ
def measure_latency(func: Callable) -> Callable:
    """
    関数の実行時間を計測し、結果と実行時間を返すデコレータ
    """
    # 統計情報を保持する辞書
    measure_latency.stats = getattr(measure_latency, 'stats', {})
    
    @functools.wraps(func)
    def wrapper(*args, **kwargs) -> tuple:
        # 関数名を取得
        func_name = func.__name__
        if func_name not in measure_latency.stats:
            measure_latency.stats[func_name] = {
                'times': [],  # 実行時間のリスト
                'count': 0,   # 実行回数
            }
        
        # 時間計測開始
        start_time = time.time()
        
        # 元の関数を実行
        result = func(*args, **kwargs)
        
        # 時間計測終了
        end_time = time.time()
        elapsed_ms = (end_time - start_time) * 1000  # ミリ秒に変換
        
        # 統計情報を更新
        measure_latency.stats[func_name]['times'].append(elapsed_ms)
        measure_latency.stats[func_name]['count'] += 1
        
        # 結果と実行時間を返す
        return result, {'latency_ms': elapsed_ms}
    
    return wrapper

def print_latency_stats(function_name: str = None) -> Dict:
    """
    計測された関数のレイテンシー統計情報を表示
    function_name が None の場合はすべての関数の統計を表示
    """
    stats = measure_latency.stats
    
    if function_name is not None and function_name in stats:
        functions = [function_name]
    else:
        functions = list(stats.keys())
    
    results = {}
    
    for func_name in functions:
        times = stats[func_name]['times']
        if not times:
            continue
            
        count = stats[func_name]['count']
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

def get_raw_data(function_name: str = None) -> Dict:
    """
    計測された生データを取得
    """
    stats = measure_latency.stats
    
    if function_name is not None and function_name in stats:
        return {function_name: stats[function_name]['times']}
    else:
        return {func_name: stats[func_name]['times'] for func_name in stats}

