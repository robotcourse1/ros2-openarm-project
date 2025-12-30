#!/usr/bin/env python3
"""
轨迹平滑度与成功率数据分析工具
生成曲线图和统计报告
"""

import csv
import json
import matplotlib
matplotlib.use('Agg')  # 非交互式后端
import matplotlib.pyplot as plt
import numpy as np
from pathlib import Path
from datetime import datetime
from typing import List, Dict


class StatisticsAnalyzer:
    """统计数据分析器"""
    
    def __init__(self, results_dir: Path = None):
        if results_dir is None:
            # 结果目录：~/openarm_ws/results/motion_control
            results_dir = Path.home() / "openarm_ws" / "results" / "motion_control"
        self.results_dir = Path(results_dir)
        self.results_dir.mkdir(parents=True, exist_ok=True)
    
    def load_trajectory_data(self) -> List[Dict]:
        """加载轨迹数据"""
        trajectories = []
        for json_file in self.results_dir.glob("trajectories_*.json"):
            with open(json_file, 'r') as f:
                data = json.load(f)
                if isinstance(data, list):
                    trajectories.extend(data)
                else:
                    trajectories.append(data)
        return trajectories
    
    def load_grasp_results(self) -> List[Dict]:
        """加载抓取结果"""
        results = []
        result_file = self.results_dir / "grasp_results.csv"
        if result_file.exists():
            with open(result_file, 'r') as f:
                reader = csv.DictReader(f)
                results = list(reader)
        return results
    
    def calculate_statistics(self) -> Dict:
        """计算统计数据"""
        trajectories = self.load_trajectory_data()
        grasp_results = self.load_grasp_results()
        
        stats = {
            'total_trajectories': len(trajectories),
            'total_grasp_attempts': len(grasp_results),
            'successful_grasps': sum(1 for r in grasp_results if r.get('success', '').lower() == 'true'),
            'failed_grasps': sum(1 for r in grasp_results if r.get('success', '').lower() == 'false'),
        }
        
        if stats['total_grasp_attempts'] > 0:
            stats['success_rate'] = (stats['successful_grasps'] / stats['total_grasp_attempts']) * 100.0
        else:
            stats['success_rate'] = 0.0
        
        # 轨迹平滑度统计
        if trajectories:
            vel_smoothness = [t.get('vel_smoothness', 0) for t in trajectories if 'vel_smoothness' in t]
            acc_smoothness = [t.get('acc_smoothness', 0) for t in trajectories if 'acc_smoothness' in t]
            
            stats['avg_vel_smoothness'] = np.mean(vel_smoothness) if vel_smoothness else 0.0
            stats['std_vel_smoothness'] = np.std(vel_smoothness) if vel_smoothness else 0.0
            stats['avg_acc_smoothness'] = np.mean(acc_smoothness) if acc_smoothness else 0.0
            stats['std_acc_smoothness'] = np.std(acc_smoothness) if acc_smoothness else 0.0
            
            # 执行时间统计
            durations = [t.get('duration', 0) for t in trajectories if 'duration' in t]
            stats['avg_duration'] = np.mean(durations) if durations else 0.0
            stats['min_duration'] = np.min(durations) if durations else 0.0
            stats['max_duration'] = np.max(durations) if durations else 0.0
        
        return stats
    
    def plot_success_rate(self, output_file: str = None):
        """绘制成功率统计图"""
        grasp_results = self.load_grasp_results()
        if not grasp_results:
            print("没有抓取结果数据可绘制")
            return
        
        # 按时间排序
        grasp_results.sort(key=lambda x: x.get('timestamp', ''))
        
        # 计算累积成功率
        successes = 0
        total = 0
        cumulative_rate = []
        timestamps = []
        
        for result in grasp_results:
            total += 1
            if result.get('success', '').lower() == 'true':
                successes += 1
            rate = (successes / total) * 100.0
            cumulative_rate.append(rate)
            timestamps.append(total)
        
        fig, axes = plt.subplots(1, 2, figsize=(12, 5))
        fig.suptitle('抓取成功率分析', fontsize=16, fontweight='bold')
        
        # 累积成功率曲线
        ax1 = axes[0]
        ax1.plot(timestamps, cumulative_rate, 'b-', linewidth=2, label='累积成功率')
        ax1.axhline(y=90, color='r', linestyle='--', label='目标成功率 (90%)')
        ax1.set_xlabel('抓取尝试次数')
        ax1.set_ylabel('成功率 (%)')
        ax1.set_title('累积成功率变化')
        ax1.legend()
        ax1.grid(True, alpha=0.3)
        ax1.set_ylim([0, 100])
        
        # 成功率柱状图（按手臂）
        ax2 = axes[1]
        left_results = [r for r in grasp_results if r.get('arm') == 'left_arm']
        right_results = [r for r in grasp_results if r.get('arm') == 'right_arm']
        
        left_success = sum(1 for r in left_results if r.get('success', '').lower() == 'true')
        left_total = len(left_results)
        right_success = sum(1 for r in right_results if r.get('success', '').lower() == 'true')
        right_total = len(right_results)
        
        arms = []
        success_rates = []
        if left_total > 0:
            arms.append('左臂')
            success_rates.append((left_success / left_total) * 100.0)
        if right_total > 0:
            arms.append('右臂')
            success_rates.append((right_success / right_total) * 100.0)
        
        if arms:
            colors = ['blue' if '左' in a else 'red' for a in arms]
            bars = ax2.bar(arms, success_rates, color=colors, alpha=0.7, edgecolor='black', linewidth=1.5)
            ax2.axhline(y=90, color='r', linestyle='--', label='目标成功率 (90%)')
            ax2.set_ylabel('成功率 (%)')
            ax2.set_title('各手臂成功率对比')
            ax2.set_ylim([0, 100])
            ax2.legend()
            ax2.grid(True, alpha=0.3, axis='y')
            
            # 添加数值标签
            for bar, rate in zip(bars, success_rates):
                height = bar.get_height()
                ax2.text(bar.get_x() + bar.get_width()/2., height,
                        f'{rate:.1f}%',
                        ha='center', va='bottom', fontweight='bold')
        
        plt.tight_layout()
        
        if output_file is None:
            output_file = self.results_dir / f"success_rate_{datetime.now().strftime('%Y%m%d_%H%M%S')}.png"
        else:
            output_file = Path(output_file)
        
        plt.savefig(output_file, dpi=300, bbox_inches='tight')
        print(f"成功率分析图已保存: {output_file}")
        plt.close()
    
    def generate_report(self, output_file: str = None):
        """生成统计报告"""
        stats = self.calculate_statistics()
        
        if output_file is None:
            output_file = self.results_dir / f"statistics_report_{datetime.now().strftime('%Y%m%d_%H%M%S')}.txt"
        else:
            output_file = Path(output_file)
        
        with open(output_file, 'w', encoding='utf-8') as f:
            f.write("=" * 60 + "\n")
            f.write("运动规划与控制统计报告\n")
            f.write("=" * 60 + "\n\n")
            f.write(f"生成时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n\n")
            
            f.write("1. 抓取统计\n")
            f.write("-" * 60 + "\n")
            f.write(f"总抓取尝试次数: {stats['total_grasp_attempts']}\n")
            f.write(f"成功次数: {stats['successful_grasps']}\n")
            f.write(f"失败次数: {stats['failed_grasps']}\n")
            f.write(f"成功率: {stats['success_rate']:.2f}%\n\n")
            
            f.write("2. 轨迹统计\n")
            f.write("-" * 60 + "\n")
            f.write(f"总轨迹数: {stats['total_trajectories']}\n")
            if 'avg_duration' in stats:
                f.write(f"平均执行时间: {stats['avg_duration']:.3f} 秒\n")
                f.write(f"最短执行时间: {stats.get('min_duration', 0):.3f} 秒\n")
                f.write(f"最长执行时间: {stats.get('max_duration', 0):.3f} 秒\n\n")
            
            f.write("3. 轨迹平滑度\n")
            f.write("-" * 60 + "\n")
            if 'avg_vel_smoothness' in stats:
                f.write(f"平均速度平滑度: {stats['avg_vel_smoothness']:.4f} ± {stats.get('std_vel_smoothness', 0):.4f}\n")
                f.write(f"平均加速度平滑度: {stats['avg_acc_smoothness']:.4f} ± {stats.get('std_acc_smoothness', 0):.4f}\n\n")
        
        print(f"统计报告已保存: {output_file}")
        return output_file


def main():
    """主函数：生成所有分析图表和报告"""
    analyzer = StatisticsAnalyzer()
    
    print("开始生成统计分析...")
    
    # 生成图表
    print("1. 生成成功率分析图...")
    analyzer.plot_success_rate()
    
    print("2. 生成统计报告...")
    analyzer.generate_report()
    
    # 打印统计摘要
    stats = analyzer.calculate_statistics()
    print("\n" + "=" * 60)
    print("统计摘要")
    print("=" * 60)
    print(f"成功率: {stats['success_rate']:.2f}%")
    print(f"总轨迹数: {stats['total_trajectories']}")
    if 'avg_vel_smoothness' in stats:
        print(f"平均速度平滑度: {stats['avg_vel_smoothness']:.4f}")
    print("=" * 60)


if __name__ == '__main__':
    main()

