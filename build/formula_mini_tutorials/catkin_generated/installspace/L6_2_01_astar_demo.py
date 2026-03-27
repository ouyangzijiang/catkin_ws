#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
L6-2 A* 路径规划可视化演示
============================
PythonRobotics 风格的 A* / Dijkstra 网格搜索动画。
学生可在笔记本或 ROS2GO 上直接运行，无需 ROS 环境。

用法:
    python3 L6_2_01_astar_demo.py            # A* 模式
    python3 L6_2_01_astar_demo.py --dijkstra  # Dijkstra 模式（对比用）

依赖: numpy, matplotlib
    pip install numpy matplotlib
"""

import argparse
import math
import heapq
import time

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches


# ─── 网格与节点 ────────────────────────────────────────────────

class Node:
    """搜索节点：记录坐标、代价、父节点"""

    def __init__(self, x: int, y: int, cost: float = 0.0, h: float = 0.0, parent=None):
        self.x = x
        self.y = y
        self.cost = cost        # g(n): 从起点到这里的实际代价
        self.h = h              # h(n): 到终点的估计代价
        self.parent = parent

    @property
    def f(self) -> float:
        """f(n) = g(n) + h(n)"""
        return self.cost + self.h

    def __lt__(self, other):
        return self.f < other.f


# ─── 地图定义 ──────────────────────────────────────────────────

def create_grid(width: int = 40, height: int = 30):
    """
    创建一个带障碍的二维网格。
    0 = 可走（空地），1 = 障碍（墙）。
    学生可以修改这个函数来改变障碍布局。
    """
    grid = np.zeros((height, width), dtype=int)

    # ---- 边界墙 ----
    grid[0, :] = 1
    grid[-1, :] = 1
    grid[:, 0] = 1
    grid[:, -1] = 1

    # ---- 障碍物（模拟赛道墙） ----
    # 横墙 1
    grid[8, 5:20] = 1
    # 横墙 2
    grid[18, 15:35] = 1
    # 竖墙 1
    grid[5:16, 25] = 1
    # L 形障碍
    grid[22:27, 8] = 1
    grid[26, 8:18] = 1

    return grid


# ─── 启发函数 ──────────────────────────────────────────────────

def heuristic_euclidean(x1, y1, x2, y2):
    """欧几里得距离（不会高估 → admissible）"""
    return math.hypot(x2 - x1, y2 - y1)


# ─── A* 搜索 ──────────────────────────────────────────────────

# 8 连通：上下左右 + 对角，路径代价分别为 1 和 sqrt(2)
MOTIONS = [
    (1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),        # 上下左右
    (1, 1, math.sqrt(2)), (1, -1, math.sqrt(2)),                    # 对角
    (-1, 1, math.sqrt(2)), (-1, -1, math.sqrt(2)),
]


def astar(grid, start, goal, use_heuristic=True):
    """
    A* 网格搜索。
    返回 (path, visited_order) 用于可视化。
    如果 use_heuristic=False 则退化为 Dijkstra。
    """
    height, width = grid.shape
    sx, sy = start
    gx, gy = goal
#计算从起点到目标的启发式估计（欧几里得距离），cost为0，因为起点到起点的实际代价为0。
# 启发式函数 h0 只在 A* 模式下计算，Dijkstra 模式下设为0。
    h0 = heuristic_euclidean(sx, sy, gx, gy) if use_heuristic else 0.0
    start_node = Node(sx, sy, cost=0.0, h=h0)

    # open list（优先队列）
    open_list = []
    heapq.heappush(open_list, start_node)

    # closed set（已展开），已访问节点
    closed = set()

    # 存储每个节点的最小 g(n)，即从起点到该节点的最短实际路径代价。
    cost_so_far = {(sx, sy): 0.0}

    visited_order = []  # 记录展开顺序，用于动画

    while open_list:
        current = heapq.heappop(open_list) 
        #f 最小的节点，包含了从起点到当前节点的代价 g(n) 和从当前节点到目标的估计代价 h(n)。

        if (current.x, current.y) in closed:
            continue
        closed.add((current.x, current.y))
        visited_order.append((current.x, current.y))

        # 到达终点，回溯路径
        if current.x == gx and current.y == gy:
            path = []
            node = current
            while node is not None:
                path.append((node.x, node.y))
                node = node.parent
            path.reverse()
            return path, visited_order, cost_so_far

        # 展开邻居
        for dx, dy, move_cost in MOTIONS:
            nx, ny = current.x + dx, current.y + dy

            # 边界检查
            if nx < 0 or nx >= width or ny < 0 or ny >= height:
                continue
            # 障碍检查
            if grid[ny, nx] == 1:
                continue
            # 已展开
            if (nx, ny) in closed:
                continue
            #计算新代价，并更新 open_list
            new_cost = current.cost + move_cost
            if (nx, ny) in cost_so_far and new_cost >= cost_so_far[(nx, ny)]:
                continue

            cost_so_far[(nx, ny)] = new_cost
            h = heuristic_euclidean(nx, ny, gx, gy) if use_heuristic else 0.0
            neighbor = Node(nx, ny, cost=new_cost, h=h, parent=current)
            heapq.heappush(open_list, neighbor)

    # 无解
    return None, visited_order, cost_so_far


# ─── 可视化 ─────────────────────────────────────────────────────

def visualize(grid, start, goal, path, visited_order, cost_map, algorithm_name="A*"):
    """
    动画显示搜索过程 + 最终路径。
    黄色 = 已展开，红色 = 最终路径，绿色 = 起点，蓝色 = 终点。
    每个展开节点上标注 g(n) 代价值（取整），同代价层数字相同。
    """
    height, width = grid.shape

    fig, ax = plt.subplots(1, 1, figsize=(12, 9))
    ax.set_xlim(-0.5, width - 0.5)
    ax.set_ylim(-0.5, height - 0.5)
    ax.set_aspect('equal')
    ax.invert_yaxis()
    ax.set_title(f'{algorithm_name} Path Planning — Expanded: ?', fontsize=14)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')

    # 画网格和障碍
    for y in range(height):
        for x in range(width):
            if grid[y, x] == 1:
                rect = patches.Rectangle((x - 0.5, y - 0.5), 1, 1,
                                         linewidth=0, facecolor='#333333')
                ax.add_patch(rect)

    # 画网格线
    for x in range(width + 1):
        ax.axvline(x - 0.5, color='#cccccc', linewidth=0.3)
    for y in range(height + 1):
        ax.axhline(y - 0.5, color='#cccccc', linewidth=0.3)

    # 起点和终点
    ax.plot(start[0], start[1], 'o', color='#00cc00', markersize=12, label='Start')
    ax.plot(goal[0], goal[1], 's', color='#0066ff', markersize=12, label='Goal')
    ax.legend(loc='upper right', fontsize=10)

    plt.ion()
    plt.show()

    # ---- 动画：逐步显示展开过程（每个节点标注 g(n) 代价） ----
    batch_size = max(1, len(visited_order) // 80)  # 分批绘制，约 80 帧
    for i in range(0, len(visited_order), batch_size):
        batch_indices = range(i, min(i + batch_size, len(visited_order)))
        for idx in batch_indices:
            px, py = visited_order[idx]
            ax.plot(px, py, 's', color='#ffdd44', markersize=8, alpha=0.6)
            g_val = cost_map.get((px, py), 0)
            label = str(int(round(g_val)))
            ax.text(px, py, label, fontsize=4.5, ha='center', va='center',
                    color='#333333', fontweight='bold')
        ax.set_title(
            f'{algorithm_name} Path Planning — Expanded: {min(i + batch_size, len(visited_order))}',
            fontsize=14)
        plt.pause(0.03)

    # ---- 画最终路径 ----
    if path:
        px = [p[0] for p in path]
        py = [p[1] for p in path]
        ax.plot(px, py, '-', color='#ff3333', linewidth=2.5, label=f'Path ({len(path)} pts)')
        ax.plot(px, py, 'o', color='#ff3333', markersize=3)
        ax.legend(loc='upper right', fontsize=10)
        ax.set_title(
            f'{algorithm_name} — Expanded {len(visited_order)} nodes, Path {len(path)} pts',
            fontsize=14)
    else:
        ax.set_title(f'{algorithm_name} — No Solution! Expanded {len(visited_order)} nodes', fontsize=14)

    plt.ioff()
    print(f"\n{'='*50}")
    print(f"Algorithm: {algorithm_name}")
    print(f"Expanded:   {len(visited_order)}")
    if path:
        print(f"Path pts:   {len(path)}")
        total_cost = sum(
            math.hypot(path[i+1][0] - path[i][0], path[i+1][1] - path[i][1])
            for i in range(len(path) - 1)
        )
        print(f"Path cost:  {total_cost:.2f}")
    else:
        print("No path found!")
    print(f"{'='*50}\n")

    plt.show()


# ─── 主程序 ─────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description='A* / Dijkstra Path Planning Demo')
    parser.add_argument('--dijkstra', action='store_true',
                        help='Use Dijkstra (no heuristic) instead of A*')
    args = parser.parse_args()

    # 创建地图
    grid = create_grid(width=40, height=30)

    # 起点和终点（可修改）
    start = (2, 2)
    goal = (37, 27)

    # 选择算法
    use_heuristic = not args.dijkstra
    algorithm_name = "A*" if use_heuristic else "Dijkstra"

    print(f"Running {algorithm_name} path planning...")
    print(f"Start: {start}, Goal: {goal}")
    print(f"Grid size: {grid.shape[1]} x {grid.shape[0]}")

    t0 = time.time()
    path, visited, cost_map = astar(grid, start, goal, use_heuristic=use_heuristic)
    elapsed = time.time() - t0
    print(f"Search time: {elapsed*1000:.1f} ms")

    # 可视化
    visualize(grid, start, goal, path, visited, cost_map, algorithm_name)


if __name__ == '__main__':
    main()
