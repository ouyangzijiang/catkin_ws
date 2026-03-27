#!/bin/bash

# 一键运行L2_1文件夹中的三个Python文件，每个在独立的终端中运行

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "在新终端中启动 Publisher (L2_1_01_pub_chatter.py)..."
gnome-terminal -- bash -c "cd '$SCRIPT_DIR' && python3 L2_1_01_pub_chatter.py; read -p 'Press Enter to close...'" &



echo "在新终端中启动 Subscriber with spin (L2_1_02_sub_chatter_spin.py)..."
gnome-terminal -- bash -c "cd '$SCRIPT_DIR' && python3 L2_1_02_sub_chatter_spin.py; read -p 'Press Enter to close...'" &



echo "在新终端中启动 Subscriber without spin (L2_1_03_sub_chatter_no_spin.py)..."
gnome-terminal -- bash -c "cd '$SCRIPT_DIR' && python3 L2_1_03_sub_chatter_no_spin.py; read -p 'Press Enter to close...'" &


