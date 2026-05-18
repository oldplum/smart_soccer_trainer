#!/usr/bin/env python3
"""
WiFi Motion Data Receiver
接收来自ESP32的动作传感器数据并保存到CSV文件
"""

import socket
import csv
import sys
import os
from datetime import datetime
import argparse


class MotionDataReceiver:
    """接收并保存动作传感器数据"""

    def __init__(self, port=5000, csv_file=None):
        self.port = port
        self.csv_file = csv_file or f"motion_data_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        self.socket = None
        self.csv_writer = None
        self.csv_file_handle = None
        self.sample_count = 0

    def setup_csv(self):
        """创建CSV文件并写入表头"""
        try:
            self.csv_file_handle = open(self.csv_file, 'w', newline='')
            self.csv_writer = csv.writer(self.csv_file_handle)
            # CSV列: timestamp, acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, pitch, roll, acc_total
            self.csv_writer.writerow([
                'Timestamp(ms)',
                'Acc_X(g)',
                'Acc_Y(g)',
                'Acc_Z(g)',
                'Acc_Total(g)',
                'Gyro_X(deg/s)',
                'Gyro_Y(deg/s)',
                'Gyro_Z(deg/s)',
                'Pitch(deg)',
                'Roll(deg)'
            ])
            self.csv_file_handle.flush()
            print(f"✓ CSV文件已创建: {self.csv_file}")
        except IOError as e:
            print(f"✗ 无法创建CSV文件: {e}")
            sys.exit(1)

    def create_server_socket(self):
        """创建并绑定服务器socket"""
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        
        try:
            self.socket.bind(('0.0.0.0', self.port))
            self.socket.listen(1)
            print(f"✓ 服务器已启动，监听端口: {self.port}")
            print(f"  请配置ESP32连接到此服务器的IP地址")
        except OSError as e:
            print(f"✗ 无法绑定端口 {self.port}: {e}")
            sys.exit(1)

    def accept_client(self):
        """接受客户端连接"""
        print("等待ESP32连接...")
        client_socket, client_addr = self.socket.accept()
        print(f"✓ ESP32已连接: {client_addr[0]}:{client_addr[1]}")
        return client_socket

    def receive_and_save_data(self, client_socket):
        """接收数据并保存"""
        buffer = ""
        try:
            while True:
                data = client_socket.recv(4096).decode('utf-8', errors='ignore')
                if not data:
                    print("✗ 连接已断开")
                    break

                buffer += data

                # 按行处理数据
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    line = line.strip()

                    if not line:
                        continue

                    try:
                        # 解析CSV行: timestamp,acc_x,acc_y,acc_z,acc_total,gyro_x,gyro_y,gyro_z,pitch,roll
                        values = line.split(',')
                        if len(values) == 10:
                            self.csv_writer.writerow(values)
                            self.sample_count += 1

                            # 每10个样本刷新一次文件
                            if self.sample_count % 10 == 0:
                                self.csv_file_handle.flush()
                                sys.stdout.write(f"\r✓ 已接收 {self.sample_count} 条数据")
                                sys.stdout.flush()
                        else:
                            print(f"✗ 数据格式错误: {line}")
                    except ValueError as e:
                        print(f"✗ 解析数据失败: {e}")
                        continue

        except KeyboardInterrupt:
            print("\n✓ 用户中断")
        except Exception as e:
            print(f"✗ 接收数据时出错: {e}")
        finally:
            client_socket.close()

    def cleanup(self):
        """清理资源"""
        if self.csv_file_handle:
            self.csv_file_handle.flush()
            self.csv_file_handle.close()
            print(f"\n✓ CSV文件已保存: {self.csv_file}")
            print(f"  总共接收: {self.sample_count} 条数据")

        if self.socket:
            self.socket.close()

    def run(self):
        """运行接收程序"""
        print("=" * 60)
        print("WiFi 动作传感器数据接收程序")
        print("=" * 60)

        self.setup_csv()
        self.create_server_socket()

        while True:
            try:
                client = self.accept_client()
                self.receive_and_save_data(client)
            except KeyboardInterrupt:
                print("\n✓ 程序已停止")
                break
            except Exception as e:
                print(f"✗ 错误: {e}")
                break
            finally:
                self.cleanup()


def main():
    parser = argparse.ArgumentParser(
        description='接收ESP32的WiFi动作传感器数据并保存到CSV'
    )
    parser.add_argument(
        '-p', '--port',
        type=int,
        default=5000,
        help='监听端口 (默认: 5000)'
    )
    parser.add_argument(
        '-o', '--output',
        type=str,
        help='输出CSV文件路径 (默认: motion_data_YYYYMMDD_HHMMSS.csv)'
    )

    args = parser.parse_args()

    receiver = MotionDataReceiver(port=args.port, csv_file=args.output)
    receiver.run()


if __name__ == '__main__':
    main()
