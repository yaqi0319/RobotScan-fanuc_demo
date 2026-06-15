import socket
import struct
import threading
import time
import sys
from src.FanucController import FanucController


class MockFanucRobotServer:
    """
    一个用于本地测试的仿真 Fanuc 机械臂 Modbus TCP 服务端。
    它不仅支持读写对应的线圈与保持寄存器，还模拟了真实机械臂运动的过程：
    当接收到运动触发信号时，机械臂变为“在移动”状态，过 1.5 秒后更新当前坐标为目标坐标，
    随后结束移动状态并清除触发信号。
    """
    def __init__(self, host='127.0.0.1', port=5020):
        self.host = host
        self.port = port
        self.running = False
        self.server_sock = None
        self.thread = None
        self.move_thread = None
        
        # 寄存器字典 (Modbus 地址 -> 16位整型)
        self.registers = {}
        # 线圈字典 (Modbus 地址 -> 布尔值)
        self.coils = {}
        # 离散输入字典 (Modbus 地址 -> 布尔值)
        self.discrete_inputs = {}
        
        # 初始化寄存器数值
        # 当前直角坐标：X=100.0, Y=200.0, Z=300.0, W=0.0, P=90.0, R=180.0 (存放在 1000 开始的寄存器中)
        self.set_float_registers(1000, [100.0, 200.0, 300.0, 0.0, 90.0, 180.0])
        # 当前关节轴角度：J1=10.0, J2=20.0, J3=30.0, J4=40.0, J5=50.0, J6=60.0 (存放在 1020 开始的寄存器中)
        self.set_float_registers(1020, [10.0, 20.0, 30.0, 40.0, 50.0, 60.0])
        # 目标坐标：初始化为 0 (2000 开始的寄存器)
        self.set_float_registers(2000, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        
        # 运动状态信号（默认 False）
        self.coils[100] = False
        self.discrete_inputs[100] = False
        
        # 触发寄存器 (寄存器 1 / 线圈 1)
        self.registers[1] = 0
        self.coils[1] = False
        
    def set_float_registers(self, start_addr, values):
        for idx, val in enumerate(values):
            data = struct.pack('>f', val)
            reg1, reg2 = struct.unpack('>HH', data)
            self.registers[start_addr + idx * 2] = reg1
            self.registers[start_addr + idx * 2 + 1] = reg2

    def get_float_registers(self, start_addr):
        values = []
        for i in range(6):
            r1 = self.registers.get(start_addr + i * 2, 0)
            r2 = self.registers.get(start_addr + i * 2 + 1, 0)
            data = struct.pack('>HH', r1, r2)
            val = struct.unpack('>f', data)[0]
            values.append(val)
        return values

    def start(self):
        self.server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_sock.bind((self.host, self.port))
        self.server_sock.listen(5)
        self.running = True
        self.thread = threading.Thread(target=self._listen_loop, daemon=True)
        self.thread.start()
        
        # 启动机器人模拟动作线程
        self.move_thread = threading.Thread(target=self._movement_simulation_loop, daemon=True)
        self.move_thread.start()
        print(f"[Mock Server] 模拟 Fanuc 机器人服务端已在 {self.host}:{self.port} 启动。")

    def stop(self):
        self.running = False
        if self.server_sock:
            self.server_sock.close()
        print("[Mock Server] 服务端已关闭。")

    def _listen_loop(self):
        while self.running:
            try:
                conn, addr = self.server_sock.accept()
                t = threading.Thread(target=self._handle_client, args=(conn,), daemon=True)
                t.start()
            except Exception:
                break

    def _movement_simulation_loop(self):
        """模拟机器人的物理移动过程"""
        while self.running:
            # 检测是否触发了移动（检查线圈或保持寄存器中的触发标记）
            trigger = self.registers.get(1, 0) or (1 if self.coils.get(1, False) else 0)
            if trigger == 1:
                print("\n[Mock Server] 检测到移动触发信号！机械臂开始移动...")
                self.discrete_inputs[100] = True
                self.coils[100] = True
                
                # 模拟机器人移动所需的物理耗时
                time.sleep(1.5)
                
                # 移动完成，更新当前坐标值为下发的目标坐标
                target_pos = self.get_float_registers(2000)
                self.set_float_registers(1000, target_pos)
                
                # 模拟关节运动变化 (这里简单做些差值计算演示)
                self.set_float_registers(1020, [val + 5.0 for val in self.get_float_registers(1020)])
                
                print(f"[Mock Server] 机械臂已到达目标位置: {target_pos}")
                
                # 运动结束，复位状态与信号
                self.discrete_inputs[100] = False
                self.coils[100] = False
                self.registers[1] = 0
                self.coils[1] = False
                print("[Mock Server] 清除触发标记，结束移动状态。\n")
            else:
                time.sleep(0.1)

    def _handle_client(self, conn):
        conn.settimeout(5.0)
        while self.running:
            try:
                # 1. 读取 Modbus TCP 头 (6字节)
                header = conn.recv(6)
                if not header or len(header) < 6:
                    break
                tid, proto, length = struct.unpack('>HHH', header)
                
                # 2. 读取 Modbus TCP 体
                body = conn.recv(length)
                if not body or len(body) < length:
                    break
                
                unit_id = body[0]
                fc = body[1]
                
                # 3. 处理请求
                if fc in [1, 2]:  # 读线圈 (FC 1) 或读离散输入 (FC 2)
                    addr, qty = struct.unpack('>HH', body[2:6])
                    byte_count = (qty + 7) // 8
                    data_bytes = bytearray(byte_count)
                    for i in range(qty):
                        target_dict = self.coils if fc == 1 else self.discrete_inputs
                        val = target_dict.get(addr + i, False)
                        if val:
                            data_bytes[i // 8] |= (1 << (i % 8))
                    
                    resp_body = struct.pack('>BBB', unit_id, fc, byte_count) + data_bytes
                
                elif fc in [3, 4]:  # 读保持寄存器 (FC 3) 或读输入寄存器 (FC 4)
                    addr, qty = struct.unpack('>HH', body[2:6])
                    byte_count = qty * 2
                    data_regs = bytearray()
                    for i in range(qty):
                        val = self.registers.get(addr + i, 0)
                        data_regs.extend(struct.pack('>H', val))
                    
                    resp_body = struct.pack('>BBB', unit_id, fc, byte_count) + data_regs
                    
                elif fc == 5:  # 写单线圈
                    addr, val = struct.unpack('>HH', body[2:6])
                    self.coils[addr] = (val == 0xFF00)
                    resp_body = body
                    
                elif fc == 6:  # 写单寄存器
                    addr, val = struct.unpack('>HH', body[2:6])
                    self.registers[addr] = val
                    resp_body = body
                    
                elif fc == 16:  # 写多个保持寄存器
                    addr, qty, byte_count = struct.unpack('>HHB', body[2:7])
                    for i in range(qty):
                        val = struct.unpack('>H', body[7 + i*2 : 9 + i*2])[0]
                        self.registers[addr + i] = val
                    resp_body = struct.pack('>BHH', unit_id, addr, qty)
                    
                else:
                    break
                
                # 4. 发送响应
                resp_header = struct.pack('>HHH', tid, proto, len(resp_body))
                conn.sendall(resp_header + resp_body)
                
            except Exception:
                break
        conn.close()


def main():
    print("==================================================")
    print("      Fanuc Robot Modbus TCP Python 控制类演示      ")
    print("==================================================")
    
    # 1. 启动仿真服务端
    server = MockFanucRobotServer(host='127.0.0.1', port=5020)
    server.start()
    
    # 给服务端启动留一点时间
    time.sleep(0.5)
    
    # 2. 客户端连接与初始化
    # 连接到本地仿真服务端（真实机械臂请改成对应的机械臂 IP，如 192.168.1.10，端口默认 502）
    client = FanucController(host='127.0.0.1', port=5020, timeout=2.0)
    
    try:
        # 3. 读取当前位置 (X, Y, Z, W, P, R) 和关节坐标 (J1 - J6)
        # 这里使用 Holding Registers (fc=3) 以适配我们本地 mock 存储，如果是真机根据 TP 配置可能需要 fc=4
        curr_pos = client.read_position(base_addr=1000, fc=3)
        curr_joints = client.read_joint_position(base_addr=1020, fc=3)
        
        print("\n--- [读取初始状态] ---")
        print(f"当前直角坐标: X={curr_pos[0]:.2f}, Y={curr_pos[1]:.2f}, Z={curr_pos[2]:.2f}, "
              f"W={curr_pos[3]:.2f}, P={curr_pos[4]:.2f}, R={curr_pos[5]:.2f}")
        print(f"当前关节坐标: J1={curr_joints[0]:.2f}, J2={curr_joints[1]:.2f}, J3={curr_joints[2]:.2f}, "
              f"J4={curr_joints[3]:.2f}, J5={curr_joints[4]:.2f}, J6={curr_joints[5]:.2f}")
        
        # 4. 查询当前是否正在移动
        # 本地仿真状态存在 Discrete Input 100 中，功能码 2
        is_moving = client.is_moving(status_addr=100, fc=2)
        print(f"机械臂是否在移动: {is_moving}")
        
        # 5. 下发运动指令并移动机械臂
        # 下发目标直角坐标并触发运动 (下发到地址 2000，触发地址为寄存器 1，写功能码为 6)
        target = [150.5, -50.0, 280.0, 10.0, 45.0, 90.0]
        print(f"\n--- [触发运动指令] ---")
        print(f"目标下发坐标: {target}")
        
        client.move_robot(target, base_addr=2000, trigger_addr=1, trigger_fc=6)
        
        # 6. 循环查询是否正在运动，直到运动停止并到达目的地
        print("\n--- [运动状态跟踪监控] ---")
        for _ in range(20):
            moving_state = client.is_moving(status_addr=100, fc=2)
            pos = client.read_position(base_addr=1000, fc=3)
            print(f"  > [监控] 正在移动: {moving_state} | 当前坐标: X={pos[0]:.2f}, Y={pos[1]:.2f}, Z={pos[2]:.2f}")
            
            if not moving_state and pos[0] == target[0]:
                print(">>> 机械臂已平稳到达目的地，监控结束。")
                break
            time.sleep(0.3)
            
    except Exception as e:
        print(f"通信测试发生异常: {e}", file=sys.stderr)
        
    finally:
        # 7. 关闭与释放资源
        client.disconnect()
        server.stop()
        print("测试流程全部完毕。")


if __name__ == "__main__":
    main()
