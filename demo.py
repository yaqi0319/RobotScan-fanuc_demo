"""
Fanuc Robot Modbus TCP Demo
===========================
协议说明（modbus_protocol.md）:
  寄存器 1-50   实际位置 POS[0]  (读取机器人当前坐标系坐标值和关节角度)
  寄存器 101-150 目标位置 PR[1]
  寄存器 201-218  程序状态 (18 个寄存器)

机器人 IP: 192.168.1.10
端口:      502 (Modbus TCP 默认)

三步演示:
  Step 1 - 读取机器人当前位置
  Step 2 - 判断机器人是否在移动
  Step 3 - 机器人移动到目标位置（已注释，位置安全性待确认后再启用）
"""

import time
import sys
from src.FanucController import FanucController

# ============================================================
# 协议地址常量（来自 modbus_protocol.md）
# ============================================================
ROBOT_HOST = "192.168.1.10"   # 机器人 IP
ROBOT_PORT = 502               # Modbus TCP 端口

# 寄存器 1-50：实际位置 POS[0]
# 坐标值为 32 位浮点数，每个坐标占 2 个寄存器
# 地址从 1 开始（Modbus 协议中地址偏移 0，即 ADR=0 对应寄存器 1）
POS_BASE_ADDR   = 0    # 对应寄存器 1（ADR=0）

# 寄存器 201-218：程序状态（18 个寄存器）
STATUS_BASE_ADDR = 200  # 对应寄存器 201（ADR=200）
STATUS_LEN       = 18

# 关节角度：协议说"1-50 寄存器包含当前坐标系坐标值和关节角度"
# 直角坐标(X,Y,Z,W,P,R) 占 12 个寄存器（6 轴 × 2 寄存器），从地址 0 开始
# 关节角度(J1-J6) 同样 12 个寄存器，从地址 12 开始（寄存器 13-24）
CARTESIAN_ADDR = 0    # 寄存器 1-12：直角坐标
JOINT_ADDR     = 12   # 寄存器 13-24：关节角度

# 寄存器 101-150：目标位置 PR[1]
TARGET_BASE_ADDR = 100  # 对应寄存器 101（ADR=100）

# 程序状态中"机器人是否在运动"的判断位（根据实际 PMC/TP 配置可能不同）
# 这里假设 STATUS 寄存器 201 的 bit0 = 1 表示机器人正在运动
MOVING_STATUS_REG_OFFSET = 0   # 状态寄存器中第 0 个（即寄存器 201）


def step1_read_current_position(robot: FanucController):
    """
    Step 1: 读取机器人当前位置
    从寄存器 1-24 读取当前直角坐标和关节角度（均在 1-50 范围内）。
    """
    print("\n" + "=" * 55)
    print("  Step 1: 读取机器人当前位置")
    print("=" * 55)

    # 读取直角坐标 X, Y, Z, W, P, R（寄存器 1-12，ADR=0, LEN=12）
    print(f"  [读取] 直角坐标 寄存器地址: {CARTESIAN_ADDR+1} ~ {CARTESIAN_ADDR+12}")
    cart_regs = robot.read(FC=3, ADR=CARTESIAN_ADDR, LEN=12)

    from src.FanucController import registers_to_float
    labels = ["X", "Y", "Z", "W", "P", "R"]
    cartesian = []
    for i in range(6):
        val = registers_to_float(cart_regs[i * 2], cart_regs[i * 2 + 1])
        cartesian.append(val)

    print("\n  当前直角坐标 (POS[0]):")
    for label, val in zip(labels, cartesian):
        print(f"    {label:>2} = {val:>10.3f}")

    # 读取关节角度 J1-J6（寄存器 13-24，ADR=12, LEN=12）
    print(f"\n  [读取] 关节角度 寄存器地址: {JOINT_ADDR+1} ~ {JOINT_ADDR+12}")
    joint_regs = robot.read(FC=3, ADR=JOINT_ADDR, LEN=12)

    j_labels = ["J1", "J2", "J3", "J4", "J5", "J6"]
    joints = []
    for i in range(6):
        val = registers_to_float(joint_regs[i * 2], joint_regs[i * 2 + 1])
        joints.append(val)

    print("\n  当前关节角度:")
    for label, val in zip(j_labels, joints):
        print(f"    {label:>2} = {val:>10.3f} °")

    return cartesian, joints


def step2_check_is_moving(robot: FanucController):
    """
    Step 2: 判断机器人是否在移动
    读取程序状态寄存器 201-218（ADR=200, LEN=18）。
    根据协议，第一个状态寄存器（201）的值非 0 表示程序正在运行/机器人在运动。
    （实际含义请根据示教器 PMC 配置确认）
    """
    print("\n" + "=" * 55)
    print("  Step 2: 判断机器人是否在移动")
    print("=" * 55)

    print(f"  [读取] 程序状态 寄存器地址: {STATUS_BASE_ADDR+1} ~ {STATUS_BASE_ADDR+STATUS_LEN}")
    status_regs = robot.read(FC=3, ADR=STATUS_BASE_ADDR, LEN=STATUS_LEN)

    print("\n  程序状态寄存器原始值:")
    for i, val in enumerate(status_regs):
        print(f"    寄存器 {STATUS_BASE_ADDR + 1 + i:>3} = {val:#06x}  ({val})")

    # 判断是否在移动：取状态寄存器 201（第 0 个）
    # bit0 = 1 表示 TP 程序正在运行（即机器人在运动）
    # 此判断逻辑需根据实际 Fanuc PMC 配置调整
    moving_reg = status_regs[MOVING_STATUS_REG_OFFSET]
    is_moving = bool(moving_reg & 0x01)   # 取最低位

    print(f"\n  运动状态寄存器 (寄存器 {STATUS_BASE_ADDR+1}): {moving_reg:#06x}")
    print(f"  机器人是否在移动: {'✓ 是（正在运动）' if is_moving else '✗ 否（静止）'}")

    return is_moving


def step3_move_to_target(robot: FanucController, target: list[float]):
    """
    Step 3: 机器人移动到目标位置
    ⚠️ 此步骤已注释，位置安全性待确认后再启用。

    操作说明：
      1. 将目标坐标写入寄存器 101-112（ADR=100，12 个寄存器）
      2. 写入触发信号，TP 程序检测到后执行移动
      3. 等待运动完成
    """
    print("\n" + "=" * 55)
    print("  Step 3: 机器人移动到目标位置")
    print("=" * 55)

    # ⚠️ 下面所有代码均已注释 —— 确认目标位置安全后再取消注释 ⚠️
    #
    # 目标位置（请在确认安全后填写实际坐标）：
    # target = [X, Y, Z, W, P, R]
    #
    # print(f"  目标坐标: X={target[0]:.2f}, Y={target[1]:.2f}, Z={target[2]:.2f}, "
    #       f"W={target[3]:.2f}, P={target[4]:.2f}, R={target[5]:.2f}")
    # print(f"  [写入] 目标位置 寄存器地址: {TARGET_BASE_ADDR+1} ~ {TARGET_BASE_ADDR+12}")
    #
    # # 将浮点坐标写入寄存器 101-112（ADR=100）
    # robot.move_robot(
    #     target_position=target,
    #     base_addr=TARGET_BASE_ADDR,   # 写入寄存器 101 开始
    #     trigger_addr=1,               # 触发地址（根据 TP 程序配置调整）
    #     trigger_fc=6,                 # FC=6 写单保持寄存器
    # )
    #
    # print("\n  等待机器人运动完成...")
    # for _ in range(30):
    #     time.sleep(0.5)
    #     still_moving = step2_check_is_moving(robot)
    #     if not still_moving:
    #         print("\n  ✓ 机器人已到达目标位置，运动完成！")
    #         break
    # else:
    #     print("\n  ⚠ 超时：机器人未在预期时间内停止，请检查 TP 程序。")

    print("\n  ⚠️  Step 3 已注释，位置安全性确认后再启用。")
    print(f"     预设目标坐标: {target}")
    print("     取消注释 step3_move_to_target() 内部代码后即可执行移动。")


def main():
    print("=" * 55)
    print("   Fanuc Robot Modbus TCP Demo")
    print("=" * 55)
    print(f"  机器人 IP:  {ROBOT_HOST}")
    print(f"  端口:       {ROBOT_PORT}")

    # ------------------------------------------------------------------
    # 连接机器人
    # ------------------------------------------------------------------
    try:
        robot = FanucController(host=ROBOT_HOST, port=ROBOT_PORT, timeout=3.0)
        print(f"\n  ✓ 成功连接到机器人 {ROBOT_HOST}:{ROBOT_PORT}")
    except ConnectionError as e:
        print(f"\n  ✗ 连接失败: {e}", file=sys.stderr)
        print("  请确认：")
        print("    1. 机器人已开机，IP 地址为 192.168.1.10")
        print("    2. Modbus TCP 服务已在示教器中启用")
        print("    3. PC 与机器人在同一网段")
        sys.exit(1)

    try:
        # ------------------------------------------------------------------
        # Step 1: 读取当前位置
        # ------------------------------------------------------------------
        cartesian, joints = step1_read_current_position(robot)

        # ------------------------------------------------------------------
        # Step 2: 判断是否在移动
        # ------------------------------------------------------------------
        is_moving = step2_check_is_moving(robot)

        # ------------------------------------------------------------------
        # Step 3: 移动到目标位置（已注释，待确认安全后启用）
        # ------------------------------------------------------------------
        # ⚠️ 在确认以下坐标安全之前，请勿取消注释 step3 内部的代码！
        target_position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # TODO: 填写安全目标坐标
        step3_move_to_target(robot, target_position)

        print("\n" + "=" * 55)
        print("  Demo 完成")
        print("=" * 55)

    except Exception as e:
        print(f"\n  ✗ 运行异常: {e}", file=sys.stderr)

    finally:
        robot.disconnect()
        print("  连接已断开。")


if __name__ == "__main__":
    main()
