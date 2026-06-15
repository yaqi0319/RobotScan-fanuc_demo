"""
Fanuc Robot Modbus TCP Demo
===========================
协议说明（modbus_protocol.md）:
  寄存器 1-50    实际位置 POS[0]  (当前坐标系坐标值和关节角度)
  寄存器 101-150 目标位置 PR[1]
  寄存器 201-218 程序状态 (18 个寄存器/任务)

机器人 IP: 192.168.1.10
端口:      502 (Modbus TCP 默认)

地址换算规则：Modbus 寄存器编号从 1 起，ADR 从 0 起
  → 寄存器 N  对应  ADR = N - 1

三步演示:
  Step 1 - 读取机器人当前位置
  Step 2 - 判断机器人是否在移动
  Step 3 - 机器人移动到目标位置（已注释，位置安全性待确认后再启用）
"""

import time
import sys
from src.FanucController import FanucController, registers_to_float, float_to_registers

# ============================================================
# 协议地址常量（来自 modbus_protocol.md + image-1.png + image.png）
# ============================================================
ROBOT_HOST = "192.168.1.10"   # 机器人 IP
ROBOT_PORT = 502               # Modbus TCP 端口

# ------ 实际位置 POS[0]（寄存器 1-50，ADR=0-49）------
# 每个 32 位浮点数占 2 个寄存器（高字在前，低字在后）
#
# 正交坐标 X/Y/Z/W/P/R：
#   寄存器 1-2   → X  (ADR=0 )
#   寄存器 3-4   → Y  (ADR=2 )
#   寄存器 5-6   → Z  (ADR=4 )
#   寄存器 7-8   → W  (ADR=6 )
#   寄存器 9-10  → P  (ADR=8 )
#   寄存器 11-12 → R  (ADR=10)
CARTESIAN_ADDR = 0    # ADR=0，读 LEN=12 → 寄存器 1-12
CARTESIAN_LEN  = 12

# 关节角度 J1-J6：
#   寄存器 27-28 → J1 (ADR=26)
#   寄存器 29-30 → J2 (ADR=28)
#   寄存器 31-32 → J3 (ADR=30)
#   寄存器 33-34 → J4 (ADR=32)
#   寄存器 35-36 → J5 (ADR=34)
#   寄存器 37-38 → J6 (ADR=36)
JOINT_ADDR = 26   # ADR=26，读 LEN=12 → 寄存器 27-38
JOINT_LEN  = 12

# ------ 目标位置 PR[1]（寄存器 101-150，ADR=100-149）------
# 正交坐标 X/Y/Z/W/P/R 写入寄存器 101-112（ADR=100, LEN=12）
TARGET_BASE_ADDR = 100  # ADR=100 → 寄存器 101

# ------ 程序状态（寄存器 201-218，ADR=200-217）------
# 每个任务 18 个寄存器，各字段偏移（1-based）：
#   偏移 1-8  → 程序名（字符串）
#   偏移 9    → 行编号
#   偏移 10   → 执行状态：0=结束, 1=暂停, 2=执行中
#   偏移 11-18→ 母程序名
STATUS_BASE_ADDR = 200  # ADR=200 → 寄存器 201
STATUS_LEN       = 18

# 执行状态寄存器：偏移 10（1-based）→ 数组下标 9（0-based）→ 寄存器 210 (ADR=209)
STATUS_EXEC_IDX  = 9    # read() 返回数组的下标
STATUS_RUNNING   = 2    # 值==2 表示 TP 程序执行中（机器人在运动）


# ============================================================
# Step 1: 读取机器人当前位置
# ============================================================
def step1_read_current_position(robot: FanucController):
    """
    读取直角坐标（X/Y/Z/W/P/R）和关节角度（J1-J6）。

    寄存器映射（来自 image-1.png）：
      直角坐标：寄存器 1-12  → ADR=0,  LEN=12
      关节角度：寄存器 27-38 → ADR=26, LEN=12
    """
    print("\n" + "=" * 55)
    print("  Step 1: 读取机器人当前位置")
    print("=" * 55)

    # --- 直角坐标 ---
    print(f"  [读取] 直角坐标  寄存器 {CARTESIAN_ADDR+1}~{CARTESIAN_ADDR+CARTESIAN_LEN}"
          f"  (ADR={CARTESIAN_ADDR}, LEN={CARTESIAN_LEN})")
    cart_regs = robot.read(FC=3, ADR=CARTESIAN_ADDR, LEN=CARTESIAN_LEN)

    cart_labels = [("X", "mm"), ("Y", "mm"), ("Z", "mm"),
                   ("W", "deg"), ("P", "deg"), ("R", "deg")]
    cartesian = []
    for i in range(6):
        val = registers_to_float(cart_regs[i * 2], cart_regs[i * 2 + 1])
        cartesian.append(val)

    print("\n  当前直角坐标 POS[0]:")
    for (label, unit), val in zip(cart_labels, cartesian):
        print(f"    {label:>2}  = {val:>10.3f}  {unit}")

    # --- 关节角度 ---
    print(f"\n  [读取] 关节角度  寄存器 {JOINT_ADDR+1}~{JOINT_ADDR+JOINT_LEN}"
          f"  (ADR={JOINT_ADDR}, LEN={JOINT_LEN})")
    joint_regs = robot.read(FC=3, ADR=JOINT_ADDR, LEN=JOINT_LEN)

    joints = []
    for i in range(6):
        val = registers_to_float(joint_regs[i * 2], joint_regs[i * 2 + 1])
        joints.append(val)

    print("\n  当前关节角度:")
    for i, val in enumerate(joints, 1):
        print(f"    J{i}  = {val:>10.3f}  deg")

    return cartesian, joints


# ============================================================
# Step 2: 判断机器人是否在移动
# ============================================================
def step2_check_is_moving(robot: FanucController):
    """
    读取程序状态寄存器 201-218（ADR=200, LEN=18），
    取第 10 个寄存器（偏移10，数组下标9 → 寄存器 210, ADR=209）的执行状态：
      0 = 结束（静止）
      1 = 暂停（静止）
      2 = 执行中（机器人正在运动）
    """
    print("\n" + "=" * 55)
    print("  Step 2: 判断机器人是否在移动")
    print("=" * 55)

    print(f"  [读取] 程序状态  寄存器 {STATUS_BASE_ADDR+1}~{STATUS_BASE_ADDR+STATUS_LEN}"
          f"  (ADR={STATUS_BASE_ADDR}, LEN={STATUS_LEN})")
    status_regs = robot.read(FC=3, ADR=STATUS_BASE_ADDR, LEN=STATUS_LEN)

    print("\n  程序状态寄存器原始值:")
    field_names = (
        ["程序名[1]", "程序名[2]", "程序名[3]", "程序名[4]",
         "程序名[5]", "程序名[6]", "程序名[7]", "程序名[8]",
         "行编号", "执行状态"] +
        [f"母程序名[{i}]" for i in range(1, 9)]
    )
    for i, (val, name) in enumerate(zip(status_regs, field_names)):
        marker = "  ◀ 关键字段" if i == STATUS_EXEC_IDX else ""
        print(f"    寄存器 {STATUS_BASE_ADDR + 1 + i:>3}  ({name:<10}) = {val}{marker}")

    # 执行状态：数组下标 9 → 寄存器 210
    exec_status = status_regs[STATUS_EXEC_IDX]
    is_moving   = (exec_status == STATUS_RUNNING)

    status_desc = {0: "结束（静止）", 1: "暂停（静止）", 2: "执行中（运动中）"}
    desc = status_desc.get(exec_status, f"未知值 {exec_status}")

    print(f"\n  执行状态寄存器 (寄存器 {STATUS_BASE_ADDR + STATUS_EXEC_IDX + 1}, ADR={STATUS_BASE_ADDR + STATUS_EXEC_IDX})")
    print(f"    值 = {exec_status}  →  {desc}")
    print(f"  机器人是否在移动: {'✓ 是（TP 程序执行中）' if is_moving else '✗ 否（静止或暂停）'}")

    return is_moving


# ============================================================
# Step 3: 机器人移动到目标位置（已注释，待安全确认后启用）
# ============================================================
def step3_move_to_target(robot: FanucController, target: list[float]):
    """
    将目标直角坐标写入寄存器 101-112（ADR=100, LEN=12），
    然后发触发信号，TP 程序检测后执行移动。

    ⚠️ 函数体内代码全部注释 —— 确认目标坐标安全后再取消注释！
    """
    print("\n" + "=" * 55)
    print("  Step 3: 机器人移动到目标位置")
    print("=" * 55)

    # ⚠️ 确认目标坐标安全后，取消下方注释 ⚠️
    #
    # target = [X, Y, Z, W, P, R]   ← 填写实际安全坐标
    #
    # print(f"  [写入] 目标坐标  寄存器 {TARGET_BASE_ADDR+1}~{TARGET_BASE_ADDR+12}"
    #       f"  (ADR={TARGET_BASE_ADDR}, LEN=12)")
    # print(f"  目标: X={target[0]:.2f}, Y={target[1]:.2f}, Z={target[2]:.2f}, "
    #       f"W={target[3]:.2f}, P={target[4]:.2f}, R={target[5]:.2f}")
    #
    # robot.move_robot(
    #     target_position=target,
    #     base_addr=TARGET_BASE_ADDR,   # ADR=100 → 寄存器 101
    #     trigger_addr=1,               # 触发寄存器地址（根据 TP 程序配置调整）
    #     trigger_fc=6,                 # FC=6：写单保持寄存器
    # )
    #
    # print("\n  等待机器人运动完成...")
    # for _ in range(60):
    #     time.sleep(0.5)
    #     if not step2_check_is_moving(robot):
    #         print("\n  ✓ 机器人已到达目标位置，运动完成！")
    #         break
    # else:
    #     print("\n  ⚠ 超时：机器人未在预期时间内停止，请检查 TP 程序。")

    print(f"\n  ⚠️  Step 3 已注释，位置安全性确认后再启用。")
    print(f"     预设目标坐标 [X,Y,Z,W,P,R]: {target}")
    print("     → 取消 step3_move_to_target() 内部注释后即可执行移动。")


# ============================================================
# 主程序
# ============================================================
def main():
    print("=" * 55)
    print("   Fanuc Robot Modbus TCP Demo")
    print("=" * 55)
    print(f"  机器人 IP:  {ROBOT_HOST}")
    print(f"  端口:       {ROBOT_PORT}")

    try:
        robot = FanucController(host=ROBOT_HOST, port=ROBOT_PORT, timeout=3.0)
        print(f"\n  ✓ 成功连接到机器人 {ROBOT_HOST}:{ROBOT_PORT}")
    except ConnectionError as e:
        print(f"\n  ✗ 连接失败: {e}", file=sys.stderr)
        print("  请确认：")
        print("    1. 机器人已开机，IP 地址为 192.168.1.10")
        print("    2. Modbus TCP 服务已在示教器中启用（$SNPX_PARAM 或 $MODBUSTCP）")
        print("    3. PC 与机器人在同一网段（192.168.1.x）")
        sys.exit(1)

    try:
        # Step 1: 读取当前位置
        cartesian, joints = step1_read_current_position(robot)

        # Step 2: 判断是否在移动
        is_moving = step2_check_is_moving(robot)

        # Step 3: 移动到目标位置（已注释，待确认安全后启用）
        # ⚠️ 确认以下坐标安全之前，请勿修改 step3 内部的注释！
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
