# Fanuc Demo
这是一个python控制Fanuc机械臂的demo，使用Modbus TCP进行通信

## modbus协议

完整协议文档见 [doc/modbus_protocol.md](doc/modbus_protocol.md)。

### 寄存器地址总览

| 寄存器范围 | 用途 |
|---|---|
| 1 – 50 | 实际位置 POS[0]（当前直角坐标 + 关节角度） |
| 101 – 150 | 目标位置 PR[1]（写入目标坐标以触发移动） |
| 201 – 218 | 程序状态（18个寄存器/任务） |

### 数据格式说明

- 坐标值使用 **32位有符号整数**，单位为 **0.001 mm / 0.001 deg**（即实际值 × 1000 存入寄存器）
- 每个坐标值占用 **2个相邻的16位寄存器**，字节序为 **低位字在前（Word Swapped）**
- 机器人 IP：`192.168.1.10`，端口：`502`

### 位置寄存器结构（POS[0] / PR[1]）

**直角坐标（寄存器 1–12，ADR=0，LEN=12）**

| 偏移地址 | 字段 | 单位 |
|---|---|---|
| 1-2 | X | mm |
| 3-4 | Y | mm |
| 5-6 | Z | mm |
| 7-8 | W | deg |
| 9-10 | P | deg |
| 11-12 | R | deg |

**关节角度（寄存器 27–38，ADR=26，LEN=12）**

| 偏移地址 | 字段 | 单位 |
|---|---|---|
| 27-28 | J1 | deg |
| 29-30 | J2 | deg |
| 31-32 | J3 | deg |
| 33-34 | J4 | deg |
| 35-36 | J5 | deg |
| 37-38 | J6 | deg |

### 程序状态寄存器（寄存器 201–218，ADR=200，LEN=18）

| 偏移地址 | 字段 | 说明 |
|---|---|---|
| 1-8 | 程序名 | 字符串，最多16字符 |
| 9 | 行编号 | 当前执行行号 |
| 10 | 执行状态 | **0=结束，1=暂停，2=执行中** |
| 11-18 | 母程序名 | 最初启动的程序名 |

---

## 快速开始

### 1. 克隆代码

```bash
git clone https://github.com/yaqi0319/RobotScan-fanuc_demo.git
cd RobotScan-fanuc_demo
```

### 2. 安装 uv

```bash
# macOS / Linux
curl -LsSf https://astral.sh/uv/install.sh | sh

# Windows (PowerShell)
powershell -ExecutionPolicy ByPass -c "irm https://astral.sh/uv/install.ps1 | iex"
```

### 3. 同步依赖

```bash
uv sync
```

### 4. 运行 Demo

```bash
uv run demo.py
```

> **运行前请确认：**
> 1. 机器人已开机，IP 地址为 `192.168.1.10`
> 2. 示教器中已启用 Modbus TCP 服务（`$SNPX_PARAM` 或 `$MODBUSTCP`）
> 3. PC 与机器人在同一网段（`192.168.1.x`）