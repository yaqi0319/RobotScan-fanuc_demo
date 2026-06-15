import socket
import struct
from array import array
from math import ceil
from struct import pack, unpack
from modbus.client import client

def registers_to_float(reg1: int, reg2: int, swap_words: bool = False) -> float:
    """
    将两个16位Modbus寄存器转换为一个32位浮点数 (IEEE 754)。
    
    参数:
        reg1 (int): 第一个16位寄存器值。
        reg2 (int): 第二个16位寄存器值。
        swap_words (bool): 如果为 True，则交换寄存器的顺序 (reg2, reg1)。
                           默认位 False (高位在前/大端字序)。
    """
    if swap_words:
        data = struct.pack('>HH', reg2, reg1)
    else:
        data = struct.pack('>HH', reg1, reg2)
    return struct.unpack('>f', data)[0]

def float_to_registers(val: float, swap_words: bool = False) -> tuple[int, int]:
    """
    将一个32位浮点数转换为两个16位Modbus寄存器值。
    
    参数:
        val (float): 需要转换的浮点数值。
        swap_words (bool): 如果为 True，则交换生成的寄存器顺序。
                           默认为 False。
    """
    data = struct.pack('>f', val)
    reg1, reg2 = struct.unpack('>HH', data)
    if swap_words:
        return reg2, reg1
    return reg1, reg2


class FanucController(client):
    """
    FanucController 是一个通过 Modbus TCP 与 Fanuc 机械臂进行通信的控制类。
    
    它封装了底层的 Modbus 客户端，并添加了：
    1. 支持自定义端口 (例如 502/503/504) 与网络超时配置。
    2. 网络连接断开时的自动重连和重试逻辑。
    3. 支持 32 位浮点数（PR[] 位置寄存器）在 Modbus 寄存器中的打包与解析。
    4. 实现了读取当前位置、下发目标位置控制移动、查询移动状态的完整逻辑。
    """
    
    def __init__(self, host: str = '192.168.1.10', port: int = 502, unit: int = 1, timeout: float = 3.0):
        """
        初始化 Fanuc Modbus TCP 控制器。
        
        参数:
            host (str): 机械臂控制器的 IP 地址。
            port (int): Modbus TCP 端口号（默认为 502，可以在示教器 $MODBUSTCP.$PORT 或 $SNPX_PARAM.$PORT 中设置）。
            unit (int): Modbus 单元 ID（从站号，默认为 1）。
            timeout (float): 建立连接与发送/接收的超时时间（秒）。
        """
        self.host = host
        self.port = port
        self.unit = unit
        self.timeout = timeout
        self.TID = 0
        self.sock = None
        self.connect()

    def connect(self) -> bool:
        """
        建立与 Fanuc 机械臂 Modbus TCP 服务端的连接。
        """
        self.disconnect()
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(self.timeout)
            self.sock.connect((self.host, self.port))
            return True
        except Exception as e:
            self.sock = None
            raise ConnectionError(f"无法连接到 Fanuc 控制器 {self.host}:{self.port} - 错误信息: {e}")

    def disconnect(self):
        """
        断开与 Fanuc 机械臂的连接，并释放套接字资源。
        """
        if self.sock is not None:
            try:
                self.sock.close()
            except Exception:
                pass
            self.sock = None

    @property
    def is_connected(self) -> bool:
        """
        获取当前连接状态。
        """
        return self.sock is not None

    def read(self, FC: int = 4, ADR: int = 0, LEN: int = 10):
        """
        读取寄存器，当套接字异常断开时会自动尝试重连并重试一次。
        
        参数:
            FC (int): 功能码 (1=读线圈, 2=读离散输入, 3=读保持寄存器, 4=读输入寄存器)
            ADR (int): 起始地址
            LEN (int): 读取长度
        """
        if self.sock is None:
            self.connect()
        try:
            return super().read(FC=FC, ADR=ADR, LEN=LEN)
        except (socket.error, OSError) as e:
            # 尝试自动重连并重试
            try:
                self.connect()
                return super().read(FC=FC, ADR=ADR, LEN=LEN)
            except Exception as ex:
                raise ConnectionError(f"Modbus 读取失败且重连尝试失败: {ex}") from e

    def write(self, *DAT: int, FC: int = 16, ADR: int = 0):
        """
        写入寄存器，当套接字异常断开时会自动尝试重连并重试一次。
        
        参数:
            *DAT (int): 写入的数据序列
            FC (int): 功能码 (5=写单线圈, 6=写单寄存器, 15=写多线圈, 16=写多寄存器)
            ADR (int): 起始地址
        """
        if self.sock is None:
            self.connect()
        try:
            return super().write(*DAT, FC=FC, ADR=ADR)
        except (socket.error, OSError) as e:
            # 尝试自动重连并重试
            try:
                self.connect()
                return super().write(*DAT, FC=FC, ADR=ADR)
            except Exception as ex:
                raise ConnectionError(f"Modbus 写入失败且重连尝试失败: {ex}") from e

    # ==================== 1. 读取机械臂位置 ====================
    def read_position(self, base_addr: int = 1000, fc: int = 4, swap_words: bool = False) -> list[float]:
        """
        读取机械臂当前的直角坐标位置 (X, Y, Z, W, P, R)。
        
        说明:
            由于 Modbus 寄存器为16位，而机械臂位置坐标为32位单精度浮点数 (IEEE 754)，
            因此每个坐标轴占用2个寄存器，总计读取12个寄存器。
            
        参数:
            base_addr (int): 示教器配置的用于存放当前坐标的起始 Modbus 寄存器地址（默认设为 1000）。
            fc (int): 功能码。3 = 读保持寄存器(Holding Registers), 4 = 读输入寄存器(Input Registers)。默认 4。
            swap_words (bool): 是否交换高低字序（有些客户端/PLC要求低16位在前，则设为 True）。
            
        返回:
            list of float: [X, Y, Z, W, P, R] 机械臂直角坐标数据。
        """
        regs = self.read(FC=fc, ADR=base_addr, LEN=12)
        if not regs or len(regs) < 12:
            raise ValueError(f"读取位置数据失败：预期读取 12 个寄存器，实际仅获取到 {len(regs) if regs else 0} 个")
            
        position = []
        for i in range(6):
            r1 = regs[i * 2]
            r2 = regs[i * 2 + 1]
            position.append(registers_to_float(r1, r2, swap_words=swap_words))
        return position

    def read_joint_position(self, base_addr: int = 1020, fc: int = 4, swap_words: bool = False) -> list[float]:
        """
        读取机械臂当前的关节轴位置 (J1, J2, J3, J4, J5, J6)。
        
        参数:
            base_addr (int): 关节坐标的起始 Modbus 寄存器地址（默认设为 1020）。
            fc (int): 功能码（3=保持寄存器, 4=输入寄存器）。
            swap_words (bool): 是否交换高低字序。
            
        返回:
            list of float: [J1, J2, J3, J4, J5, J6] 关节角度/位移。
        """
        regs = self.read(FC=fc, ADR=base_addr, LEN=12)
        if not regs or len(regs) < 12:
            raise ValueError(f"读取关节位置失败：预期读取 12 个寄存器，实际仅获取到 {len(regs) if regs else 0} 个")
            
        joints = []
        for i in range(6):
            r1 = regs[i * 2]
            r2 = regs[i * 2 + 1]
            joints.append(registers_to_float(r1, r2, swap_words=swap_words))
        return joints

    # ==================== 2. 移动机械臂 ====================
    def move_robot(self, target_position: list[float], base_addr: int = 2000, trigger_addr: int = 1, trigger_fc: int = 6, swap_words: bool = False) -> bool:
        """
        下发目标位置数据并触发机械臂移动。
        
        注意:
            要使机械臂实际运动，必须在 Fanuc 控制器上运行一个 Teach Pendant (TP) 程序或 Karel 程序，
            在后台循环监测 Modbus 寄存器。一旦触发信号置位，TP程序将读取目标寄存器值并赋给位置寄存器 (PR[])，
            然后通过运动指令 (如 L PR[x] 2000mm/min FINE) 执行移动，完成后复位触发信号。
            
        参数:
            target_position (list of float): 目标直角坐标位置 [X, Y, Z, W, P, R]。
            base_addr (int): 下发目标坐标的起始 Modbus 寄存器地址（需要写入 12 个寄存器，默认设为 2000）。
            trigger_addr (int): 触发移动的控制信号地址。
            trigger_fc (int): 触发写入的功能码。5 = 写线圈(Coil), 6 = 写单寄存器(Holding Register)。默认为 6。
            swap_words (bool): 下发的目标位置浮点数是否交换高低字序。
            
        返回:
            bool: 写入指令是否发送成功。
        """
        if len(target_position) != 6:
            raise ValueError("target_position 必须包含且仅包含 6 个元素 [X, Y, Z, W, P, R]")
            
        # 1. 将 6 个浮点数转换为 12 个 16 位整数寄存器值
        regs = []
        for val in target_position:
            r1, r2 = float_to_registers(val, swap_words=swap_words)
            regs.extend([r1, r2])
            
        # 2. 写入目标位置坐标到保持寄存器 (使用功能码 16: 写多个保持寄存器)
        self.write(*regs, FC=16, ADR=base_addr)
        
        # 3. 发送运动触发指令
        if trigger_fc == 5:
            # 0xFF00 为线圈置 ON (True) 的标准 Modbus 值，0x0000 为 OFF
            self.write(0xFF00, FC=5, ADR=trigger_addr)
        elif trigger_fc == 6:
            # 寄存器写入 1 作为触发信号
            self.write(1, FC=6, ADR=trigger_addr)
        else:
            raise ValueError("trigger_fc 参数必须为 5 (线圈) 或 6 (保持寄存器)")
            
        return True

    def clear_move_trigger(self, trigger_addr: int = 1, trigger_fc: int = 6) -> bool:
        """
        复位移动触发指令，便于下一次写入目标时再次触发。
        """
        if trigger_fc == 5:
            self.write(0x0000, FC=5, ADR=trigger_addr)
        elif trigger_fc == 6:
            self.write(0, FC=6, ADR=trigger_addr)
        else:
            raise ValueError("trigger_fc 参数必须为 5 (线圈) 或 6 (保持寄存器)")
        return True

    # ==================== 3. 机械臂是否在移动 ====================
    def is_moving(self, status_addr: int = 100, fc: int = 2) -> bool:
        """
        查询机械臂当前是否正在运动。
        
        注意:
            需要在 Fanuc 示教器或 PMC 中将反映运行状态的系统变量（如 $MOR_GRP[1].$ROB_MOVE）
            分配映射到对应的 Modbus IO，这里才可以进行读取。
            
        参数:
            status_addr (int): 状态位在 Modbus 中的地址（默认设为 100）。
            fc (int): 功能码。1 = 读线圈(Coil), 2 = 读离散输入(Discrete Input), 
                                3 = 读保持寄存器(Holding Register), 4 = 读输入寄存器(Input Register)。默认 2。
            
        返回:
            bool: True 表示机械臂正在移动，False 表示静止。
        """
        regs = self.read(FC=fc, ADR=status_addr, LEN=1)
        if not regs or len(regs) < 1:
            raise ValueError(f"读取运动状态失败：未能从地址 {status_addr} 获取数据")
            
        if fc in [1, 2]:
            # 读线圈/离散输入，底层库返回字节。通过判断字节第0位获取线圈状态。
            return bool(regs[0] & 1)
        else:
            # 读保持/输入寄存器，返回16位整型。0 表示静止，非0 表示正在运动。
            return bool(regs[0])
