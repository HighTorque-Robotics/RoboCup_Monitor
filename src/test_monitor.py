import socket
import time
import math
import ctypes
import struct

# ================= 配置区域 =================

UDP_IP = "255.255.255.255" 


UDP_PORT = 57335 

ROBOT_ID = 1  # 模拟 1 号机器人
# ===========================================

class Point3D(ctypes.Structure):
    _fields_ = [
        ("x", ctypes.c_double),
        ("y", ctypes.c_double),
        ("z", ctypes.c_double),
    ]

class TeamInfo(ctypes.Structure):

    _fields_ = [
        # --- 0~16 bytes: TimePoints (int64) ---
        ("txp_timestamp", ctypes.c_longlong),  # 8 bytes
        ("recv_timestamp", ctypes.c_longlong), # 8 bytes
        
        # --- 16~20 bytes: General ---
        ("player_number", ctypes.c_uint8),
        ("team_number", ctypes.c_uint8),
        ("test", ctypes.c_uint8),
        ("incapacitated", ctypes.c_bool),

        # --- 20~22 bytes: Behavior Start ---
        ("role", ctypes.c_uint8),
        ("attack_right", ctypes.c_bool),
        
        
        # --- 24~96 bytes: Vectors ---
        ("dest", Point3D),          # Offset 24
        ("final_dest", Point3D),
        ("attack_target", Point3D),
        
        # --- 96~100 bytes ---
        ("time_since_last_kick", ctypes.c_float),
        
        # --- 100~112 bytes: State & Mates ---
        ("state", ctypes.c_uint8),  # Offset 100
        ("priority", ctypes.c_bool),
        ("kicker_id", ctypes.c_uint8),
        ("mates_online", ctypes.c_bool * 6), # 6 bytes
        ("see_ball", ctypes.c_bool),
        ("see_circle", ctypes.c_bool),
        ("see_goal", ctypes.c_bool),
        
        # --- 112~280 bytes: Vision Vectors (7个 Point3D) ---
        ("robot_pos", Point3D),     # Offset 112
        ("ball_field", Point3D),
        ("ball_global", Point3D),
        ("circle_field", Point3D),
        ("circle_global", Point3D),
        ("goal_field", Point3D)
        ("goal_global", Point3D),
        
        # --- 280~292 bytes: Quality ---
        ("ball_quality", ctypes.c_float),
        ("field_quality", ctypes.c_float),
        ("field_consistency", ctypes.c_float),
        
        # --- 292~295 bytes: GC Info ---
        ("gc_connected", ctypes.c_bool),
        ("gc_state", ctypes.c_uint8),
        ("gc_state2", ctypes.c_uint8),
        
    ]

def send_mock_data():
    # 1. 验证大小是否匹配 C++ 输出
    actual_size = ctypes.sizeof(TeamInfo)
    print(f"Python计算的结构体大小: {actual_size} 字节")
    if actual_size != 296:
        print("❌ 错误: 大小不匹配！C++是296。请检查字段定义。")
        return
    else:
        print("✅ 大小匹配 (296字节)。准备发送...")

    # 2. 创建 UDP Socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

    print(f"目标 IP: {UDP_IP}")
    print(f"目标端口: {UDP_PORT} (请确保与 C++ dconstant.hpp 一致)")
    print("正在发送... (按 Ctrl+C 停止)")

    info = TeamInfo()
    # 填充基础信息
    info.player_number = ROBOT_ID
    info.team_number = 1
    info.role = 1 # STRIKER
    info.state = 1 # PLAYING
    info.gc_state = 3 # PLAYING
    info.gc_connected = True
    info.see_ball = True 
    info.see_circle = True
    
    t = 0.0
    try:
        while True:
            # === 模拟运动逻辑 ===
            # 机器人画圆
            info.robot_pos.x = 100.0 * math.cos(t)
            info.robot_pos.y = 100.0 * math.sin(t)
            info.robot_pos.z = (t * 180 / math.pi) % 360 # 朝向
            
            # 球在机器人前面一点
            info.ball_global.x = info.robot_pos.x + 0.5
            info.ball_global.y = info.robot_pos.y + 0.5
            
            # 更新时间戳 (纳秒)
            now_ns = int(time.time() * 1e9)
            info.txp_timestamp = now_ns
            info.recv_timestamp = now_ns

            # === 发送 ===
            data = bytes(info)
            sock.sendto(data, (UDP_IP, UDP_PORT))
            
            time.sleep(0.033) # 30Hz
            t += 0.05

    except KeyboardInterrupt:
        print("\n已停止。")
        sock.close()

if __name__ == "__main__":
    send_mock_data()