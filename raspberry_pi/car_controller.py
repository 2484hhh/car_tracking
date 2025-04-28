import serial
import time
import json

class CarController:
    def __init__(self, port='/dev/ttyUSB0', baudrate=9600):
        """初始化串口"""
        self.ser = serial.Serial(port, baudrate, timeout=1)
        time.sleep(2)  # 给Arduino一点时间重启

    def send_command(self, cmd, data=None):
        """发送结构化指令给Arduino，支持单字符和JSON格式"""
        if self.ser.is_open:
            if data is not None:
                msg = json.dumps({"cmd": cmd, "data": data}) + '\n'
                self.ser.write(msg.encode('utf-8'))
                print(f"发送JSON指令: {msg.strip()}")
            else:
                self.ser.write(cmd.encode('utf-8'))
                print(f"发送指令: {cmd}")
        else:
            print("串口未打开")

    def read_feedback(self):
        """读取Arduino反馈"""
        if self.ser.is_open and self.ser.in_waiting:
            try:
                line = self.ser.readline().decode('utf-8').strip()
                if line:
                    print(f"收到反馈: {line}")
                    return line
            except Exception as e:
                print(f"读取反馈出错: {e}")
        return None

    def close(self):
        """关闭串口"""
        if self.ser.is_open:
            self.ser.close()
