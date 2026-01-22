#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import serial
import struct
import math

HEADER = 0x54
VERLEN = 0x2C
POINTS_PER_PACK = 12

def calc_crc8(data):
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x80:
                crc = (crc << 1) ^ 0x31
            else:
                crc <<= 1
            crc &= 0xFF
    return crc

class LD20Packet:
    def __init__(self):
        self.distance = []
        self.confidence = []
        self.angle = []
        self.start_angle = 0
        self.end_angle = 0
        self.speed = 0
        self.timestamp = 0

def read_packet(ser):
    while True:
        b = ser.read(1)
        if not b:
            return None
        if b[0] == HEADER:
            break
    
    ver = ser.read(1)
    if not ver or ver[0] != VERLEN:
        return None
    
    raw = ser.read(45)
    if len(raw) != 45:
        return None
    
    pkt = LD20Packet()
    idx = 0
    
    pkt.speed = struct.unpack('<H', raw[idx:idx + 2])[0]
    idx += 2
    
    pkt.start_angle = struct.unpack('<H', raw[idx:idx + 2])[0] / 100.0
    idx += 2
    
    for _ in range(POINTS_PER_PACK):
        dist_mm = struct.unpack('<H', raw[idx:idx + 2])[0]
        conf = raw[idx + 2]
        idx += 3
        pkt.distance.append(dist_mm / 1000.0)
        pkt.confidence.append(conf)
    
    pkt.end_angle = struct.unpack('<H', raw[idx:idx + 2])[0] / 100.0
    idx += 2
    
    pkt.timestamp = struct.unpack('<H', raw[idx:idx + 2])[0]
    idx += 2
    
    if pkt.end_angle >= pkt.start_angle:
        step = (pkt.end_angle - pkt.start_angle) / POINTS_PER_PACK
    else:
        step = (pkt.end_angle + 360 - pkt.start_angle) / POINTS_PER_PACK
    
    for i in range(POINTS_PER_PACK):
        angle_deg = pkt.start_angle + i * step
        pkt.angle.append(math.radians(angle_deg))
    
    return pkt

class LD20Node(Node):
    def __init__(self):
        super().__init__('ld20_lidar_node')
        self.publisher = self.create_publisher(LaserScan, 'scan', 10)
        
        self.ser = serial.Serial('/dev/ttyUSB0', 230400, timeout=1)
        
        # Accumulate a full 360 scan
        self.scan_data = {}
        self.last_angle = None
        
        self.timer = self.create_timer(0.001, self.read_lidar)
        
    def read_lidar(self):
        pkt = read_packet(self.ser)
        if not pkt:
            return
        
        # Add points to accumulator
        for dist, angle, conf in zip(pkt.distance, pkt.angle, pkt.confidence):
            if conf > 100:  # Filter low confidence
                angle_normalized = (angle + 2*math.pi) % (2*math.pi)
                self.scan_data[angle_normalized] = dist
        
        # Detect full rotation (when angle wraps around)
        current_angle = pkt.end_angle
        if self.last_angle is not None:
            if current_angle < self.last_angle - 180:  # Wrapped around
                self.publish_scan()
                self.scan_data = {}
        
        self.last_angle = current_angle
    
    def publish_scan(self):
        if len(self.scan_data) < 100:  # Need enough points
            return
        
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'laser'
        
        scan.angle_min = 0.0
        scan.angle_max = 2 * math.pi
        scan.angle_increment = (2 * math.pi) / 360
        scan.time_increment = 0.0
        scan.scan_time = 0.1
        scan.range_min = 0.02
        scan.range_max = 12.0
        
        # Create ranges array
        ranges = [float('inf')] * 360
        for angle, dist in self.scan_data.items():
            idx = int((angle / (2 * math.pi)) * 360) % 360
            if 0.02 < dist < 12.0:
                ranges[idx] = dist
        
        scan.ranges = ranges
        self.publisher.publish(scan)
        self.get_logger().info(f'Published scan with {len(self.scan_data)} points')

def main(args=None):
    rclpy.init(args=args)
    node = LD20Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
