#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""底盘串口桥（双向）：下行发轮速，上行收编码器回传积分成里程计。

数据流：
  下行：Nav2(velocity_smoother) / 键盘遥控 ──> /cmd_vel(Twist) ──> 差速换算 ──> 串口 ──> 下位机
  上行：下位机(编码器增量) ──────────────> 串口 ──> 积分 ──> /odom + TF(odom→base_footprint)

  下行差速换算（与 vi_car.urdf 的 Gazebo 插件同款公式）：
    v_left  = v − ω·W/2
    v_right = v + ω·W/2          W = 轮距 0.125m
  上行里程计积分（标准差速正解，中点法）：
    ds = (d_right + d_left) / 2          本帧位移
    dθ = (d_right − d_left) / W          本帧转角
    x += ds·cos(θ+dθ/2); y += ds·sin(θ+dθ/2); θ += dθ

为什么收发在同一个节点：串口只有一个读端，两个进程同时读同一 tty 会随机分抢
字节流，所以收发必须同进程；对外它同时是 /cmd_vel 的消费者和 /odom 的生产者。

============================ 串口协议（均为 8 字节，小端） ============================

下发帧（本节点 → 下位机，pyserial 已置 raw 模式，二进制安全）：
  [0]   0xAA  帧头1
  [1]   0x55  帧头2
  [2:4] int16 左轮目标速度，单位 mm/s（负 = 反转/倒退）
  [4:6] int16 右轮目标速度，单位 mm/s
  [6]   uint8 流水号（每帧 +1，下位机可用它判断数据流是否存活）
  [7]   uint8 校验和 = (字节[2..6] 求和) & 0xFF

回传帧（下位机 → 本节点，帧头与下发帧镜像以区分方向）：
  [0]   0x55  帧头1
  [1]   0xAA  帧头2
  [2:4] int16 左轮编码器增量（计数，自上一帧以来，倒退为负）
  [4:6] int16 右轮编码器增量
  [6]   uint8 流水号
  [7]   uint8 校验和 = (字节[2..6] 求和) & 0xFF
  发送节奏建议 ≥30Hz，单帧增量不要超过 ±32767（超了就提高发送频率）。

============================ 安全行为 ============================
  - /cmd_vel 断流超过 cmd_timeout 秒 → 自动发零速（Nav2 退出或卡死时车自己停）
  - 线速度 / 角速度限幅（与 config/nav2/vi_car_nav2_params.yaml 一致）
  - 串口断开自动重连；节点退出前发一帧零速
  - encoder_counts_per_rev 未配置（=0）时忽略回传帧，不发虚假里程计
"""

import math
import struct

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster

import serial

DOWN_HEADER = b'\xaa\x55'   # 下行帧头：本节点 → 下位机
UP_HEADER1 = 0x55           # 上行帧头：下位机 → 本节点（与下行镜像）
UP_HEADER2 = 0xAA
FRAME_LEN = 8


def diff_drive(v, w, wheel_separation):
    """差速运动学逆解：机体 (v, ω) → 左右轮线速度 (m/s)。

    左右各两轮同速（左前=左后，右前=右后），与 Gazebo 插件的两个 joint 对一致。
    """
    half = wheel_separation / 2.0
    return v - w * half, v + w * half


def encode_frame(v_left_ms, v_right_ms, seq):
    """打包下发帧，速度单位 m/s，帧内换算成 mm/s 定点整数。"""
    left_mm = int(round(v_left_ms * 1000.0))
    right_mm = int(round(v_right_ms * 1000.0))
    payload = struct.pack('<hh', left_mm, right_mm) + bytes([seq & 0xFF])
    checksum = sum(payload) & 0xFF
    return DOWN_HEADER + payload + bytes([checksum])


# 占位协方差：x/y 0.1、yaw 0.2，不用的维度给 1e6；
# 每个元素必须显式写 float，ROS 消息校验不接受 int
_POSE_COV = [
    0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 1e6, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 1e6, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 1e6, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.2]
_TWIST_COV = [
    0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 1e6, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 1e6, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 1e6, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.2]


class ChassisSerialNode(Node):

    def __init__(self):
        super().__init__('chassis_serial_node')

        # ---------- 参数（默认值 = vi_car.urdf 车体尺寸 + nav2 限速） ----------
        self.declare_parameter('serial_port', '/dev/ttyUSB1')  # IMU 通常占 ttyUSB0
        self.declare_parameter('serial_baud', 115200)
        self.declare_parameter('publish_rate', 30.0)           # 下发频率，Nav2 控制器 20Hz
        self.declare_parameter('wheel_separation', 0.125)      # 轮距 W (m)，URDF 同款
        self.declare_parameter('wheel_radius', 0.025)          # 轮半径 r (m)
        self.declare_parameter('max_lin_vel', 0.26)            # 与 nav2 max_vel_x 一致
        self.declare_parameter('max_ang_vel', 1.0)             # 与 nav2 max_vel_theta 一致
        self.declare_parameter('cmd_timeout', 0.5)             # /cmd_vel 断流判停时间 (s)
        # 轮轴转一圈对应的编码器计数（编码器线数 × 减速比，按下位机固件的计数口径填）；
        # 0 = 忽略回传帧，不发布里程计
        self.declare_parameter('encoder_counts_per_rev', 0.0)
        # 导航模式(AMCL 写 map→odom)必须 true，本节点补上 odom→base_footprint；
        # 建图模式(cartographer provide_odom_frame=true 自己发 odom TF)必须设 false，避免双写
        self.declare_parameter('publish_odom_tf', True)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')

        self._port = self.get_parameter('serial_port').value
        self._baud = int(self.get_parameter('serial_baud').value)
        rate = float(self.get_parameter('publish_rate').value)
        self._wheel_separation = float(self.get_parameter('wheel_separation').value)
        self._wheel_radius = float(self.get_parameter('wheel_radius').value)
        self._max_lin = float(self.get_parameter('max_lin_vel').value)
        self._max_ang = float(self.get_parameter('max_ang_vel').value)
        self._cmd_timeout = float(self.get_parameter('cmd_timeout').value)
        self._counts_per_rev = float(self.get_parameter('encoder_counts_per_rev').value)
        self._publish_tf = bool(self.get_parameter('publish_odom_tf').value)
        self._odom_frame = self.get_parameter('odom_frame').value
        self._base_frame = self.get_parameter('base_frame').value

        # ---------- 状态 ----------
        self._serial = None
        self._seq = 0
        self._v = 0.0
        self._w = 0.0
        self._last_cmd_ns = None
        self._was_stopped = True
        self._next_open_try_ns = 0
        # 里程计状态
        self._rx_buf = bytearray()
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        self._v_est = 0.0
        self._w_est = 0.0
        self._last_fb_ns = None
        self._have_odom = False
        self._warned_no_encoder = False

        self.create_subscription(Twist, 'cmd_vel', self._on_cmd_vel, 10)
        self._odom_pub = self.create_publisher(Odometry, 'odom', 50)
        self._tf_broadcaster = TransformBroadcaster(self)
        self.create_timer(1.0 / rate, self._tick)

        self.get_logger().info(
            '底盘串口桥启动：W=%.3fm r=%.3fm 限速(%.2f m/s, %.2f rad/s) → %s @%d，'
            '编码器口径=%s' % (
                self._wheel_separation, self._wheel_radius, self._max_lin,
                self._max_ang, self._port, self._baud,
                ('%.0f 计数/圈' % self._counts_per_rev) if self._counts_per_rev > 0 else '未配置(不发里程计)'))

    # ---------- 串口 ----------
    def _ensure_serial(self):
        try:
            self._serial = serial.Serial(
                self._port, self._baud, timeout=0.05, write_timeout=0.05)
        except Exception as e:
            self._serial = None
            self.get_logger().warn(
                '打开串口 %s 失败：%s，2 秒后重试' % (self._port, e),
                throttle_duration_sec=2.0)
            return False
        self.get_logger().info('串口已连接：%s @%d' % (self._port, self._baud))
        return True

    def _close_serial(self):
        try:
            if self._serial is not None:
                self._serial.close()
        except Exception:
            pass
        self._serial = None

    # ---------- /cmd_vel 回调 ----------
    def _on_cmd_vel(self, msg):
        # 差速底盘只用 linear.x 与 angular.z，linear.y 恒为 0（差速车不会横移）
        self._v = msg.linear.x
        self._w = msg.angular.z
        self._last_cmd_ns = self.get_clock().now().nanoseconds

    # ---------- 上行：编码器回传 ----------
    def _read_and_parse(self):
        """把串口收到的字节喂进帧解析器，收到完整回传帧就积分一次里程计。"""
        n = self._serial.in_waiting
        if n > 0:
            self._rx_buf.extend(self._serial.read(n))
        while len(self._rx_buf) >= FRAME_LEN:
            if self._rx_buf[0] != UP_HEADER1 or self._rx_buf[1] != UP_HEADER2:
                del self._rx_buf[0]          # 没对上帧头，滑动一字节重新找
                continue
            frame = bytes(self._rx_buf[:FRAME_LEN])
            if (sum(frame[2:7]) & 0xFF) != frame[7]:
                del self._rx_buf[0]          # 校验失败的坏帧，丢弃后重新找头
                self.get_logger().warn('回传帧校验失败，丢弃', throttle_duration_sec=2.0)
                continue
            dl, dr = struct.unpack('<hh', frame[2:6])
            del self._rx_buf[:FRAME_LEN]
            self._integrate_odom(dl, dr)

    def _integrate_odom(self, dl, dr):
        if self._counts_per_rev <= 0.0:
            if not self._warned_no_encoder:
                self.get_logger().warn(
                    '收到回传帧，但 encoder_counts_per_rev=0（未配置编码器口径），忽略里程计')
                self._warned_no_encoder = True
            return
        m_per_count = 2.0 * math.pi * self._wheel_radius / self._counts_per_rev
        d_left = dl * m_per_count
        d_right = dr * m_per_count
        ds = (d_right + d_left) / 2.0
        dth = (d_right - d_left) / self._wheel_separation

        now_ns = self.get_clock().now().nanoseconds
        if self._last_fb_ns is not None:
            dt = (now_ns - self._last_fb_ns) / 1e9
            if dt > 1e-3:
                self._v_est = ds / dt
                self._w_est = dth / dt
        self._last_fb_ns = now_ns

        # 中点法积分（比欧拉法在转弯时精度高一档）
        self._x += ds * math.cos(self._yaw + dth / 2.0)
        self._y += ds * math.sin(self._yaw + dth / 2.0)
        self._yaw = (self._yaw + dth + math.pi) % (2.0 * math.pi) - math.pi
        self._have_odom = True

    def _publish_odom(self, now):
        qz = math.sin(self._yaw / 2.0)
        qw = math.cos(self._yaw / 2.0)
        stamp = now.to_msg()

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self._odom_frame
        odom.child_frame_id = self._base_frame
        odom.pose.pose.position.x = self._x
        odom.pose.pose.position.y = self._y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.pose.covariance = list(_POSE_COV)
        odom.twist.twist.linear.x = self._v_est
        odom.twist.twist.angular.z = self._w_est
        odom.twist.covariance = list(_TWIST_COV)
        self._odom_pub.publish(odom)

        if self._publish_tf:
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = self._odom_frame
            t.child_frame_id = self._base_frame
            t.transform.translation.x = self._x
            t.transform.translation.y = self._y
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            self._tf_broadcaster.sendTransform(t)

    # ---------- 主循环 ----------
    def _tick(self):
        # 串口未连接时每 2 秒尝试一次（设备未插/被拔自动重连）
        if self._serial is None:
            now_ns = self.get_clock().now().nanoseconds
            if now_ns < self._next_open_try_ns:
                return
            self._next_open_try_ns = now_ns + 2_000_000_000
            if not self._ensure_serial():
                return

        # 先收：编码器回传 → 里程计
        try:
            self._read_and_parse()
        except Exception as e:
            self.get_logger().warn(
                '串口读取失败：%s，尝试重连' % str(e), throttle_duration_sec=2.0)
            self._close_serial()
            return

        # 断流保护：超过 cmd_timeout 没收到 /cmd_vel 就发零速
        now_ns = self.get_clock().now().nanoseconds
        if self._last_cmd_ns is None or now_ns - self._last_cmd_ns > self._cmd_timeout * 1e9:
            v, w = 0.0, 0.0
            if not self._was_stopped:
                self.get_logger().warn('/cmd_vel 断流超过 %.1fs，自动停车' % self._cmd_timeout)
            self._was_stopped = True
        else:
            v, w = self._v, self._w
            self._was_stopped = False

        # 限幅（与 nav2 参数一致，双保险；真正的执行限速以下位机电机能力为准）
        v = max(-self._max_lin, min(self._max_lin, v))
        w = max(-self._max_ang, min(self._max_ang, w))

        v_left, v_right = diff_drive(v, w, self._wheel_separation)
        frame = encode_frame(v_left, v_right, self._seq)
        self._seq = (self._seq + 1) & 0xFF

        try:
            self._serial.write(frame)
        except Exception as e:
            self.get_logger().warn(
                '串口写入失败：%s，尝试重连' % str(e), throttle_duration_sec=2.0)
            self._close_serial()
            return

        # 后发：有里程计数据就发布（/odom + TF 一起）
        if self._have_odom:
            self._publish_odom(self.get_clock().now())

    def destroy_node(self):
        # 退出前发一帧零速，避免下位机保持最后速度
        if self._serial is not None:
            try:
                v_left, v_right = diff_drive(0.0, 0.0, self._wheel_separation)
                self._serial.write(encode_frame(v_left, v_right, self._seq))
            except Exception:
                pass
        self._close_serial()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ChassisSerialNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
