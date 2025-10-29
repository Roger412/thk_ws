
import sys
import threading

import geometry_msgs.msg
import rcl_interfaces.msg
import rclpy
import math

from time import time

import select
import termios
import tty

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from rclpy.parameter import Parameter
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry

msg = """
This node takes keypresses from the keyboard and publishes them
as Twist/TwistStamped messages. It works best with a US keyboard layout.
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >

t : up (+z)
b : down (-z)

p : toggle control mode

anything else : stop

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

CTRL-C to quit
"""

moveBindings = {
    'i': (1, 0, 0, 0),
    'o': (1, 0, 0, -1),
    'j': (0, 0, 0, 1),
    'l': (0, 0, 0, -1),
    'u': (1, 0, 0, 1),
    ',': (-1, 0, 0, 0),
    '.': (-1, 0, 0, 1),
    'm': (-1, 0, 0, -1),
    'O': (1, -1, 0, 0),
    'I': (1, 0, 0, 0),
    'J': (0, 1, 0, 0),
    'L': (0, -1, 0, 0),
    'U': (1, 1, 0, 0),
    '<': (-1, 0, 0, 0),
    '>': (-1, -1, 0, 0),
    'M': (-1, 1, 0, 0),
    't': (0, 0, 1, 0),
    'b': (0, 0, -1, 0),
}

speedBindings = {
    'q': (1.1, 1.1),
    'z': (.9, .9),
    'w': (1.1, 1),
    'x': (.9, 1),
    'e': (1, 1.1),
    'c': (1, .9),
}

def getKey(settings, timeout=0.05): # Set timeout to # seconds to make it non-blocking
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    key = sys.stdin.read(1) if rlist else ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def saveTerminalSettings():
    return termios.tcgetattr(sys.stdin)


def restoreTerminalSettings(old_settings):
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


def vels(speed, turn):
    return 'currently:\tspeed %.2f\tturn %.2f ' % (speed, turn)

class TeleopTwistKeyboardNode(Node):
	def __init__(self):
		super().__init__('teleop_twist_keyboard')

		# ───── Parameters ─────
		self.declare_parameter('stamped', False)
		self.declare_parameter('frame_id', '')

		desc_speed = ParameterDescriptor(description="Linear speed scaling factor")
		desc_turn  = ParameterDescriptor(description="Angular speed scaling factor")
		desc_mode  = ParameterDescriptor(description="Select control mode: 'keyboard' or 'auto'")
		desc_pos   = ParameterDescriptor(description="Target position (x/y)")
		desc_head  = ParameterDescriptor(description="Target heading [rad]")

		self.declare_parameter('speed', 0.5, desc_speed)
		self.declare_parameter('turn', 1.0, desc_turn)
		self.declare_parameter('control_mode', 'keyboard', desc_mode)
		self.declare_parameter('x_pos', 0.0, desc_pos)
		self.declare_parameter('y_pos', 0.0, desc_pos)
		self.declare_parameter('heading', 0.0, desc_head)


		# ───── Internal vars ─────
		self.odom_data = {"x": 0.0, "y": 0.0, "heading": 0.0}

		# Choose message type
		self.stamped = self.get_parameter('stamped').value
		self.frame_id = self.get_parameter('frame_id').value
		self.TwistMsg = TwistStamped if self.stamped else Twist

		# Publisher
		self.pub = self.create_publisher(self.TwistMsg, 'cmd_vel', 10)

		# Subscriber
		self.create_subscription(Odometry, '/odom/unfiltered', self.odom_callback, 10)

		# Parameter callback
		self.add_on_set_parameters_callback(self.param_callback)

		# Terminal setup
		self.settings = termios.tcgetattr(sys.stdin)

		self.get_logger().info("TeleopTwistKeyboardNode initialized.")

	# ───── Odometry callback ─────
	def odom_callback(self, msg):
		self.odom_data["x"] = msg.pose.pose.position.x
		self.odom_data["y"] = msg.pose.pose.position.y
		q = msg.pose.pose.orientation
		siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
		cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
		self.odom_data["heading"] = math.atan2(siny_cosp, cosy_cosp)

	# ───── Parameter change callback ─────
	def param_callback(self, params):
		for param in params:
			if param.name == 'speed' and param.type_ == param.Type.DOUBLE:
				self.get_logger().info(f"Speed changed to {param.value}")
			elif param.name == 'turn' and param.type_ == param.Type.DOUBLE:
				self.get_logger().info(f"Turn changed to {param.value}")
			elif param.name == 'control_mode':
				self.get_logger().info(f"Control mode set to {param.value}")
			elif param.name in ['x_pos', 'y_pos', 'heading']:
				self.get_logger().info(f"{param.name} updated → {param.value}")
		return SetParametersResult(successful=True)


def main():
	settings = saveTerminalSettings()

	rclpy.init()
	node = TeleopTwistKeyboardNode()

	spinner = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
	spinner.start()
	        
	x = 0.0
	y = 0.0
	z = 0.0
	th = 0.0
	status = 0.0


	last_key_time = time()
	stop_timeout = 0.2  # seconds of inactivity before auto-stop
	prev_mode = None

	try:		
		while rclpy.ok():	
			
			twist_msg = node.TwistMsg()

			if node.stamped:
				twist_msg.header.stamp = node.get_clock().now().to_msg()
				twist_msg.header.frame_id = node.frame_id
				twist = twist_msg.twist
			else:
				twist = twist_msg

			key = getKey(settings, timeout=0.05)  # non-blocking read every 50ms

			if key == 'p':
				current_mode = node.get_parameter('control_mode').value
				new_mode = 'auto' if current_mode == 'keyboard' else 'keyboard'
				node.set_parameters([
					Parameter(name='control_mode', value=new_mode)
				])
				print(f"\n🟢 Control mode toggled → {new_mode.upper()}")
				# safety stop
				x = y = z = th = 0.0
				twist.linear.x = twist.linear.y = 0.0
				twist.angular.z = 0.0
				node.pub.publish(twist_msg)
				continue  # skip rest of loop this iteration

			# get control mode after possible toggle
			control_mode = node.get_parameter('control_mode').value

			# Print intro only when switching into keyboard mode
			if control_mode == 'keyboard' and prev_mode != 'keyboard':
				print(msg)
				speed = node.get_parameter('speed').value
				turn = node.get_parameter('turn').value
				print(vels(speed, turn))

			# Update previous mode for next iteration
			prev_mode = control_mode

			if control_mode == 'keyboard':
				if key in moveBindings.keys():
					x = moveBindings[key][0]
					y = moveBindings[key][1]
					z = moveBindings[key][2]
					th = moveBindings[key][3]
					last_key_time = time()  # movement key pressed
				elif key in speedBindings.keys():
					speed = speed * speedBindings[key][0]
					turn = turn * speedBindings[key][1]

					print(vels(speed, turn))
					if (status == 14):
						print(msg)
					status = (status + 1) % 15

					node.set_parameters([
						Parameter(name='speed', value=speed),
						Parameter(name='turn',  value=turn)
					])
				else:
					x = 0.0
					y = 0.0
					z = 0.0
					th = 0.0
					if (key == '\x03'):
						break

				if (time() - last_key_time > stop_timeout):
					x = y = z = th = 0.0

				speed = node.get_parameter('speed').value
				turn = node.get_parameter('turn').value
				twist.linear.x = x * speed
				twist.linear.y = y * speed
				twist.linear.z = z * speed
				twist.angular.x = 0.0
				twist.angular.y = 0.0
				twist.angular.z = th * turn

                                   
			elif control_mode == 'auto':
				# ───── Autonomous pose control (PI control) ─────
				x_target = node.get_parameter('x_pos').value
				y_target = node.get_parameter('y_pos').value
				heading_target = node.get_parameter('heading').value

				x_curr = node.odom_data["x"]
				y_curr = node.odom_data["y"]
				heading_curr = node.odom_data["heading"]

				# Control gains
				k_lin_p = 0.1
				k_lin_i = 0.02     # integral gain (tune carefully)
				k_ang_p = 0.1
				k_ang_i = 0.02

				# position error
				dx = x_target - x_curr
				dy = y_target - y_curr
				dist = math.sqrt(dx**2 + dy**2)

				# heading error (normalized to [-pi, pi])
				angle_error = heading_target - heading_curr
				angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

				# time step (for integral accumulation)
				now = time()
				if not hasattr(node, "last_time"):
					node.last_time = now
					node.integral_dist = 0.0
					node.integral_angle = 0.0
				dt = now - node.last_time
				node.last_time = now

				# accumulate integral terms (with basic anti-windup)
				node.integral_dist += dist * dt
				node.integral_angle += angle_error * dt
				node.integral_dist = max(min(node.integral_dist, 1.0), -1.0)     # clamp
				node.integral_angle = max(min(node.integral_angle, 1.0), -1.0)

				# thresholds
				pos_tolerance = 0.2     # [m]
				heading_tolerance = 0.2 # [rad]

				if dist > pos_tolerance:
					# Phase 1: move to (x, y)
					lin_cmd = k_lin_p * dist + k_lin_i * node.integral_dist
					twist.linear.x = lin_cmd * (dx / (dist + 1e-6))   # normalize direction
					twist.linear.y = lin_cmd * (dy / (dist + 1e-6))
					twist.angular.z = 0.0
					state = "MOVING_TO_POSITION"

				elif abs(angle_error) > heading_tolerance:
					# Phase 2: rotate in place
					ang_cmd = k_ang_p * angle_error + k_ang_i * node.integral_angle
					twist.linear.x = twist.linear.y = 0.0
					twist.angular.z = ang_cmd
					state = "CORRECTING_HEADING"

				else:
					# Done: stop + reset integrators
					twist.linear.x = twist.linear.y = twist.angular.z = 0.0
					node.integral_dist = 0.0
					node.integral_angle = 0.0
					state = "GOAL_REACHED"

				# 🟢 Diagnostics
				print(
					f"[AUTO] {state}\n"
					f"  Target: ({x_target:.2f}, {y_target:.2f}, {heading_target:.2f} rad)\n"
					f"  Current: ({x_curr:.2f}, {y_curr:.2f}, {heading_curr:.2f} rad)\n"
					f"  Errors: dx={dx:.3f}, dy={dy:.3f}, dist={dist:.3f}, angle_err={angle_error:.3f} rad\n"
					f"  Integrals: dist_i={node.integral_dist:.3f}, ang_i={node.integral_angle:.3f}\n"
					f"  Cmd_vel: lin=({twist.linear.x:.3f}, {twist.linear.y:.3f}), ang_z={twist.angular.z:.3f}\n"
				)

					
			node.pub.publish(twist_msg)
			

	except Exception as e:
		print(e)

	finally:
		rclpy.shutdown()
		spinner.join()
	
	restoreTerminalSettings(settings)


if __name__ == '__main__':
    main()