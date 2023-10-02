
import os, struct, array
from fcntl import ioctl

# ! ros2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy


class JoystickRos2(Node):
    def __init__(self):
        super().__init__('joystick_node')
        self.publisher = self.create_publisher(Joy, 'joystick', 10)

        # Initialize the states and mappings
        self.axis_states = {}
        self.button_states = {}

        # Define the thresholds
        self.threshold = 10000
        self.divisions = [-32767, -self.threshold, 0, self.threshold, 32767]

        # Define the mappings
        self.axis_names = {
            0x00 : 'Left/Right Axis stick left',
            0x01 : 'Up/Down Axis stick left',
            0x02 : 'LT',
            0x03 : 'Left/Right Axis stick right',
            0x04 : 'Up/Down Axis stick right',
            0x05 : 'RT',
            0x06 : 'cross key left/right',
            0x07 : 'cross key up/down'
        }
        self.button_names = {
            0x00 : 'A',
            0x01 : 'B',
            0x02 : 'X',
            0x03 : 'Y',
            0x04 : 'LB',
            0x05 : 'RB',
            0x06 : 'back',
            0x07 : 'start',
            0x08 : 'power',
            0x09 : 'Button stick left',
            0x0a : 'Button stick right'
        }

        self.axis_map = []
        self.button_map = []

        self.fn = '/dev/input/js0'
        self.get_joy_device()

    def get_joy_device(self):
        try:
            # 조이스틱 디바이스 열기
            self.get_logger().info(f'Opening {self.fn}...')
            self.jsdev = open(self.fn, 'rb')

            # 드라이버로부터 조이스틱 이름 가져오기
            buf = array.array('B', [0] * 64)
            ioctl(self.jsdev, 0x80006a13 + (0x10000 * len(buf)), buf)  # JSIOCGNAME(len)
            js_name = buf.tobytes().rstrip(b'\x00').decode('utf-8')
            self.get_logger().info('Device name: %s' % js_name)

            # Get number of axes and buttons.
            # 드라이버로부터 축 개수 가져오기
            buf = array.array('B', [0])
            ioctl(self.jsdev, 0x80016a11, buf)  # JSIOCGAXES
            num_axes = buf[0]

            # 드라이버로부터 버튼 개수 가져오기
            buf = array.array('B', [0])
            ioctl(self.jsdev, 0x80016a12, buf)  # JSIOCGBUTTONS
            num_buttons = buf[0]

            # Get the axis map.
            buf = array.array('B', [0] * 0x40)
            ioctl(self.jsdev, 0x80406a32, buf)  # JSIOCGAXMAP

            for axis in buf[:num_axes]:
                axis_name = self.axis_names.get(axis, 'unknown(0x%02x)' % axis)
                self.axis_map.append(axis_name)
                self.axis_states[axis_name] = 0.0

            # Get the button map.
            buf = array.array('H', [0] * 200)
            ioctl(self.jsdev, 0x80406a34, buf)  # JSIOCGBTNMAP

            for btn in buf[:num_buttons]:
                btn_name = self.button_names.get(btn, 'unknown(0x%03x)' % btn)
                self.button_map.append(btn_name)
                self.button_states[btn_name] = 0

            self.get_logger().info('%d axes found: %s' % (num_axes, ', '.join(self.axis_map)))
            self.get_logger().info('%d buttons found: %s' % (num_buttons, ', '.join(self.button_map)))

        except IOError:
            self.get_logger().error(f'Error opening {self.fn}')


    def run(self):
        while rclpy.ok():
            try:
                evbuf = self.jsdev.read(8)
                if evbuf:
                    time, value, type, number = struct.unpack('IhBB', evbuf)

                    if type & 0x80:
                        self.get_logger().info("(initial)")

                    if type & 0x01:
                        button = self.button_map[number]
                        if button:
                            self.button_states[button] = value
                            if value:
                                self.get_logger().info("%s pressed" % button)
                            else:
                                self.get_logger().info("%s released" % button)

                    if type & 0x02:
                        axis = self.axis_map[number]
                        if axis:
                            cur_value = value
                            for i in range(len(self.divisions) - 1):
                                if self.divisions[i] <= cur_value <= self.divisions[i + 1]:
                                    cur_state = i - 2  # -2, -1, 0, 1, 2 값 중 하나로 분류
                                    break
                            else:
                                cur_state = 0  # 범위에 들어가지 않는 경우 0으로 설정
                            self.axis_states[axis] = cur_state
                            self.get_logger().info("%s: %d" % (axis, cur_state))  # 로그로 변경



                    # ! 리스트로 보내는게 의미가 있을까..? 근데 Joy 메세지형 자체가 배열형태라 (?)
                    joy_msg = Joy()
                    joy_msg.header.stamp = self.get_clock().now().to_msg()
                    joy_msg.axes = [self.axis_states[axis] for axis in self.axis_map]
                    joy_msg.buttons = [self.button_states[button] for button in self.button_map]
                    self.publisher.publish(joy_msg)

            except Exception as e:
                self.get_logger().error(f'Error reading joystick input: {str(e)}')


def main(args=None):

    # ! 시작전에 장치확인
    print('Available devices:')
    for fn in os.listdir('/dev/input'):
        if fn.startswith('js'):
            print(('  /dev/input/%s' % (fn)))


    rclpy.init(args=args)
    JoyRos2 = JoystickRos2()
    JoyRos2.run()
    JoyRos2.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

