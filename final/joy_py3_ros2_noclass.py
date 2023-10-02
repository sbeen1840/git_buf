import os
import struct
import array
from fcntl import ioctl
import rclpy
from sensor_msgs.msg import Joy

def main():
    rclpy.init()
    node = rclpy.create_node('joystick_node')
    pub = node.create_publisher(Joy, 'joy', 10)


    # /dev/input에 있는 js로 시작되는 장치들이 있다면 화면에 출력
    # js0, js1 ... 게임패드나 조이스틱이 해당됩니다.
    for fn in os.listdir('/dev/input'):
        if fn.startswith('js'):
            print('  /dev/input/%s' % (fn))


    # We'll store the states here.
    axis_states = {}
    button_states = {}

    # ! 5단계 지정을 위해
    threshold = 10000
    divisions = [-32767, -threshold, 0, threshold, 32767]

    # ! 맵핑 바꿈
    # ! (http://wiki.ros.org/joy#Microsoft_Xbox_360_Wired_Controller_for_Linux)
    # These constants were borrowed from linux/input.h
    axis_names = {
        0x00 : 'Left/Right Axis stick left',
        0x01 : 'Up/Down Axis stick left',
        0x02 : 'LT',
        0x03 : 'Left/Right Axis stick right',
        0x04 : 'Up/Down Axis stick right',
        0x05 : 'RT',
        0x06 : 'cross key left/right',
        0x07 : 'cross key up/down'
    }

    button_names = {
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

    axis_map = []
    button_map = []


    # Open the joystick device.
    # /dev/input/js0 장치 열기
    fn = '/dev/input/js0'
    print('Opening %s...' % fn)
    jsdev = open(fn, 'rb')


    # Get the device name.
    #buf = bytearray(63)
    buf = array.array('B', [0] * 64)
    ioctl(jsdev, 0x80006a13 + (0x10000 * len(buf)), buf) # JSIOCGNAME(len)
    js_name = buf.tostring().rstrip(b'\x00').decode('utf-8')
    print(('Device name: %s' % js_name))

    # Get number of axes and buttons.
    buf = array.array('B', [0])
    ioctl(jsdev, 0x80016a11, buf) # JSIOCGAXES
    num_axes = buf[0]

    buf = array.array('B', [0])
    ioctl(jsdev, 0x80016a12, buf) # JSIOCGBUTTONS
    num_buttons = buf[0]

    # Get the axis map.
    buf = array.array('B', [0] * 0x40)
    ioctl(jsdev, 0x80406a32, buf) # JSIOCGAXMAP
    # 읽은 값에서 총 축 수만큼 loop 돌림
    for axis in buf[:num_axes]:
        # axis_names의 첫번째 번호와 같은 이름을 가져온다.
        # 예를들어 axis가 0이면 axis_names에서 0x00 > 'x'를 가져오고
        # axis가 1이면 axis_names에 0x01 > 'y'를 가져오기 된다.
        axis_name = axis_names.get(axis, 'unknown(0x%02x)' % axis)
        axis_map.append(axis_name)
        # 해당 축 0.0 으로 초기화
        axis_states[axis_name] = 0.0

    # Get the button map.
    # 축과 마찬가지로 버튼 번호로 이름을 가져온다.
    buf = array.array('H', [0] * 200)
    ioctl(jsdev, 0x80406a34, buf) # JSIOCGBTNMAP

    for btn in buf[:num_buttons]:
        btn_name = button_names.get(btn, 'unknown(0x%03x)' % btn)
        button_map.append(btn_name)
        button_states[btn_name] = 0

    print('%d axes found' % num_axes)
    print('%d buttons found' % num_buttons)


    while rclpy.ok():

        joy_msg = Joy()
        evbuf = jsdev.read(8)
        if evbuf:
            time, value, type, number = struct.unpack('IhBB', evbuf)

            if type & 0x80:
                print("(initial)", end="")

            if type & 0x01:
                button = button_map[number]
                if button:
                    button_states[button] = value
                    if value:
                        print(("%s pressed" % (button)))
                    else:
                        print(("%s released" % (button)))
                joy_msg.header.stamp = node.get_clock().now().to_msg()
                joy_msg.buttons[number] = value


            if type & 0x02:
                axis = axis_map[number]
                if axis:
                    cur_value = value
                    for i in range(len(divisions) - 1):
                        if divisions[i] <= cur_value <= divisions[i + 1]:
                            cur_state = i - 2
                            break
                    else:
                        cur_state = 0

                    axis_states[axis] = cur_state
                    print("%s: %d" % (axis, cur_state))
                joy_msg.header.stamp = node.get_clock().now().to_msg()
                joy_msg.axes[number] = cur_state

            pub.publish(joy_msg)

    jsdev.close()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


