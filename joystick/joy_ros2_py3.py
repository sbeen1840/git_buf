
# Used 2to3 to convert from python2 to python3
# Change 'tobytes' to 'tostring'


import os, struct, array
from fcntl import ioctl

# Iterate over the joystick devices.
print('Available devices:')

for fn in os.listdir('/dev/input'):
    if fn.startswith('js'):
        print(('  /dev/input/%s' % (fn)))
#%%



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
fn = '/dev/input/js0'
print(('Opening %s...' % fn))
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

for axis in buf[:num_axes]:
    axis_name = axis_names.get(axis, 'unknown(0x%02x)' % axis)
    axis_map.append(axis_name)
    axis_states[axis_name] = 0.0

# Get the button map.
buf = array.array('H', [0] * 200)
ioctl(jsdev, 0x80406a34, buf) # JSIOCGBTNMAP

for btn in buf[:num_buttons]:
    btn_name = button_names.get(btn, 'unknown(0x%03x)' % btn)
    button_map.append(btn_name)
    button_states[btn_name] = 0

print(('%d axes found: %s' % (num_axes, ', '.join(axis_map))))
print(('%d buttons found: %s' % (num_buttons, ', '.join(button_map))))

# Main event loop
while True:
    evbuf = jsdev.read(8)
    if evbuf:
        time, value, type, number = struct.unpack('IhBB', evbuf)

        if type & 0x80:
            print("(initial)")

        if type & 0x01:
            button = button_map[number]
            if button:
                button_states[button] = value
                if value:
                    print(("%s pressed" % (button)))
                else:
                    print(("%s released" % (button)))

        # ! 축 5단계(-2~2) 변경파트
        if type & 0x02:
            axis = axis_map[number]
            if axis:
                # 현재 축 값
                cur_value = value

                # 현재 상태 계산
                for i in range(len(divisions) - 1):
                    if divisions[i] <= cur_value <= divisions[i + 1]:
                        cur_state = i - 2  # -2, -1, 0, 1, 2 값 중 하나로 분류
                        break
                else:
                    cur_state = 0  # 범위에 들어가지 않는 경우 0으로 설정

                # 상태 업데이트 및 출력
                axis_states[axis] = cur_state
                print("%s: %d" % (axis, cur_state))