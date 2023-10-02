import os, struct, array
from fcntl import ioctl

# /dev/input에 있는 js로 시작되는 장치들이 있다면 화면에 출력
# js0, js1 ... 게임패드나 조이스틱이 이해 해당된다.
for fn in os.listdir('/dev/input'):
    if fn.startswith('js'):
        print('  /dev/input/%s' % (fn))

# 축 값 저장 변수
axis_states = {}
# 버튼 값 저장 변수
button_states = {}

# ! 추가한 부분
threshold = 10000
# 축의 범위를 나눌 구간값 정의
divisions = [-32767, -threshold, 0, threshold, 32767]


# These constants were borrowed from linux/input.h
# 축 들의 이름을 알기 위한 변수들
# 키 맵핑을 위해 사용
axis_names = {
    0x00 : 'x',
    0x01 : 'y',
    0x02 : 'z',
    0x03 : 'rx',
    0x04 : 'ry',
    0x05 : 'rz',
    0x06 : 'throttle',
    0x07 : 'rudder',
    0x08 : 'wheel',
    0x09 : 'gas',
    0x0a : 'brake',
    0x10 : 'hat0x',
    0x11 : 'hat0y',
    0x12 : 'hat1x',
    0x13 : 'hat1y',
    0x14 : 'hat2x',
    0x15 : 'hat2y',
    0x16 : 'hat3x',
    0x17 : 'hat3y',
    0x18 : 'pressure',
    0x19 : 'distance',
    0x1a : 'tilt_x',
    0x1b : 'tilt_y',
    0x1c : 'tool_width',
    0x20 : 'volume',
    0x28 : 'misc',
}

# 버튼의 이름을 알기 위한
# 키 맵핑을 위해 사용
button_names = {
    0x120 : 'trigger',
    0x121 : 'thumb',
    0x122 : 'thumb2',
    0x123 : 'top',
    0x124 : 'top2',
    0x125 : 'pinkie',
    0x126 : 'base',
    0x127 : 'base2',
    0x128 : 'base3',
    0x129 : 'base4',
    0x12a : 'base5',
    0x12b : 'base6',
    0x12f : 'dead',
    0x130 : 'a',
    0x131 : 'b',
    0x132 : 'c',
    0x133 : 'x',
    0x134 : 'y',
    0x135 : 'z',
    0x136 : 'tl',
    0x137 : 'tr',
    0x138 : 'tl2',
    0x139 : 'tr2',
    0x13a : 'select',
    0x13b : 'start',
    0x13c : 'mode',
    0x13d : 'thumbl',
    0x13e : 'thumbr',

    0x220 : 'dpad_up',
    0x221 : 'dpad_down',
    0x222 : 'dpad_left',
    0x223 : 'dpad_right',

    # XBox 360 controller uses these codes.
    0x2c0 : 'dpad_left',
    0x2c1 : 'dpad_right',
    0x2c2 : 'dpad_up',
    0x2c3 : 'dpad_down',
}

axis_map = []
button_map = []

# Open the joystick device.
# /dev/input/js0 장치 열기
fn = '/dev/input/js0'
print('Opening %s...' % fn)
jsdev = open(fn, 'rb')
#%%
# Get the device name.
#buf = bytearray(63)
# 배열 선언
# 바이트(unsigned char)형으로 64개 생성
buf = array.array('B', [0] * 64)
# 드라이버로부터 조이스틱 이름 가져오기
ioctl(jsdev, 0x80006a13 + (0x10000 * len(buf)), buf) # JSIOCGNAME(len)
js_name = buf.tobytes().rstrip(b'\x00').decode('utf-8')
print('Device name: %s' % js_name)


#%%
# Get number of axes and buttons.
# 드라이버로부터 축 개수 가져오기
buf = array.array('B', [0])
ioctl(jsdev, 0x80016a11, buf) # JSIOCGAXES
# x,y 축이면 2
num_axes = buf[0]


#%%
# 드라이버로부터 버튼 개수 가져오기
buf = array.array('B', [0])
ioctl(jsdev, 0x80016a12, buf) # JSIOCGBUTTONS
# 버튼이 두 개면 2
num_buttons = buf[0]


#%%
# Get the axis map.
# 키 맵핑
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
#%%
# Get the button map.
# 축과 마찬가지로 버튼 번호로 이름을 가져온다.
buf = array.array('H', [0] * 200)
ioctl(jsdev, 0x80406a34, buf) # JSIOCGBTNMAP

for btn in buf[:num_buttons]:
    btn_name = button_names.get(btn, 'unknown(0x%03x)' % btn)
    button_map.append(btn_name)
    button_states[btn_name] = 0

print('%d axes found: %s' % (num_axes, ', '.join(axis_map)))
print('%d buttons found: %s' % (num_buttons, ', '.join(button_map)))

# Main event loop
# 키 이벤트 처리
while True:
    # 키 읽기 블록 상태(Block)
    # 키 입력이 들어오기 전까지 무조건 대기
    evbuf = jsdev.read(8)
    # 이벤트가 발생했다면
    if evbuf:
        # 시간, 값, 타입, 번호 등으로 가져옴
        time, value, type, number = struct.unpack('IhBB', evbuf)

        # type이 0x80이면 장치 초기 상태이다.
        if type & 0x80:
            print("(initial)", end="")

        # type이 0x01 버튼이 눌렸거나 떨어졌을때이다.
        if type & 0x01:
            # number 값으로 해당 버튼 이름 가져오기
            button = button_map[number]
            if button:
                # 해당 버튼 상태(button_states)에 value 값으로 변경
                button_states[button] = value
                if value:
                    print("%s pressed" % (button))
                else:
                    print("%s released" % (button))

        # # type이 0x02이면 축이 이동한 상태이다.
        # if type & 0x02:
        #     # number로 해당 축의 이름 가져오기
        #     axis = axis_map[number]
        #     if axis:
        #         # 값을 32767로 나눠서 0 또는 1, -1 로 표시
        #         # 축 값이 -32767 ~ 0 ~ 32767 사이 값으로 표시되는 데
        #         # 0보다 큰지 작은지 0인지를 구분하기 위함이다.
        #         fvalue = value / 32767.0
        #         # 상태값(0, 1, -1)을 저장
        #         axis_states[axis] = fvalue
        #         print("%s: %.3f" % (axis, fvalue))

# ! 수정된 부분
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