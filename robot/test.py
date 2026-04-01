from pynput import keyboard

# 存储按键状态
key_states = {}

# 按键按下事件
def on_press(key):
    try:
        # 记录按键按下
        key_states[key.char] = 1
        print(f"{key.char} 按下: 1")
    except AttributeError:
        # 特殊键（如方向键，空格键）没有 .char 属性
        key_states[key] = 1
        print(f"{key} 按下: 1")

# 按键释放事件
def on_release(key):
    try:
        # 记录按键松开
        key_states[key.char] = 0
        print(f"{key.char} 松开: 0")
    except AttributeError:
        # 特殊键（如方向键，空格键）没有 .char 属性
        key_states[key] = 0
        print(f"{key} 松开: 0")

    # 如果按下 Esc 键退出监听
    if key == keyboard.Key.esc:
        return False

# 启动键盘监听器
with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
    listener.join()