from pyjoycon import JoyCon, get_L_id, get_R_id
import vgamepad as vg
import time

# ============================================================
# Utility Functions
# ============================================================

def normalize(value, min_val=0, max_val=4095):
    center = (max_val - min_val) / 2
    return (value - center) / center

def deadband(value, threshold=0.08):
    return 0 if abs(value) < threshold else value

def to_xbox_axis(value):
    return int(max(-1.0, min(1.0, value)) * 32767)

# ============================================================
# Connect JoyCons
# ============================================================

print("Connecting JoyCons...")

left_id = get_L_id()
right_id = get_R_id()

if not left_id:
    raise Exception("Left JoyCon not found")
if not right_id:
    raise Exception("Right JoyCon not found")

left = JoyCon(*left_id)
right = JoyCon(*right_id)

print("JoyCons connected!")

# ============================================================
# Create Virtual Xbox Controller
# ============================================================

gamepad = vg.VX360Gamepad()
print("Virtual Xbox controller created!")

# ============================================================
# Main Loop
# ============================================================

while True:
    l_status = left.get_status()
    r_status = right.get_status()

    # ======================
    # LEFT STICK
    # ======================
    lx = normalize(l_status["analog-sticks"]["left"]["horizontal"])
    ly = normalize(l_status["analog-sticks"]["left"]["vertical"])

    lx = deadband(lx)
    ly = deadband(-ly)  # invert Y for Xbox

    gamepad.left_joystick(
        x_value=to_xbox_axis(lx),
        y_value=to_xbox_axis(ly)
    )

    # ======================
    # RIGHT STICK
    # ======================
    rx = normalize(r_status["analog-sticks"]["right"]["horizontal"])
    ry = normalize(r_status["analog-sticks"]["right"]["vertical"])

    rx = deadband(rx)
    ry = deadband(-ry)

    gamepad.right_joystick(
        x_value=to_xbox_axis(rx),
        y_value=to_xbox_axis(ry)
    )

    # ======================
    # TRIGGERS
    # ======================
    if l_status["buttons"]["left"]["zl"]:
        gamepad.left_trigger(value=255)
    else:
        gamepad.left_trigger(value=0)

    if r_status["buttons"]["right"]["zr"]:
        gamepad.right_trigger(value=255)
    else:
        gamepad.right_trigger(value=0)

    # ======================
    # BUMPERS
    # ======================
    if l_status["buttons"]["left"]["l"]:
        gamepad.press_button(vg.XUSB_BUTTON.XUSB_GAMEPAD_LEFT_SHOULDER)
    else:
        gamepad.release_button(vg.XUSB_BUTTON.XUSB_GAMEPAD_LEFT_SHOULDER)

    if r_status["buttons"]["right"]["r"]:
        gamepad.press_button(vg.XUSB_BUTTON.XUSB_GAMEPAD_RIGHT_SHOULDER)
    else:
        gamepad.release_button(vg.XUSB_BUTTON.XUSB_GAMEPAD_RIGHT_SHOULDER)

    # ======================
    # ABXY (Right JoyCon)
    # ======================
    right_buttons = r_status["buttons"]["right"]

    button_map = {
        "a": vg.XUSB_BUTTON.XUSB_GAMEPAD_A,
        "b": vg.XUSB_BUTTON.XUSB_GAMEPAD_B,
        "x": vg.XUSB_BUTTON.XUSB_GAMEPAD_X,
        "y": vg.XUSB_BUTTON.XUSB_GAMEPAD_Y,
    }

    for name, xbox_btn in button_map.items():
        if right_buttons[name]:
            gamepad.press_button(xbox_btn)
        else:
            gamepad.release_button(xbox_btn)

    # ======================
    # START / BACK
    # ======================
    shared = r_status["buttons"]["shared"]

    if shared["plus"]:
        gamepad.press_button(vg.XUSB_BUTTON.XUSB_GAMEPAD_START)
    else:
        gamepad.release_button(vg.XUSB_BUTTON.XUSB_GAMEPAD_START)

    if shared["minus"]:
        gamepad.press_button(vg.XUSB_BUTTON.XUSB_GAMEPAD_BACK)
    else:
        gamepad.release_button(vg.XUSB_BUTTON.XUSB_GAMEPAD_BACK)

    # ======================
    # DPAD (Left JoyCon)
    # ======================
    dpad = l_status["buttons"]["left"]

    dpad_map = {
        "up": vg.XUSB_BUTTON.XUSB_GAMEPAD_DPAD_DOWN,
        "down": vg.XUSB_BUTTON.XUSB_GAMEPAD_DPAD_UP,
        "left": vg.XUSB_BUTTON.XUSB_GAMEPAD_DPAD_RIGHT,
        "right": vg.XUSB_BUTTON.XUSB_GAMEPAD_DPAD_LEFT,
    }

    for direction, xbox_btn in dpad_map.items():
        if dpad[direction]:
            gamepad.press_button(xbox_btn)
        else:
            gamepad.release_button(xbox_btn)

    # Push update to Windows
    gamepad.update()

    time.sleep(0.01)