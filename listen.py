import socket
import struct
import io
import threading
import time
from time import monotonic
import RPi.GPIO as GPIO
from picamera2 import Picamera2

# ----------------------
# Helpers
# ----------------------
def recv_all(sock, n):
    """Receive exactly n bytes or return None if the peer closes early."""
    buf = b""
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            return None
        buf += chunk
    return buf

# ----------------------
# Network Configuration
# ----------------------
HOST = '0.0.0.0'
WHEEL_PORT = 8000
CAMERA_PORT = 8001
PID_CONFIG_PORT = 8002
ENCODER_STREAM_PORT = 8003 
ENCODER_STREAM_HZ = 50

# ----------------------
# Pins
# ----------------------
RIGHT_MOTOR_ENA = 18
RIGHT_MOTOR_IN1 = 17
RIGHT_MOTOR_IN2 = 27
LEFT_MOTOR_ENB  = 25
LEFT_MOTOR_IN3  = 23
LEFT_MOTOR_IN4  = 24
LEFT_ENCODER    = 26
RIGHT_ENCODER   = 16

# ----------------------
# PID Constants (defaults; client can override)
# ----------------------
use_PID = 0.0

# Straight-line PID (your old behavior)
KP, KI, KD = 0.0, 0.0, 0.0

# Turning PID (from your newer code)
KP_R, KI_R, KD_R = 0.0, 0.0, 0.0

MAX_CORRECTION = 30  # clamp for both straight and turn corrections

# ----------------------
# Global state
# ----------------------
running = True
left_pwm, right_pwm = 0.0, 0.0
left_count, right_count = 0, 0
prev_left_state, prev_right_state = None, None

use_ramping = True
RAMP_RATE = 250            # PWM units per second
MIN_RAMP_THRESHOLD = 15    # Only ramp if change is greater than this
MIN_PWM_THRESHOLD = 15
BRAKE_ON_ZERO = False

current_movement, prev_movement = 'stop', 'stop'

# ----------------------
# GPIO / Encoders / PWM
# ----------------------
def setup_gpio():
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

    # Motor pins
    GPIO.setup(RIGHT_MOTOR_ENA, GPIO.OUT)
    GPIO.setup(RIGHT_MOTOR_IN1, GPIO.OUT)
    GPIO.setup(RIGHT_MOTOR_IN2, GPIO.OUT)
    GPIO.setup(LEFT_MOTOR_ENB,  GPIO.OUT)
    GPIO.setup(LEFT_MOTOR_IN3,  GPIO.OUT)
    GPIO.setup(LEFT_MOTOR_IN4,  GPIO.OUT)

    # Prevent slight motor jerk when connection is established
    GPIO.output(RIGHT_MOTOR_ENA, GPIO.LOW)
    GPIO.output(LEFT_MOTOR_ENB,  GPIO.LOW)

    # Encoders
    GPIO.setup(LEFT_ENCODER,  GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(RIGHT_ENCODER, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.add_event_detect(LEFT_ENCODER,  GPIO.BOTH, callback=left_encoder_callback)
    GPIO.add_event_detect(RIGHT_ENCODER, GPIO.BOTH, callback=right_encoder_callback)

    # PWM at 100 Hz
    global left_motor_pwm, right_motor_pwm
    left_motor_pwm  = GPIO.PWM(LEFT_MOTOR_ENB,  100)
    right_motor_pwm = GPIO.PWM(RIGHT_MOTOR_ENA, 100)
    left_motor_pwm.start(0)
    right_motor_pwm.start(0)

def left_encoder_callback(channel):
    global left_count, prev_left_state
    current_state = GPIO.input(LEFT_ENCODER)
    # Count only on state changes (reduces false positives)
    if prev_left_state is not None and current_state != prev_left_state:
        left_count += 1
        prev_left_state = current_state
    elif prev_left_state is None:
        prev_left_state = current_state

def right_encoder_callback(channel):
    global right_count, prev_right_state
    current_state = GPIO.input(RIGHT_ENCODER)
    if prev_right_state is not None and current_state != prev_right_state:
        right_count += 1
        prev_right_state = current_state
    elif prev_right_state is None:
        prev_right_state = current_state

def reset_encoder():
    global left_count, right_count
    left_count, right_count = 0, 0

def set_motors(left, right):
    global prev_movement, current_movement

    # Pre-start "kick" to overcome static friction and reduce initial yaw
    if prev_movement == 'stop' and current_movement in ['forward', 'backward']:
        if current_movement == 'forward':
            GPIO.output(RIGHT_MOTOR_IN1, GPIO.HIGH)
            GPIO.output(RIGHT_MOTOR_IN2, GPIO.LOW)
            GPIO.output(LEFT_MOTOR_IN3,  GPIO.HIGH)
            GPIO.output(LEFT_MOTOR_IN4,  GPIO.LOW)
        elif current_movement == 'backward':
            GPIO.output(RIGHT_MOTOR_IN1, GPIO.LOW)
            GPIO.output(RIGHT_MOTOR_IN2, GPIO.HIGH)
            GPIO.output(LEFT_MOTOR_IN3,  GPIO.LOW)
            GPIO.output(LEFT_MOTOR_IN4,  GPIO.HIGH)
        left_motor_pwm.ChangeDutyCycle(100)
        right_motor_pwm.ChangeDutyCycle(100)
        time.sleep(0.05)

    # Right motor
    if right > 0:
        GPIO.output(RIGHT_MOTOR_IN1, GPIO.HIGH)
        GPIO.output(RIGHT_MOTOR_IN2, GPIO.LOW)
        right_motor_pwm.ChangeDutyCycle(min(right, 100))
    elif right < 0:
        GPIO.output(RIGHT_MOTOR_IN1, GPIO.LOW)
        GPIO.output(RIGHT_MOTOR_IN2, GPIO.HIGH)
        right_motor_pwm.ChangeDutyCycle(min(abs(right), 100))
    else:
        if BRAKE_ON_ZERO:
            # active brake (old behavior)
            GPIO.output(RIGHT_MOTOR_IN1, GPIO.HIGH)
            GPIO.output(RIGHT_MOTOR_IN2, GPIO.HIGH)
            right_motor_pwm.ChangeDutyCycle(100)
        else:
            # COAST (new behavior)
            GPIO.output(RIGHT_MOTOR_IN1, GPIO.LOW)
            GPIO.output(RIGHT_MOTOR_IN2, GPIO.LOW)
            right_motor_pwm.ChangeDutyCycle(0)

    # Left motor
    if left > 0:
        GPIO.output(LEFT_MOTOR_IN3, GPIO.HIGH)
        GPIO.output(LEFT_MOTOR_IN4, GPIO.LOW)
        left_motor_pwm.ChangeDutyCycle(min(left, 100))
    elif left < 0:
        GPIO.output(LEFT_MOTOR_IN3, GPIO.LOW)
        GPIO.output(LEFT_MOTOR_IN4, GPIO.HIGH)
        left_motor_pwm.ChangeDutyCycle(min(abs(left), 100))
    else:
        if BRAKE_ON_ZERO:
            GPIO.output(LEFT_MOTOR_IN3, GPIO.HIGH)
            GPIO.output(LEFT_MOTOR_IN4, GPIO.HIGH)
            left_motor_pwm.ChangeDutyCycle(100)
        else:
            GPIO.output(LEFT_MOTOR_IN3, GPIO.LOW)
            GPIO.output(LEFT_MOTOR_IN4, GPIO.LOW)
            left_motor_pwm.ChangeDutyCycle(0)

def apply_min_threshold(pwm_value, min_threshold):
    if pwm_value == 0:
        return 0
    if abs(pwm_value) < min_threshold:
        return min_threshold if pwm_value > 0 else -min_threshold
    return pwm_value

def sgn(x: float) -> int:
    return 1 if x > 0 else (-1 if x < 0 else 0)

def encoder_stream_server():
    """Continuously push (left_count, right_count) as two big-endian int32s."""
    global running, left_count, right_count

    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind((HOST, ENCODER_STREAM_PORT))
    server_socket.listen(1)
    print(f"Encoder stream server started on port {ENCODER_STREAM_PORT}")

    period = 1.0 / ENCODER_STREAM_HZ

    while running:
        try:
            client_socket, _ = server_socket.accept()
            print("Encoder stream client connected")
            while running:
                # Pack (L, R) as 8 bytes: "!ii" = big-endian int32, int32
                payload = struct.pack("!ii", left_count, right_count)
                try:
                    client_socket.sendall(payload)
                except Exception:
                    print("Encoder stream client disconnected")
                    break
                time.sleep(period)
        except Exception as e:
            print(f"Encoder stream server error: {e}")
        finally:
            try:
                client_socket.close()
            except:
                pass

    server_socket.close()


# ----------------------
# PID Control Thread
# ----------------------
def pid_control():
    # Straight uses your original approach.
    # Turn uses PID from your newer code.
    global left_pwm, right_pwm, left_count, right_count
    global use_PID, KP, KI, KD, KP_R, KI_R, KD_R
    global prev_movement, current_movement

    # Straight PID state
    integral = 0.0
    last_error = 0.0
    last_time = monotonic()

    # Turn PID state
    integral_turn = 0.0
    last_error_turn = 0.0

    # For turn PID deltas
    lastL = left_count
    lastR = right_count

    # Ramping state
    ramp_left_pwm = 0.0
    ramp_right_pwm = 0.0
    previous_left_target = 0.0
    previous_right_target = 0.0

    while running:
        current_time = monotonic()
        dt = current_time - last_time if current_time > last_time else 1e-6
        last_time = current_time

        prev_movement = current_movement
        if (left_pwm > 0 and right_pwm > 0):
            current_movement = 'forward'
        elif (left_pwm < 0 and right_pwm < 0):
            current_movement = 'backward'
        elif (left_pwm == 0 and right_pwm == 0):
            current_movement = 'stop'
        else:
            current_movement = 'turn'

        if not use_PID:
            target_left_pwm = left_pwm
            target_right_pwm = right_pwm

        else:
            if current_movement in ('forward', 'backward'):
                # === Straight (keep your original behavior) ===
                error = left_count - right_count
                proportional = KP * error
                integral += KI * error * dt
                integral = max(-MAX_CORRECTION, min(integral, MAX_CORRECTION))
                derivative = KD * (error - last_error) / dt if dt > 0 else 0.0
                correction = proportional + integral + derivative
                correction = max(-MAX_CORRECTION, min(correction, MAX_CORRECTION))
                last_error = error

                if current_movement == 'backward':
                    correction = -correction

                target_left_pwm  = left_pwm  - correction
                target_right_pwm = right_pwm + correction

                # reset turn PID accumulators while in straight
                integral_turn = 0.0
                last_error_turn = 0.0
                # and refresh the delta baseline for turn
                lastL, lastR = left_count, right_count

            elif current_movement == 'turn':
                # === Turn (PID from your new code) ===
                # Make a clean pivot target: equal-and-opposite magnitudes
                base = max(abs(left_pwm), abs(right_pwm))
                t_ls, t_rs = sgn(left_pwm), sgn(right_pwm)
                # if one side is zero, enforce opposite sign for a pivot
                if t_ls == 0 and t_rs != 0:
                    t_ls = -t_rs
                if t_rs == 0 and t_ls != 0:
                    t_rs = -t_ls
                # ensure opposite signs (pivot); if same, keep as-is (it would be straight)
                if t_ls == t_rs and t_ls != 0:
                    t_rs = -t_ls

                target_left_pwm  = base * t_ls
                target_right_pwm = base * t_rs

                # Compute tick deltas since last loop
                curL, curR = left_count, right_count
                dL, dR = curL - lastL, curR - lastR
                lastL, lastR = curL, curR

                # PID on absolute tick rates so direction sign doesn't flip the error
                turn_error = abs(dL) - abs(dR)

                proportional_t = KP_R * turn_error
                integral_turn += KI_R * turn_error * dt
                integral_turn = max(-MAX_CORRECTION, min(integral_turn, MAX_CORRECTION))
                derivative_t = KD_R * (turn_error - last_error_turn) / dt if dt > 0 else 0.0
                correction_turn = proportional_t + integral_turn + derivative_t
                correction_turn = max(-MAX_CORRECTION, min(correction_turn, MAX_CORRECTION))
                last_error_turn = turn_error

                # Apply correction symmetrically around the pivot command
                target_left_pwm  -= correction_turn
                target_right_pwm += correction_turn

                # reset straight accumulators while turning
                integral = 0.0
                last_error = 0.0

            else:
                # Stopped: reset everything and zero targets
                integral = 0.0
                last_error = 0.0
                integral_turn = 0.0
                last_error_turn = 0.0
                reset_encoder()
                target_left_pwm = 0.0
                target_right_pwm = 0.0

        # ----------------------
        # Ramping (keep your original "synchronized" ramp)
        # ----------------------
        if use_ramping and use_PID:
            max_change_per_cycle = RAMP_RATE * dt

            left_diff  = target_left_pwm  - ramp_left_pwm
            right_diff = target_right_pwm - ramp_right_pwm

            left_needs_ramp  = abs(left_diff)  > MIN_RAMP_THRESHOLD
            right_needs_ramp = abs(right_diff) > MIN_RAMP_THRESHOLD

            left_direction_change  = (target_left_pwm  * previous_left_target  < 0) and target_left_pwm  != 0 and previous_left_target  != 0
            right_direction_change = (target_right_pwm * previous_right_target < 0) and target_right_pwm != 0 and previous_right_target != 0

            if left_direction_change:
                ramp_left_pwm = target_left_pwm
            if right_direction_change:
                ramp_right_pwm = target_right_pwm

            if not left_direction_change and not right_direction_change:
                if left_needs_ramp or right_needs_ramp:
                    if abs(left_diff) <= max_change_per_cycle:
                        ramp_left_pwm = target_left_pwm
                    else:
                        ramp_left_pwm += max_change_per_cycle if left_diff > 0 else -max_change_per_cycle

                    if abs(right_diff) <= max_change_per_cycle:
                        ramp_right_pwm = target_right_pwm
                    else:
                        ramp_right_pwm += max_change_per_cycle if right_diff > 0 else -max_change_per_cycle
                else:
                    ramp_left_pwm  = target_left_pwm
                    ramp_right_pwm = target_right_pwm

            previous_left_target  = target_left_pwm
            previous_right_target = target_right_pwm
        else:
            ramp_left_pwm  = target_left_pwm
            ramp_right_pwm = target_right_pwm

        final_left_pwm  = apply_min_threshold(ramp_left_pwm,  MIN_PWM_THRESHOLD)
        final_right_pwm = apply_min_threshold(ramp_right_pwm, MIN_PWM_THRESHOLD)
        set_motors(final_left_pwm, final_right_pwm)

        if ramp_left_pwm != 0 or ramp_right_pwm != 0:
            print(f"(Left PWM, Right PWM)=({ramp_left_pwm:.2f},{ramp_right_pwm:.2f}), "
                  f"(Left Enc, Right Enc)=({left_count}, {right_count}), mode={current_movement}")

        time.sleep(0.01)

# ----------------------
# Camera streaming server
# ----------------------
def camera_stream_server():
    picam2 = Picamera2()
    camera_config = picam2.create_preview_configuration(lores={"size": (640,480)})
    picam2.configure(camera_config)
    picam2.start()

    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind((HOST, CAMERA_PORT))
    server_socket.listen(1)
    print(f"Camera stream server started on port {CAMERA_PORT}")

    while running:
        try:
            client_socket, _ = server_socket.accept()
            print(f"Camera stream client connected")
            while running:
                stream = io.BytesIO()
                picam2.capture_file(stream, format='jpeg')
                stream.seek(0)
                jpeg_data = stream.getvalue()
                jpeg_size = len(jpeg_data)
                try:
                    client_socket.sendall(struct.pack("!I", jpeg_size))
                    client_socket.sendall(jpeg_data)
                except:
                    print("Camera stream client disconnected")
                    break
                time.sleep(0.01)
        except Exception as e:
            print(f"Camera stream server error: {str(e)}")

        if 'client_socket' in locals() and client_socket:
            client_socket.close()

    server_socket.close()
    picam2.stop()

# ----------------------
# PID configuration server
# ----------------------
def pid_config_server():
    global use_PID, KP, KI, KD, KP_R, KI_R, KD_R

    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind((HOST, PID_CONFIG_PORT))
    server_socket.listen(1)
    print(f"PID config server started on port {PID_CONFIG_PORT}")

    while running:
        try:
            client_socket, _ = server_socket.accept()
            print(f"PID config client connected")
            try:
                # Try 7 floats first: use_PID, KP, KI, KD, KP_R, KI_R, KD_R
                data = recv_all(client_socket, 28)
                ok = False
                if data is not None and len(data) == 28:
                    use_PID, KP, KI, KD, KP_R, KI_R, KD_R = struct.unpack("!fffffff", data)
                    ok = True
                    if use_PID:
                        print(f"Updated PID: STR(KP={KP},KI={KI},KD={KD}) TURN(KP_R={KP_R},KI_R={KI_R},KD_R={KD_R})")
                    else:
                        print("PID turned OFF by client.")
                else:
                    # Fall back to 4 floats
                    data4 = recv_all(client_socket, 16) if data is None else None
                    if data4 is not None and len(data4) == 16:
                        use_PID, KP, KI, KD = struct.unpack("!ffff", data4)
                        ok = True
                        if use_PID:
                            print(f"Updated STR PID only: KP={KP}, KI={KI}, KD={KD}")
                        else:
                            print("PID turned OFF by client.")
                resp = struct.pack("!i", 1 if ok else 0)
                client_socket.sendall(resp)
            except Exception as e:
                print(f"PID config socket error: {str(e)}")
                try:
                    client_socket.sendall(struct.pack("!i", 0))
                except:
                    pass
            client_socket.close()
        except Exception as e:
            print(f"PID config server error: {str(e)}")

    server_socket.close()

# ----------------------
# Wheel command server
# ----------------------
def wheel_server():
    global left_pwm, right_pwm, running, left_count, right_count

    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind((HOST, WHEEL_PORT))
    server_socket.listen(1)
    print(f"Wheel server started on port {WHEEL_PORT}")

    while running:
        try:
            client_socket, _ = server_socket.accept()
            print(f"Wheel client connected")
            while running:
                try:
                    data = recv_all(client_socket, 8)
                    if not data:
                        print("Wheel client sending speed error")
                        break
                    left_speed, right_speed = struct.unpack("!ff", data)
                    left_pwm, right_pwm = left_speed * 100.0, right_speed * 100.0

                    # Send encoder counts back
                    response = struct.pack("!ii", left_count, right_count)
                    client_socket.sendall(response)
                except Exception as e:
                    print(f"Wheel client disconnected")
                    break
        except Exception as e:
            print(f"Wheel server error: {str(e)}")

        if 'client_socket' in locals() and client_socket:
            client_socket.close()

    server_socket.close()

# ----------------------
# Main
# ----------------------
def main():
    try:
        setup_gpio()

        # PID control
        pid_thread = threading.Thread(target=pid_control, daemon=True)
        pid_thread.start()

        # Camera
        camera_thread = threading.Thread(target=camera_stream_server, daemon=True)
        camera_thread.start()

        # PID config
        pid_config_thread = threading.Thread(target=pid_config_server, daemon=True)
        pid_config_thread.start()

        # NEW: Encoder stream
        enc_stream_thread = threading.Thread(target=encoder_stream_server, daemon=True)
        enc_stream_thread.start()

        # Wheel server (main thread)
        wheel_server()

    except KeyboardInterrupt:
        print("Stopping...")
    finally:
        global running
        running = False
        GPIO.cleanup()
        print("Cleanup complete")

if __name__ == "__main__":
    main()
