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
    """Receive exactly n bytes from a TCP socket or return None if the peer closes early."""
    buf = b""
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            return None
        buf += chunk
    return buf

def sgn(x: float) -> int:
    """Sign function: +1, -1, or 0."""
    return 1 if x > 0 else (-1 if x < 0 else 0)

# Network Configuration
HOST = '0.0.0.0'
WHEEL_PORT = 8000
CAMERA_PORT = 8001
PID_CONFIG_PORT = 8002

# Pins
RIGHT_MOTOR_ENA = 18
RIGHT_MOTOR_IN1 = 17
RIGHT_MOTOR_IN2 = 27
LEFT_MOTOR_ENB = 25
LEFT_MOTOR_IN3 = 23
LEFT_MOTOR_IN4 = 24
LEFT_ENCODER = 26
RIGHT_ENCODER = 16

# PID Constants (default values, will be overridden by client)
use_PID = 0.0
KP, KI, KD = 0.0, 0.0, 0.0      # straight-line PID
KP_R, KI_R, KD_R = 0.0, 0.0, 0.0 # turn (pivot) PID
MAX_CORRECTION = 30  # Maximum PWM correction value

# Global variables
running = True
left_pwm, right_pwm = 0.0, 0.0
left_count, right_count = 0, 0
prev_left_state, prev_right_state = None, None
use_ramping = True
RAMP_RATE = 250  # PWM units per second
MIN_RAMP_THRESHOLD = 15  # Only ramp if change is greater than this
MIN_PWM_THRESHOLD = 15
current_movement, prev_movement = 'stop', 'stop'

# NEW: cap straight-mode differential so you never get 51/15-type splits
STRAIGHT_MAX_DELTA = 12.0  # per side; raise to 6–8 if you need more authority
LEFT_TRIM  = 1.00
RIGHT_TRIM = 1.10   # try 1.03..1.10 depending on how bad the right is

# NEW: encoder dead-time (filters bounce/EMI bursts that cause sudden count jumps)
ENC_DEADTIME_NS = 1_000_000  # 1.0 ms; tune 0.5–3.0 ms as needed
_left_last_ns = 0
_right_last_ns = 0

def setup_gpio():
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    
    # Motor pins
    GPIO.setup(RIGHT_MOTOR_ENA, GPIO.OUT)
    GPIO.setup(RIGHT_MOTOR_IN1, GPIO.OUT)
    GPIO.setup(RIGHT_MOTOR_IN2, GPIO.OUT)
    GPIO.setup(LEFT_MOTOR_ENB, GPIO.OUT)
    GPIO.setup(LEFT_MOTOR_IN3, GPIO.OUT)
    GPIO.setup(LEFT_MOTOR_IN4, GPIO.OUT)
    
    # Prevent slight motor jerk when connection is established
    GPIO.output(RIGHT_MOTOR_ENA, GPIO.LOW)
    GPIO.output(LEFT_MOTOR_ENB, GPIO.LOW)
    
    # Encoder setup and interrupts
    GPIO.setup(LEFT_ENCODER, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(RIGHT_ENCODER, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    # Keep BOTH (no change to your API), but callbacks implement dead-time filter
    GPIO.add_event_detect(LEFT_ENCODER, GPIO.BOTH, callback=left_encoder_callback)
    GPIO.add_event_detect(RIGHT_ENCODER, GPIO.BOTH, callback=right_encoder_callback)
    
    # Initialize PWM (frequency: 100Hz)
    global left_motor_pwm, right_motor_pwm
    left_motor_pwm = GPIO.PWM(LEFT_MOTOR_ENB, 100)
    right_motor_pwm = GPIO.PWM(RIGHT_MOTOR_ENA, 100)
    left_motor_pwm.start(0)
    right_motor_pwm.start(0)

def left_encoder_callback(channel):
    global left_count, prev_left_state, _left_last_ns
    now = time.monotonic_ns()
    if now - _left_last_ns < ENC_DEADTIME_NS:
        return  # ignore bounce / too-fast edges
    current_state = GPIO.input(LEFT_ENCODER)
    # Count only on state changes (reduces double-count on BOTH)
    if (prev_left_state is not None and current_state != prev_left_state):
        left_count += 1
        prev_left_state = current_state
        _left_last_ns = now
    elif prev_left_state is None:
        prev_left_state = current_state
        _left_last_ns = now

def right_encoder_callback(channel):
    global right_count, prev_right_state, _right_last_ns
    now = time.monotonic_ns()
    if now - _right_last_ns < ENC_DEADTIME_NS:
        return
    current_state = GPIO.input(RIGHT_ENCODER)
    if (prev_right_state is not None and current_state != prev_right_state):
        right_count += 1
        prev_right_state = current_state
        _right_last_ns = now
    elif prev_right_state is None:
        prev_right_state = current_state
        _right_last_ns = now

def reset_encoder():
    global left_count, right_count
    left_count, right_count = 0, 0

def set_motors(left, right):
    global prev_movement, current_movement
    
    # Pre-start kick to reduce initial jerk/orientation change (unchanged)
    if prev_movement == 'stop' and current_movement in ['forward', 'backward']:
        if current_movement == 'forward':
            GPIO.output(RIGHT_MOTOR_IN1, GPIO.HIGH)
            GPIO.output(RIGHT_MOTOR_IN2, GPIO.LOW)
            GPIO.output(LEFT_MOTOR_IN3, GPIO.HIGH)
            GPIO.output(LEFT_MOTOR_IN4, GPIO.LOW)
        elif current_movement == 'backward':
            GPIO.output(RIGHT_MOTOR_IN1, GPIO.LOW)
            GPIO.output(RIGHT_MOTOR_IN2, GPIO.HIGH)
            GPIO.output(LEFT_MOTOR_IN3, GPIO.LOW)
            GPIO.output(LEFT_MOTOR_IN4, GPIO.HIGH)
        left_motor_pwm.ChangeDutyCycle(100)
        right_motor_pwm.ChangeDutyCycle(100)
        time.sleep(0.05)

    # Right side
    if right > 0:
        GPIO.output(RIGHT_MOTOR_IN1, GPIO.HIGH)
        GPIO.output(RIGHT_MOTOR_IN2, GPIO.LOW)
        right_motor_pwm.ChangeDutyCycle(min(right, 100))
    elif right < 0:
        GPIO.output(RIGHT_MOTOR_IN1, GPIO.LOW)
        GPIO.output(RIGHT_MOTOR_IN2, GPIO.HIGH)
        right_motor_pwm.ChangeDutyCycle(min(abs(right), 100))
    else:
        # Active braking when pwm=0
        GPIO.output(RIGHT_MOTOR_IN1, GPIO.HIGH)
        GPIO.output(RIGHT_MOTOR_IN2, GPIO.HIGH)
        right_motor_pwm.ChangeDutyCycle(100)
    
    # Left side
    if left > 0:
        GPIO.output(LEFT_MOTOR_IN3, GPIO.HIGH)
        GPIO.output(LEFT_MOTOR_IN4, GPIO.LOW)
        left_motor_pwm.ChangeDutyCycle(min(left, 100))
    elif left < 0:
        GPIO.output(LEFT_MOTOR_IN3, GPIO.LOW)
        GPIO.output(LEFT_MOTOR_IN4, GPIO.HIGH)
        left_motor_pwm.ChangeDutyCycle(min(abs(left), 100))
    else:
        GPIO.output(LEFT_MOTOR_IN3, GPIO.HIGH)
        GPIO.output(LEFT_MOTOR_IN4, GPIO.HIGH)
        left_motor_pwm.ChangeDutyCycle(100)

def apply_min_threshold(pwm_value, min_threshold):
    if pwm_value == 0:
        return 0  # Zero means stop
    elif abs(pwm_value) < min_threshold:
        return min_threshold if pwm_value > 0 else -min_threshold
    else:
        return pwm_value

# ----------------------
# PID control loop (with straight differential clamp + encoder dead-time)
# ----------------------
def pid_control():
    global left_pwm, right_pwm, left_count, right_count
    global use_PID, KP, KI, KD, KP_R, KI_R, KD_R, prev_movement, current_movement

    # Straight PI state
    integral = 0.0
    last_error = 0.0

    # Turn PID state
    integral_turn = 0.0
    last_error_turn = 0.0

    # Smoothed error (to avoid 1-tick flip-flop)
    ema_err = 0.0
    # ~60 ms time constant with 10–20 ms loop
    EMA_ALPHA = 0.25

    lastL = left_count
    lastR = right_count
    last_time = monotonic()

    # Ramping for the COMMON/base speed only
    ramp_base = 0.0
    prev_base_target = 0.0

    while running:
        now = monotonic()
        dt = now - last_time if now > last_time else 1e-6
        last_time = now

        prev_movement = current_movement
        ls, rs = sgn(left_pwm), sgn(right_pwm)
        if ls == 0 and rs == 0:
            current_movement = 'stop'
        elif ls == rs and ls != 0:
            current_movement = 'forward' if ls > 0 else 'backward'
        else:
            current_movement = 'turn'

        if current_movement != prev_movement:
            integral = integral_turn = 0.0
            last_error = last_error_turn = 0.0

        # Start with requested commands
        cmd_L = left_pwm
        cmd_R = right_pwm

        if use_PID:
            if current_movement in ('forward', 'backward'):
                # Equal magnitudes (no arc command)
                base_mag = min(abs(cmd_L), abs(cmd_R))
                sign_dir = 1 if current_movement == 'forward' else -1

                # Read encoder deltas
                curL, curR = left_count, right_count
                dL, dR = curL - lastL, curR - lastR
                lastL, lastR = curL, curR

                # Smooth the 1-tick noise
                raw_err = (dL - dR) * (1 if sign_dir > 0 else -1)
                # Flip sign when going backward so "positive = left faster" stays consistent
                ema_err = (1.0 - EMA_ALPHA) * ema_err + EMA_ALPHA * raw_err

                # PI(D) correction
                proportional = KP * ema_err
                integral += KI * ema_err * dt
                # Anti-windup
                integral = max(-STRAIGHT_MAX_DELTA, min(integral, STRAIGHT_MAX_DELTA))
                derivative = KD * (ema_err - last_error) / dt if dt > 0 else 0.0
                corr = proportional + integral + derivative
                last_error = ema_err

                # Headroom around base to stay within [MIN,100]
                span_min = max(0.0, base_mag - MIN_PWM_THRESHOLD)
                span_max = max(0.0, 100.0 - base_mag)
                headroom = min(span_min, span_max)
                delta_cap = min(headroom, STRAIGHT_MAX_DELTA)

                # Clamp differential correction
                if corr > 0:   # left is faster → slow left, boost right
                    corr = min(corr, delta_cap)
                else:
                    corr = max(corr, -delta_cap)

                # === RAMP COMMON SPEED ONLY ===
                target_base = base_mag * sign_dir
                max_change = RAMP_RATE * dt
                if abs(target_base - ramp_base) <= max_change:
                    ramp_base = target_base
                else:
                    ramp_base += max_change if (target_base > ramp_base) else -max_change

                # Apply instantaneous differential around the ramped base
                target_left_pwm  = (ramp_base - corr)
                target_right_pwm = (ramp_base + corr)

            elif current_movement == 'turn':
                # Make a clean pivot: equal-and-opposite magnitudes
                base = max(abs(cmd_L), abs(cmd_R))
                t_ls, t_rs = sgn(cmd_L), sgn(cmd_R)
                if t_ls == 0 and t_rs != 0: t_ls = -t_rs
                if t_rs == 0 and t_ls != 0: t_rs = -t_ls
                if t_ls == t_rs and t_ls != 0: t_rs = -t_ls

                target_left_pwm  = base * t_ls
                target_right_pwm = base * t_rs

                # Pivot PID on |ticks|
                curL, curR = left_count, right_count
                dL, dR = curL - lastL, curR - lastR
                lastL, lastR = curL, curR
                turn_err = abs(dL) - abs(dR)

                p_t = KP_R * turn_err
                integral_turn += KI_R * turn_err * dt
                integral_turn = max(-MAX_CORRECTION, min(integral_turn, MAX_CORRECTION))
                d_t = KD_R * (turn_err - last_error_turn) / dt if dt > 0 else 0.0
                corr_t = max(-MAX_CORRECTION, min(p_t + integral_turn + d_t, MAX_CORRECTION))
                last_error_turn = turn_err

                target_left_pwm  -= corr_t
                target_right_pwm += corr_t

            else:
                # stop
                integral = integral_turn = 0.0
                last_error = last_error_turn = 0.0
                ramp_base = 0.0
                reset_encoder()
                target_left_pwm = 0.0
                target_right_pwm = 0.0
        else:
            # No PID: just ramp the common speed a bit for smoothness
            base_mag = min(abs(cmd_L), abs(cmd_R))
            sign_dir = sgn(cmd_L + cmd_R) or 1
            target_base = base_mag * sign_dir
            max_change = RAMP_RATE * dt
            if abs(target_base - ramp_base) <= max_change:
                ramp_base = target_base
            else:
                ramp_base += max_change if (target_base > ramp_base) else -max_change
            # Keep original differential (teleop)
            diff = (cmd_R - cmd_L) / 2.0
            target_left_pwm  = ramp_base - diff
            target_right_pwm = ramp_base + diff

        # Apply optional static trim before thresholding
        target_left_pwm  *= LEFT_TRIM
        target_right_pwm *= RIGHT_TRIM

        # Enforce minimum PWM threshold (preserve sign)
        final_left_pwm  = apply_min_threshold(target_left_pwm,  MIN_PWM_THRESHOLD)
        final_right_pwm = apply_min_threshold(target_right_pwm, MIN_PWM_THRESHOLD)

        set_motors(final_left_pwm, final_right_pwm)

        if final_left_pwm != 0 or final_right_pwm != 0:
            print(f"(Left PWM, Right PWM)=({final_left_pwm:.2f},{final_right_pwm:.2f}), "
                  f"(Left Enc, Right Enc)=({left_count}, {right_count}), mode={current_movement}")
        time.sleep(0.01)

# ----------------------
# Camera streaming server
# ----------------------
def camera_stream_server():
    # Initialize camera
    picam2 = Picamera2()
    camera_config = picam2.create_preview_configuration(lores={"size": (640, 480)})
    picam2.configure(camera_config)
    picam2.start()
    
    # Create socket for streaming
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
                # Capture frame and convert to bytes
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
                
                # Small delay to avoid hogging CPU
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
                # Receive 7 floats: use_PID, KP, KI, KD, KP_R, KI_R, KD_R (network byte order)
                data = recv_all(client_socket, 28)
                if data and len(data) == 28:
                    use_PID, KP, KI, KD, KP_R, KI_R, KD_R = struct.unpack("!fffffff", data)
                    if use_PID:
                        print(f"Updated PID constants: KP={KP}, KI={KI}, KD={KD}, KP_R={KP_R}, KI_R={KI_R}, KD_R={KD_R}")
                    else:
                        print("The robot is not using PID.")
                    response = struct.pack("!i", 1)
                else:
                    response = struct.pack("!i", 0)
                client_socket.sendall(response)
                    
            except Exception as e:
                print(f"PID config socket error: {str(e)}")
                try:
                    response = struct.pack("!i", 0)
                    client_socket.sendall(response)
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
                    # Receive two floats (left_speed, right_speed) in range [-1,1]
                    data = recv_all(client_socket, 8)
                    if not data:
                        print("Wheel client disconnected during recv")
                        break
                    
                    left_speed, right_speed = struct.unpack("!ff", data)
                    print(f"Received wheel: left_speed={left_speed:.4f}, right_speed={right_speed:.4f}")
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
        
        # Start PID control thread
        pid_thread = threading.Thread(target=pid_control, daemon=True)
        pid_thread.start()
        
        # Start camera streaming thread
        camera_thread = threading.Thread(target=camera_stream_server, daemon=True)
        camera_thread.start()
        
        # Start PID configuration server thread
        pid_config_thread = threading.Thread(target=pid_config_server, daemon=True)
        pid_config_thread.start()
        
        # Start wheel server (main thread)
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
