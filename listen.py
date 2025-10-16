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
    GPIO.add_event_detect(LEFT_ENCODER, GPIO.BOTH, callback=left_encoder_callback)
    GPIO.add_event_detect(RIGHT_ENCODER, GPIO.BOTH, callback=right_encoder_callback)
    
    # Initialize PWM (frequency: 100Hz)
    global left_motor_pwm, right_motor_pwm
    left_motor_pwm = GPIO.PWM(LEFT_MOTOR_ENB, 100)
    right_motor_pwm = GPIO.PWM(RIGHT_MOTOR_ENA, 100)
    left_motor_pwm.start(0)
    right_motor_pwm.start(0)

def left_encoder_callback(channel):
    global left_count, prev_left_state
    current_state = GPIO.input(LEFT_ENCODER)
    # Count only on state changes (reduces false positives)
    if (prev_left_state is not None and current_state != prev_left_state):
        left_count += 1
        prev_left_state = current_state
    elif prev_left_state is None:
        prev_left_state = current_state

def right_encoder_callback(channel):
    global right_count, prev_right_state
    current_state = GPIO.input(RIGHT_ENCODER)
    if (prev_right_state is not None and current_state != prev_right_state):
        right_count += 1
        prev_right_state = current_state
    elif prev_right_state is None:
        prev_right_state = current_state

def reset_encoder():
    global left_count, right_count
    left_count, right_count = 0, 0

def set_motors(left, right):
    global prev_movement, current_movement
    
    # Pre-start kick to reduce initial jerk/orientation change
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
# PID control loop (arc driving removed)
# ----------------------
def pid_control():
    global left_pwm, right_pwm, left_count, right_count
    global use_PID, KP, KI, KD, KP_R, KI_R, KD_R, prev_movement, current_movement

    # Linear PID state
    integral = 0.0
    last_error = 0.0

    # Turn PID state
    integral_turn = 0.0
    last_error_turn = 0.0

    lastL = left_count
    lastR = right_count
    
    last_time = monotonic()

    # Ramping variables
    ramp_left_pwm = 0.0
    ramp_right_pwm = 0.0
    previous_left_target = 0.0
    previous_right_target = 0.0

    while running:
        current_time = monotonic()
        dt = current_time - last_time if current_time - last_time > 0 else 1e-6
        last_time = current_time

        prev_movement = current_movement

        # Movement mode decided ONLY by signs
        ls, rs = sgn(left_pwm), sgn(right_pwm)
        if ls == 0 and rs == 0:
            current_movement = 'stop'
        elif ls == rs and ls != 0:
            current_movement = 'forward' if ls > 0 else 'backward'
        else:
            current_movement = 'turn'

        # Start with the requested commands
        target_left_pwm = left_pwm
        target_right_pwm = right_pwm

        if use_PID:
            if current_movement in ('forward', 'backward'):
                # --- FORCE STRAIGHT: equal magnitudes (no arcs) ---
                base = min(abs(left_pwm), abs(right_pwm))
                sign_dir = 1 if current_movement == 'forward' else -1
                target_left_pwm  = base * sign_dir
                target_right_pwm = base * sign_dir

                # Straight-line PID: equalize encoder ticks (dL == dR)
                curL, curR = left_count, right_count
                dL, dR = curL - lastL, curR - lastR
                error = dL - dR

                proportional = KP * error
                integral += KI * error * dt
                integral = max(-MAX_CORRECTION, min(integral, MAX_CORRECTION))  # Anti-windup
                derivative = KD * (error - last_error) / dt if dt > 0 else 0.0
                correction = proportional + integral + derivative
                correction = max(-MAX_CORRECTION, min(correction, MAX_CORRECTION))
                last_error = error

                # Flip correction when going backward
                if current_movement == 'backward':
                    correction = -correction

                target_left_pwm  -= correction
                target_right_pwm += correction

                lastL, lastR = curL, curR

            elif current_movement == 'turn':
                # Normalize to equal and opposite magnitudes for a clean pivot
                base = max(abs(left_pwm), abs(right_pwm))
                target_left_pwm  = -base
                target_right_pwm =  base

                # Pivot PID: |dL| == |dR| with opposite signs -> dL + dR = 0
                curL, curR = left_count, right_count
                dL, dR = curL - lastL, curR - lastR
                turn_error = dL + dR

                proportional_t = KP_R * turn_error
                integral_turn += KI_R * turn_error * dt
                integral_turn = max(-MAX_CORRECTION, min(integral_turn, MAX_CORRECTION))
                derivative_t = KD_R * (turn_error - last_error_turn) / dt if dt > 0 else 0.0
                correction_turn = proportional_t + integral_turn + derivative_t
                correction_turn = max(-MAX_CORRECTION, min(correction_turn, MAX_CORRECTION))
                last_error_turn = turn_error
                lastL, lastR = curL, curR

                target_left_pwm  -= correction_turn
                target_right_pwm += correction_turn

            else:
                # Stopped: reset PID states and encoders
                integral = 0.0
                last_error = 0.0
                integral_turn = 0.0
                last_error_turn = 0.0
                reset_encoder()
                target_left_pwm = 0.0
                target_right_pwm = 0.0

        # Ramping (unchanged)
        if use_ramping and use_PID:
            max_change_per_cycle = RAMP_RATE * dt

            left_diff = target_left_pwm - ramp_left_pwm
            right_diff = target_right_pwm - ramp_right_pwm

            left_needs_ramp = abs(left_diff) > MIN_RAMP_THRESHOLD
            right_needs_ramp = abs(right_diff) > MIN_RAMP_THRESHOLD

            left_direction_change = (target_left_pwm * previous_left_target < 0) and target_left_pwm != 0 and previous_left_target != 0
            right_direction_change = (target_right_pwm * previous_right_target < 0) and target_right_pwm != 0 and previous_right_target != 0

            if left_direction_change:
                ramp_left_pwm = target_left_pwm
            if right_direction_change:
                ramp_right_pwm = target_right_pwm

            if not left_direction_change and not right_direction_change:
                if left_needs_ramp or right_needs_ramp:
                    # Left motor ramping
                    if abs(left_diff) <= max_change_per_cycle:
                        ramp_left_pwm = target_left_pwm
                    else:
                        ramp_left_pwm += max_change_per_cycle if left_diff > 0 else -max_change_per_cycle

                    # Right motor ramping
                    if abs(right_diff) <= max_change_per_cycle:
                        ramp_right_pwm = target_right_pwm
                    else:
                        ramp_right_pwm += max_change_per_cycle if right_diff > 0 else -max_change_per_cycle
                else:
                    ramp_left_pwm = target_left_pwm
                    ramp_right_pwm = target_right_pwm

            previous_left_target = target_left_pwm
            previous_right_target = target_right_pwm

        else:
            # No ramping: apply target directly
            ramp_left_pwm = target_left_pwm
            ramp_right_pwm = target_right_pwm

        final_left_pwm = apply_min_threshold(ramp_left_pwm, MIN_PWM_THRESHOLD)
        final_right_pwm = apply_min_threshold(ramp_right_pwm, MIN_PWM_THRESHOLD)
        set_motors(final_left_pwm, final_right_pwm)

        # Optional debug print
        if ramp_left_pwm != 0 or ramp_right_pwm != 0:
            print(f"(Left PWM, Right PWM)=({ramp_left_pwm:.2f},{ramp_right_pwm:.2f}), "
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
