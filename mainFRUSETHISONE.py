from machine import Pin
from time import sleep
from math import pi
from motor_driver import MotorDriver
from encoded_motor_driver import EncodedMotorDriver

# ===========================================================================
# 1. CONFIGURATION & CONSTANTS
# ===========================================================================

# --- PINOUT ---
STBY_PIN = 12
LEFT_DRIVER_PINS  = (15, 13, 14) 
LEFT_ENC_PINS     = (11, 10)     
RIGHT_DRIVER_PINS = (16, 18, 17) 
RIGHT_ENC_PINS    = (20, 19)     

# --- ROBOT PHYSICAL SPECS ---
# If your robot consistently under-drives, slightly DECREASE Wheel Diameter here
WHEEL_DIAMETER = 0.035          # 35 mm
GEAR_RATIO     = 98.5
ENCODER_CPR    = 12             # Counts Per Revolution (Motor shaft)
# If your robot consistently under-turns, slightly DECREASE Axle Length here
AXLE_LENGTH    = 0.150          # 150 mm

# --- CALCULATED VALUES ---
COUNTS_PER_WHEEL_REV = ENCODER_CPR * GEAR_RATIO 
DIST_PER_COUNT = (pi * WHEEL_DIAMETER) / COUNTS_PER_WHEEL_REV 

# --- TUNING ---
BASE_SPEED = 0.25   # Speed ~25%
SYNC_KP    = 0.01   # Correction strength (keep small for smooth driving)

# ===========================================================================
# 2. OBJECT SETUP
# ===========================================================================

stby = Pin(STBY_PIN, Pin.OUT)
stby.value(1) # Enable driver

left_motor  = EncodedMotorDriver(LEFT_DRIVER_PINS, LEFT_ENC_PINS)
right_motor = EncodedMotorDriver(RIGHT_DRIVER_PINS, RIGHT_ENC_PINS)

def stop_motors():
    left_motor.stop()
    right_motor.stop()

def reset_encoders():
    left_motor.reset_encoder_counts()
    right_motor.reset_encoder_counts()

# ===========================================================================
# 3. MOTION FUNCTIONS
# ===========================================================================

def drive_straight(distance_m, wait_after=0.5):
    """
    Drive straight for distance_m.
    Includes simple P-Control to keep robot straight.
    """
    print(f"Driving straight: {distance_m:.3f} m")
    reset_encoders()
    
    target_counts = int(distance_m / DIST_PER_COUNT)
    
    while True:
        l_cnt = abs(left_motor.encoder_counts)
        r_cnt = abs(right_motor.encoder_counts)
        avg   = (l_cnt + r_cnt) // 2

        if avg >= target_counts:
            break

        # Straight Correction
        error = l_cnt - r_cnt
        correction = error * SYNC_KP
        
        left_motor.forward(BASE_SPEED - correction)
        right_motor.forward(BASE_SPEED + correction)
        
        sleep(0.01)

    stop_motors()
    sleep(wait_after)

def spin_turn(degrees_deg, wait_after=0.5):
    """
    Spin in place. 
    Positive = CCW (Left)
    Negative = CW (Right)
    """
    print(f"Spinning: {degrees_deg} deg")
    reset_encoders()
    
    # Calculate Arc Length
    rads = abs(degrees_deg) * (pi / 180.0)
    arc_length = (AXLE_LENGTH / 2) * rads
    target_counts = int(arc_length / DIST_PER_COUNT)

    while True:
        l_cnt = abs(left_motor.encoder_counts)
        r_cnt = abs(right_motor.encoder_counts)
        avg   = (l_cnt + r_cnt) // 2

        if avg >= target_counts:
            break
        
        # Sync Correction during turn
        error = l_cnt - r_cnt
        correction = error * SYNC_KP
        
        speed_l = BASE_SPEED - correction
        speed_r = BASE_SPEED + correction

        if degrees_deg > 0:
            # CCW: Left Back, Right Fwd
            left_motor.backward(speed_l)
            right_motor.forward(speed_r)
        else:
            # CW: Left Fwd, Right Back
            left_motor.forward(speed_l)
            right_motor.backward(speed_r)
            
        sleep(0.01)

    stop_motors()
    sleep(wait_after)

# ===========================================================================
# 4. MAIN TRAJECTORY (The Updated Values)
# ===========================================================================

if __name__ == "__main__":
    try:
        print("=== STARTED TRAIL FOLLOWING ===")
        sleep(2) 

        # --- LEG 1 ---
        # Increased to 0.95m
        drive_straight(1.3, wait_after=3.0)

        # --- TURN 1 (CCW) ---
        # Increased to 105 degrees
        spin_turn(125, wait_after=1.0)

        # --- LEG 2 ---
        # Increased to 0.70m
        drive_straight(0.85, wait_after=3.0)

        # --- TURN 2 (CCW) ---
        # Increased to 105 degrees
        # (Assuming you are doing the box shape. Change to negative if you need Clockwise)
        spin_turn(140, wait_after=1.0) 

        # --- LEG 3 ---
        # Increased to 0.70m
        drive_straight(0.90, wait_after=3.0)

        # --- TURN 3 & LEG 4 (Closing Loop) ---
        # Increased turn to 80 deg, increased leg to 0.80m
        spin_turn(115, wait_after=1.0)
        drive_straight(1.1, wait_after=1.0)

        print("=== MISSION COMPLETE ===")

    except KeyboardInterrupt:
        print("\nSTOPPED BY USER")
    
    finally:
        stop_motors()
        stby.value(0)
        print("Motors Disabled.")