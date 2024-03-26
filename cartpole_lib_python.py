import odrive
import numpy as np
import time

##### CONSTANTS #####
SHOULDER_CPR = 4096
# SHOULDER_CPR = 2400
LINEAR_CPR = 2400

SHOULDER_TO_ANGLE_RATIO = (1/SHOULDER_CPR) * (2*np.pi)
SHOULDER_OFFSET = -np.pi/2 # account for the pole initially being straight down
LINEAR_TO_ANGLE_RATIO = (1/60500) # returns meters
LINEAR_OFFSET = 0
ODRIVE_OVERFLOW_COMP = 2**14
UINT16_MAX = 2**16

# enums for odrive states
AXIS_ENCODER_OFFSET_CALIBRATION = 7
AXIS_STATE_CLOSED_LOOP_CONTROL = 8

SHOULDER_VEL_LPF = 0.6 # higher is less filtering
LINEAR_VEL_LPF = 0.8 # higher is less filtering

##### GLOBAL VARS #####

odrv0 = None
last_shoulder_pos = 0
last_linear_pos = 0
last_state_time = 0
last_linear_vel = 0
last_shoulder_vel = 0

last_encoder_val_shoulder = 0
last_encoder_val_rail = 0

rail_encoder = 0
shoulder_encoder = 0

##### PRIVATE FUNCTIONS #####

# handle the fact that the odrive encoder initializes at 0 and won't let you set an offset
def get_rail_encoder_pos():
    global odrv0, last_encoder_val_rail, rail_encoder


    # # When using incremental encoder
    # encoder_val = odrv0.inc_encoder0.raw

    # if last_encoder_val_rail < ODRIVE_OVERFLOW_COMP and encoder_val > UINT16_MAX - ODRIVE_OVERFLOW_COMP:
    #     diff = -(UINT16_MAX - encoder_val + last_encoder_val_rail)
    # elif last_encoder_val_rail > UINT16_MAX - ODRIVE_OVERFLOW_COMP and encoder_val < ODRIVE_OVERFLOW_COMP:
    #     diff = UINT16_MAX - last_encoder_val_rail + encoder_val 
    # else:
    #     diff = encoder_val - last_encoder_val_rail
    
    # last_encoder_val_rail = encoder_val

    # rail_encoder -= diff
    
    # # return (rail_encoder * LINEAR_TO_ANGLE_RATIO) + LINEAR_OFFSET
    # return rail_encoder * 0.11938052083 # 2*pi*r = 2*pi*(19mm) = 2*pi*0.019
    
    
    # When using onboard encoder (magnetic)
    encoder_val = odrv0.onboard_encoder0.raw
    
    if last_encoder_val_rail < .25 and encoder_val >= 1 - .25:
        diff = -(1 - encoder_val + last_encoder_val_rail)
    elif last_encoder_val_rail > 1 - .25 and encoder_val <= .25:
        diff = 1 - last_encoder_val_rail + encoder_val
    else:
        diff = encoder_val - last_encoder_val_rail
    
    last_encoder_val_rail = encoder_val
    
    rail_encoder -= diff
    
    return rail_encoder * 0.11938052083 # 2*pi*r = 2*pi*(19mm) = 2*pi*0.019


def get_shoulder_encoder_pos():
    global odrv0, last_encoder_val_shoulder, shoulder_encoder

    encoder_val = odrv0.inc_encoder1.raw

    if last_encoder_val_shoulder < ODRIVE_OVERFLOW_COMP and encoder_val > UINT16_MAX - ODRIVE_OVERFLOW_COMP:
        diff = -(UINT16_MAX - encoder_val + last_encoder_val_shoulder)
    elif last_encoder_val_shoulder > UINT16_MAX - ODRIVE_OVERFLOW_COMP and encoder_val < ODRIVE_OVERFLOW_COMP:
        diff = UINT16_MAX - last_encoder_val_shoulder + encoder_val 
    else:
        diff = encoder_val - last_encoder_val_shoulder

    last_encoder_val_shoulder = encoder_val
    
    shoulder_encoder += diff 

    return ((shoulder_encoder * SHOULDER_TO_ANGLE_RATIO) + SHOULDER_OFFSET) % (2*np.pi)

##### PUBLIC FUNCTIONS #####

def get_odrive():
    global odrv0
    return odrv0

def init_odrive_no_calib():
    global odrv0
    
    odrv0 = odrive.find_any()

def init_odrive():
    global odrv0, last_shoulder_pos, last_linear_pos, last_state_time

    odrv0 = odrive.find_any()
    odrv0.clear_errors()

    odrive.utils.dump_errors(odrv0)
    
    
    # When using onboard encoder (magnetic)
    # set absolute encoder reference frame (primarily for anticogging)
    # odrv0.axis0.pos_estimate = odrv0.onboard_encoder0.raw
    # odrv0.axis0.config.anticogging.enabled = True

    # # When using incremental encoder
    # odrv0.axis0.requested_state = AXIS_ENCODER_OFFSET_CALIBRATION
    # while odrv0.axis0.current_state == AXIS_ENCODER_OFFSET_CALIBRATION:
    #     time.sleep(.1)
    # # time.sleep(10)

    # odrive.utils.dump_errors(odrv0)

    odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    time.sleep(0.5)
    
    # # odrv0.axis0.controller.config.control_mode = ControlMode.TORQUE_CONTROL
    # # time.sleep(0.5)

    # get_shoulder_encoder_pos()
    # get_rail_encoder_pos()
    # last_state_time = time.time()

    # time.sleep(0.5)

    odrive.utils.dump_errors(odrv0)
    
    print("odrive initialized")

def get_errors():
    global odrv0

    return odrive.utils.dump_errors(odrv0)

def get_measurement():
    global odrv0
    
    shoulder_pos = get_shoulder_encoder_pos()
    linear_pos = get_rail_encoder_pos()
    
    return np.array([linear_pos, shoulder_pos])

def get_state():
    global odrv0, last_shoulder_pos, last_linear_pos, last_shoulder_vel, last_linear_vel, last_state_time

    enc_start = time.time()
    shoulder_pos = get_shoulder_encoder_pos()
    linear_pos = get_rail_encoder_pos()
    enc_end = time.time()

    enc_capture_time = (enc_end + enc_start) / 2
    elapsed_time = time.time() - last_state_time
    if elapsed_time == 0:
        elapsed_time = .01

    shoulder_velocity = (shoulder_pos - last_shoulder_pos) / elapsed_time
    shoulder_velocity = (shoulder_velocity * SHOULDER_VEL_LPF) + (last_shoulder_vel * (1 - SHOULDER_VEL_LPF))

    linear_velocity = (linear_pos - last_linear_pos) / elapsed_time
    linear_velocity = (linear_velocity * LINEAR_VEL_LPF) + (last_linear_vel * (1 - LINEAR_VEL_LPF))

    last_linear_vel = linear_velocity
    last_shoulder_vel = shoulder_velocity
    last_state_time = enc_capture_time
    last_shoulder_pos = shoulder_pos
    last_linear_pos = linear_pos
    last_state_time = enc_capture_time 

    return [linear_pos, shoulder_pos, linear_velocity, shoulder_velocity]

# why busy sleeping is better: https://stackoverflow.com/questions/1133857/how-accurate-is-pythons-time-sleep
def busy_sleep(duration, loop_start = None, get_now=time.time):
    now = get_now()

    if loop_start is None:
        end = now + duration
    else: 
        end = loop_start + duration

    while now < end:
        now = get_now()

def command_linear_torque(input_torque):
    global odrv0

    odrv0.axis0.controller.input_torque = input_torque


############################### OLD LIB ###############################

# import serial
# import serial.tools.list_ports
# import time

# cartpole_encoder_data_fields = [
# "linear_position", 
# "linear_velocity", 
# "shoulder_position", 
# "shoulder_velocity", 
# "elbow_position", 
# "elbow_velocity",
# "requested_torque_1",
# "requested_torque_2",
# # "serial_tx_time",
# # "time_since_last_command",
# ]


# def cartpole_open_serial(sp, baud = 115200):
#     local_sp = serial.Serial(sp, baud, timeout=1)
#     local_sp.flush()
#     return local_sp


# def cartpole_flush_serial(local_sp):
#     local_sp.flush()
#     local_sp.flushInput()
#     local_sp.read_all()

# # inputs are in nm
# def cartpole_write_motors(local_sp, linear_torque, elbow_torque):
#     # max prescision of 3 decimal places
#     linear_vel = round(linear_torque, 3) 
#     elbow_vel = round(elbow_torque, 3)
#     string_to_write = "<" + str(linear_vel) + ", " + str(elbow_vel) + ">"

#     timeout = 10
#     while(timeout > 0):
#         try:
#             local_sp.write(string_to_write.encode())
#             # local_sp.flush()
#             return
#         except:
#             timeout -= 1
#     raise Exception("Timeout during Cartpole Serial Write")

# # timeout in attempt counts, default 10, best not to change
# def cartpole_safe_read(local_sp):
#     timeout = 10
#     while(timeout > 0):
#         encoderData = local_sp.readline()
#         try: # catch byte errors in encoderData.decode
#             encoderData = encoderData.decode()
#             encoderData = encoderData.split(",")
#             if len(encoderData) - 1 == len(cartpole_encoder_data_fields): # account for the \r\n 
#                 encoderDataConverted = [float(x) for x in encoderData[:-1]]
#                 return encoderDataConverted
#         except Exception as e:
#             timeout -= 1

#     raise Exception("Timeout during Cartpole Serial Read")


# def cartpole_unsafe_read(local_sp):
#     encoderData = local_sp.readline()
#     encoderData = encoderData.decode()
#     encoderData = encoderData.split(",")
#     return encoderData[:-1]



# def cartpole_close_serial(local_sp):
#     local_sp.close()

# # why bother busy sleeping: 
# # https://stackoverflow.com/questions/1133857/how-accurate-is-pythons-time-sleep
# def cartpole_busy_sleep(duration, loop_start = None, get_now=time.time):
#     now = get_now()

#     if loop_start is None:
#         end = now + duration
#     else: 
#         end = duration - (get_now() - loop_start)

#     while now < end:
#         now = get_now()