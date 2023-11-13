import serial
import serial.tools.list_ports
import time

cartpole_encoder_data_fields = [
"linear_position", 
"linear_velocity", 
"shoulder_position", 
"shoulder_velocity", 
"elbow_position", 
"elbow_velocity",
"requested_torque_1",
"requested_torque_2",
# "serial_tx_time",
# "time_since_last_command",
]


def cartpole_open_serial(sp, baud = 115200):
    local_sp = serial.Serial(sp, baud, timeout=1)
    local_sp.flush()
    return local_sp


def cartpole_flush_serial(local_sp):
    local_sp.flush()
    local_sp.flushInput()
    local_sp.read_all()

# inputs are in nm
def cartpole_write_motors(local_sp, linear_torque, elbow_torque):
    # max prescision of 3 decimal places
    linear_vel = round(linear_torque, 3) 
    elbow_vel = round(elbow_torque, 3)
    string_to_write = "<" + str(linear_vel) + ", " + str(elbow_vel) + ">"

    timeout = 10
    while(timeout > 0):
        try:
            local_sp.write(string_to_write.encode())
            # local_sp.flush()
            return
        except:
            timeout -= 1
    raise Exception("Timeout during Cartpole Serial Write")

# timeout in attempt counts, default 10, best not to change
def cartpole_safe_read(local_sp):
    timeout = 10
    while(timeout > 0):
        encoderData = local_sp.readline()
        try: # catch byte errors in encoderData.decode
            encoderData = encoderData.decode()
            encoderData = encoderData.split(",")
            if len(encoderData) - 1 == len(cartpole_encoder_data_fields): # account for the \r\n 
                return encoderData[:-1]
        except Exception as e:
            timeout -= 1

    raise Exception("Timeout during Cartpole Serial Read")


def cartpole_unsafe_read(local_sp):
    encoderData = local_sp.readline()
    encoderData = encoderData.decode()
    encoderData = encoderData.split(",")
    return encoderData[:-1]



def cartpole_close_serial(local_sp):
    local_sp.close()

# why bother busy sleeping: 
# https://stackoverflow.com/questions/1133857/how-accurate-is-pythons-time-sleep
def cartpole_busy_sleep(duration, loop_start = None, get_now=time.time):
    now = get_now()

    if loop_start is None:
        end = now + duration
    else: 
        end = duration - (get_now() - loop_start)

    while now < end:
        now = get_now()