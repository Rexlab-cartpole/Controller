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
# "requested_vel_1",
# "requested_vel_2",
# "serial_tx_time",
# "time_since_last_command",
]

local_sp = None

def cartpole_open_serial(sp, baud = 256000):
    global local_sp
    sp = serial.Serial(sp, baud, timeout=1)
    sp.flush()
    local_sp = sp


# inputs are in nm
def cartpole_write_motors(linear_torque, elbow_torque):
    global local_sp
    sp = local_sp
    # max prescision of 3 decimal places
    linear_vel = float(linear_torque, digits=3) 
    elbow_vel = float(elbow_torque, digits=3)
    string_to_write = "<" + str(linear_vel) + ", " + str(elbow_vel) + ">"

    timeout = 10
    while(timeout > 0):
        try:
            local_sp.write(string_to_write.encode())
            return
        except:
            timeout -= 1
    raise Exception("Timeout during Cartpole Serial Write")

# timeout in attempt counts, default 10, best not to change
def cartpole_safe_read(ser):
    timeout = 10
    while(timeout > 0):
        encoderData = ser.readline()
        try: # catch byte errors in encoderData.decode
            encoderData = encoderData.decode()
            encoderData = encoderData.split(",")
            if len(encoderData) - 1 == len(cartpole_encoder_data_fields): # account for the \r\n 
                return encoderData[:-1]
            timeout -= 1
        except:
            timeout -= 1
    raise Exception("Timeout during Cartpole Serial Read")



def cartpole_close_serial():
    sp = local_sp
    sp.close(sp)

# why bother busy sleeping: 
# https://stackoverflow.com/questions/1133857/how-accurate-is-pythons-time-sleep
def cartpole_busy_sleep(duration, get_now=time.perf_counter):
    now = get_now()
    end = now + duration
    while now < end:
        now = get_now()