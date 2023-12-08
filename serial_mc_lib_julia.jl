using LibSerialPort

local_sp

function open_serial(sp, baud = 256000)
    sp = LibSerialPort.open(sp, baud)
    set_flow_control(sp)
    sp_flush(sp, SP_BUF_BOTH)
    local_sp = sp

# inputs are in nm
function write_motors(linear_torque, elbow_torque)
    sp = local_sp
    # max prescision of 3 decimal places
    linear_vel = round(linear_torque, digits=3) 
    elbow_vel = round(elbow_torque, digits=3)
    string_to_write = "<" * string(linear_vel) * ", " * string(elbow_vel) * ">"
    write(sp, string_to_write)
end

# timeout in attempt counts, default 10, best not to change
function safe_read(timeout = 10)
    sp = local_sp
    while(timeout > 0)
        encoderData = readline(sp)
        try # catch byte errors in encoderData.decode
            encoderData = split(encoderData, ",")
            encoderData = encoderData[1:length_decode]
            encoderData = [parse(Float64, x) for x in encoderData]
            if length(encoderData) == length_decode # account for the \r\n 
                sp_flush(sp, SP_BUF_BOTH)
                return encoderData
            end
            timeout -= 1
        catch
            timeout -= 1
        end
    end
    println("Timeout during Read")
end

function close_serial()
    sp = local_sp
    close(sp)
end

function busy_sleep(time_s)
    start = time()
    while (time() - start) < time_s
    end
end