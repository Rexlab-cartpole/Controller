import multiprocessing as mp
from multiprocessing.managers import BaseManager
from queue import LifoQueue
from cartpole_lib_python import *
import cv2
import numpy as np
import matplotlib.pyplot as plt
import time
import pdb


def collect_vid_frames(q, conn):
    vid = cv2.VideoCapture(1, cv2.CAP_DSHOW)
    vid.set(cv2.CAP_PROP_FPS, 120)
    vid.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    vid.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    
    print("starting")
    
    while True:
        ret, frame = vid.read()
        if ret:
            q.put(frame)
        else:
            print("no ret in process")
        if conn.poll():
            if conn.recv() == "end":
                vid.release()
                print("breaking")
                break
    return

def busy_sleep(duration, loop_start, get_now=time.perf_counter):
    now = get_now()

    end = duration + loop_start

    while now < end:
        now = get_now()

class MyManager(BaseManager):
    pass
MyManager.register('LifoQueue', LifoQueue)

if __name__ == '__main__':
    
    # init_odrive_no_calib()
    init_odrive()
    init_odrive()
    
    control_frequency = 30
    control_period = 1/control_frequency
    T = 5 # seconds
    N = int(T*control_frequency)

    measurements = np.zeros((N,2)) # initialize data array
    states = np.zeros((N,4)) # initialize data array
    controls = np.zeros((N,1)).astype(float) # initialize control array

    frame_time = []
    unique_frame_indices = []
    kf_time = []
    loop_time = []
    frames = []

    control_clamp = 0.1
    xg = np.array([0.0, 1.5539225381281545, 0, 0]) # x, θ, xv, θv (use print(get_state())) to find these

    k_matrix = np.array([-4.68901, -27.56145, -4.56255, -5.56030]) # 30hz

    # 30 hz
    A = np.array([ [1.0,  -0.00263967,  0.0333333,  -2.92458e-5],
                [0.0,   1.01727,     0.0,         0.0335247],
                [0.0,  -0.158834,    1.0,        -0.00263967],
                [0.0,   1.03916,     0.0,         1.01727] ])

    B = np.array([ [ 0.001166386647061344],
                [-0.0024900041079065627],
                [ 0.07004855111703442],
                [-0.1498278085801933] ])

    H = np.array([[1, 0, 0, 0],
                [0, 1, 0, 0]])
        
    meas_noise_cov = np.diag([.1,.1]) # most recent
    process_noise_cov = np.diag([1,1,50,50]) # position, angle, linear velocity, angular velocity

    nx = process_noise_cov.shape[0]
    def kf_update(x_meas, x_pred_prev, est_cov_prev):
        Kn = est_cov_prev@H.T @ np.linalg.inv(H@est_cov_prev@H.T + meas_noise_cov)
        x = x_pred_prev + Kn @ (x_meas - H@x_pred_prev)
        est_cov = (np.eye(nx) - Kn@H) @ est_cov_prev @ (np.eye(nx) - Kn@H).T + Kn@meas_noise_cov@Kn.T
        return x, est_cov

    def kf_predict(state, control, est_cov):
        x_pred = A@state + B@control
        est_cov_pred = A@est_cov@A.T + process_noise_cov
        return x_pred, est_cov_pred

    meas = get_measurement() - xg[:2]
    x = np.array([meas[0], meas[1], 0, 0])
    x_pred_prev = x

    # Estimation covariance
    est_cov = np.eye(nx)*20
    est_cov_prev = est_cov

    est_covs = np.zeros((N,nx,nx)).astype(float)
    est_covs[0] = est_cov

    manager = MyManager()
    manager.start()
    lifo = manager.LifoQueue()

    # mp.set_start_method('spawn')
    parent_conn, child_conn = mp.Pipe()
    p = mp.Process(target=collect_vid_frames, args=(lifo,child_conn))
    p.start()
    
    # Wait for process to start
    print("starting frame collection process")
    timeout = 6 # seconds
    start = time.perf_counter()
    i = 0
    while lifo.empty() and time.perf_counter() - start < timeout:
        if time.perf_counter() - start > i:
            i = i + 1
            print("...")
        continue
    
    print("start control loop")
    start = time.time()
    prev_frame = None
    for i in range(N):
        time_start = time.perf_counter()
        
        # 1. Get image
        if not lifo.empty():
            frame = lifo.get()
            frames.append(frame)
            unique_frame_indices.append(i)
            frame_time.append(time.perf_counter() - time_start)
            prev_frame = frame
        else:
            frames.append(prev_frame)
            
        # 2. Get state estimate from KF ([x,θ,xv,θv] from image)
            
        # Measure encoders
        measurement = get_measurement() - xg[:2]
        measurements[i,:] = measurement
        if i > 0:
            # KF update step using encoder measurements
            x, est_cov = kf_update(measurement, x_pred_prev, est_cov_prev)
            x[3] /= 2
            est_covs[i] = est_cov
            
        states[i,:] = x # Change this to output of image KF when using PixelMPC
        
        # 3. Do control
        control = -k_matrix @ states[i,:].T * 0.3 # Arbitrary multiplier to damp odrive torque output

        if control > control_clamp:
            control = control_clamp
        elif control < -control_clamp:
            control = -control_clamp
        
        controls[i] = control
        command_linear_torque(-control) # Odrive torque direction is reversed from our 
        
        # KF predict step using encoder measurements
        x_pred_prev, est_cov_prev = kf_predict(x, [control], est_cov)
        
        kf_time.append(time.perf_counter() - time_start)
        busy_sleep(control_period, time_start, get_now=time.perf_counter)
        loop_time.append(time.perf_counter() - time_start)

    command_linear_torque(0)
    
    print("took {} seconds".format(time.time() - start))
    
    parent_conn.send("end")
    p.join()
    
    
    # Video playback
    playback = True
    if playback:
        for i in range(N):
            cv2.imshow('frame', frames[i])
            if cv2.waitKey(int(1000*control_period)) & 0xFF == ord('q'):
                break
    
    cv2.destroyAllWindows()
    
    # pdb.set_trace()
    
    print(f"number of unique frames: {len(unique_frame_indices)}")
    print(f"total number of frames: {len(frames)}")
    print(f"desired frame rate: {control_frequency}")
    print(f"actual frame rate: {len(unique_frame_indices)/T}")
    
    plt.scatter(unique_frame_indices, np.array(frame_time), s=2, c='red')
    plt.scatter(range(len(loop_time)), np.array(kf_time), s=1, c='green')
    plt.scatter(range(len(loop_time)), np.array(loop_time), s=1, c='blue')
    plt.plot(np.array([control_period for _ in range(len(loop_time))]), c='purple')
        
    plt.ylim([0,.1])
    plt.legend(["frame retrieval time", "kf/control time", "total loop time", "control period"])
    plt.ylabel("time (s)")
    plt.xlabel("iteration")
    
    plt.show()