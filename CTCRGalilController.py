import socket
from GalilRobot import GalilRobot
# from Aurora import Aurora
import numpy as np
import time
import sys, os

import threading
import queue



# DEFAULT VALUES
TUBE_OFFSETS = np.array([0, 10, 0, 0, 0, 0])
TUBE_LENGTH = [129, 72, 42]
TUBE_INITAL_POSITION = np.array([0, 77, 0, 46, 0, 24])
# TUBE_INITAL_POSITION = np.array([0, 95, 0, 40, 0, 30])
# TUBE_INITAL_POSITION = np.array([0, 0, 0, 0, 0, 0])
# TUBE_INITAL_POSITION = np.array([np.deg2rad(-180), 20, np.deg2rad(180), 20, 0, 10])
###############################################################################
# Helper Function
###############################################################################
def end_checking(tube_length, q_cur, q_delta, tol=1e-2):
    
    tube_actual_length = tube_length 
    l3, l2, l1 = tube_length
    _, f3, _, f2, _, f1 = q_cur

    _, d3, _, d2, _, d1 = q_delta

    t3_end = f3 + d3 - (l3 + TUBE_OFFSETS[1])
    t2_end = f2 + d2 - (l2 + TUBE_OFFSETS[3])
    t1_end = f1 + d1 - (l1 + TUBE_OFFSETS[5])

    if (t3_end-tol > t2_end):
        print("t3 < t2: ", t3_end-tol, t2_end)
        return False
    if (t2_end-tol > t1_end):
        print("t2 < t1: ", t2_end-tol, t1_end)
        return False
    if (t1_end-tol > 0):
        print("t1 error: ", t1_end-tol)
        return False
    return True

def front_checking(tube_length, q_cur, q_delta, tol=1e-2):

    _, f3, _, f2, _, f1 = q_cur

    _, d3, _, d2, _, d1 = q_delta

    tube3_front = f3 + d3
    tube2_front = f2 + d2
    tube1_front = f1 + d1
    
    if (tube1_front+tol < 0):
        return False
    if (tube1_front > tube2_front+tol):
        print("Tube 1 (outer) front exceeds Tube 2 (middle) front")
        return False
    if (tube2_front > tube3_front+tol):
        print("Tube 2 (middle) front exceeds Tube 3 (inner) front")
        return False
    if (tube1_front > tube3_front+tol):
        return False
    return True

def check_for_valid(tube_length, q_cur, q_delta):
    valid_front = front_checking(tube_length, q_cur, q_delta, tol=1)
    valid_back = end_checking(tube_length, q_cur, q_delta, tol=1)
    if not valid_front or not valid_back:
        print("Error Occurs on new update joints!!!")
        print(f"Tube Front Valid {valid_front}  End Valid {valid_back}")
        print("Current Joint Values", q_cur)
        print("Joint Incremental Values: ", q_delta)
        print("Results: ", q_delta + q_cur)
        return False
    return True

def InitRobot():
    if ROB_EXIST:
        print('-------------- initialization: robot --------------')
        rob = GalilRobot("GalilConfig/ConfigGalil_MiniTubuActu.xml")
        rob.connect()
        rob.motorsOn()
        rob.setHome()
        # Modified Speed and Acc/Dec
        rob.setMotorConstraints('MaxSpeed',212485)
        rob.setMotorConstraints('MaxAcc', 900000)
        rob.setMotorConstraints('MaxDec', 900000) 
        rob.printInfo(detailed=True) 
        return rob

def DecodeMessage(data):
    delta_q = [0] * 6
    msg_lst = data.decode().split(" ")
    if (len(msg_lst) != 7):
        print("Incorrect msg received")
        print(msg_lst)
        return MapToGalil(delta_q)
    goStart = False
    # Update Version
    delta_q = [float(num) for num in msg_lst[1:]]
    if msg_lst[0] not in ["spos", "home", "gpos"]:
        print("Invalid movement command recerived: " + msg_lst[0])
        return -1
    return MapToGalil(delta_q), msg_lst[0]

def MapToGalil(delta_q):
    
    """
    The current setup does not have the 'inner most'
    
        |  Tube Num |   Sim   |  Galil 
        |-----------|---------|---------------
        |   Inner   |    3    |   1 (Inner) 
        |-----------|---------|---------------
        |   Middle  |    2    |   2 (Mid)
        |-----------|---------|---------------
        |    Out    |    1    |   3 (Out)    
        |-----------|---------|---------------


    Attension!!!!

    For Galil Robot, the positive rotation indicates the counter clockwise rotation

    Therefore, the rotation sign need to be changed for Galil Robot to move in correct direction!!!!

    """
    galil_q = [0]*6
    # Inner tube in Galil  
    # galil_q[0] = 0
    # galil_q[1] = 0
    
    # Mid tube in Galil
    galil_q[0], galil_q[1] = -delta_q[4], delta_q[5]
    galil_q[2], galil_q[3] = -delta_q[2], delta_q[3]
    galil_q[4], galil_q[5] = -delta_q[0], delta_q[1]
    
    return np.array(galil_q)


###############################################################################
# Thread 1 – socket server: accept clients, read lines, push into queue
###############################################################################
class SocketListenerThread(threading.Thread):
    def __init__(self,
                 cmd_stack: list,
                 joint_stack: list[float],
                 stack_lock: threading.Lock,
                 host: str = "0.0.0.0",
                 port: int = 8190,
                 backlog: int = 1,
                 bufsize: int = 2048,
                 cmd_stack_size: int = 30):
        super().__init__(daemon=True)
        self.cmd_stack = cmd_stack
        self.cmd_stack_size = cmd_stack_size
        self.joint_stack = joint_stack
        self.stack_lock = stack_lock
        self.host = host
        self.port = port
        self.backlog = backlog
        self.bufsize = bufsize
        self.running = True
        self.start()

    def run(self):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as srv:
            srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            srv.bind((self.host, self.port))
            srv.listen(self.backlog)
            print(f"[Server] Listening on {self.host}:{self.port}")

            # Accept exactly one client; loop again if it disconnects
            while self.running:
                try:
                    conn, addr = srv.accept()
                    print(f"[Server] Client connected from {addr}")
                    q_cur = None
                    with conn:
                        while self.running:
                            data = conn.recv(self.bufsize)
                            if not data:
                                print("[Server] Client disconnected")
                                break
                            delta_q, action = DecodeMessage(data)
                            with self.stack_lock:
                                self.cmd_stack.append((action, delta_q))
                                if len(self.cmd_stack) > self.cmd_stack_size:
                                    _ = self.cmd_stack.pop(0)
                            with self.stack_lock:
                                q_cur = self.joint_stack[-1] if self.joint_stack else q_cur
                                if q_cur is not None:
                                    msg = ' '.join([str(num) for num in q_cur])
                                    conn.sendall(msg.encode("utf-8"))


                except Exception as e:
                    print(f"[Server] Socket exception: {e}")
                    time.sleep(1)

    def stop(self):
        self.running = False
        print("[Server] Listener stopping…")


###############################################################################
# Thread 2 – robot worker: pull cmds from queue and drive robot
###############################################################################
class RobotWorkerThread(threading.Thread):
    def __init__(self, cmd_stack: list, rob: GalilRobot, joint_stack: list[float],
                 stack_lock: threading.Lock, loop_hz: float = 1000.0, Tube_Length = [129, 82, 42]):
        super().__init__(daemon=True)
        self.cmd_stack = cmd_stack
        self.joint_stack   = joint_stack
        self.stack_lock    = stack_lock
        self.rob = rob
        self.period = 1.0 / loop_hz
        self.running = True
        self.q_cur = None
        self.tube_length = Tube_Length
        self.start()
        self.prev_q_delta = None
    
    def run(self):
        self.q_cur = np.zeros(6)
        self.go_home(toInitialPos=True)
        while self.running:
            with self.stack_lock:
                (action, q_delta) = self.cmd_stack.pop() if self.cmd_stack else (None, [0.] * 6) 
              
            if action == "spos":
                result = self.send_movement_command(q_delta)
            elif action == "home":
                result = self.go_home(toInitialPos=True)
            elif action == "gpos":
                self.q_cur = self.rob.getJointPositions()
                result = True
               
            with self.stack_lock:
                self.joint_stack.append(self.q_cur + TUBE_OFFSETS)
                if len(self.joint_stack) > 1000:
                    self.joint_stack.pop(0)
            time.sleep(self.period)
    
    def send_movement_command(self, q_delta):
        ts = time.time()
        if (self.rob.isRobotMoving()):
            # print("Robot during movement, abort move cmd: " + str(q_delta))
            return True
        if (np.count_nonzero(self.prev_q_delta) == 0) and (np.count_nonzero(q_delta) == 0):
            return False
        valid_move = check_for_valid(self.tube_length, self.q_cur, q_delta)
        if not valid_move:
            # print("Invalid Movement: No motion")
            return False
    
        # print('-------------- Motion --------------')
        # print("current cmd buffer size: " + str(len(self.cmd_stack)))
        # print(q_delta)
        diff_direction = np.any(self.prev_q_delta * q_delta < 0) if self.prev_q_delta is not None else False
        
        #tracker._BEEP(1)   
        # if (np.linalg.norm(np.array(q_delta)) > 5): 
        #     print("In Single Point Motion")
        #     # rob.jointPTPDirectMotion(q_delta)
        #     self.rob.jointPTPLinearMotionSinglePoint(q_delta, sKurve=True, sKurveValue=0.004)
        # else:
        # self.rob.jointPTPLinearMotionSinglePoint(q_delta, sKurve=True, sKurveValue=0.004)
        print(q_delta)
        self.rob.jointPTPDirectMotion(q_delta, diff_direction)
        self.prev_q_delta = q_delta
        t_end = time.time() - ts
        self.q_cur = self.rob.getJointPositions()
        # self.q_cur += q_delta
        # print("time for Galil To Move: " + str(t_end) + "s ----" + str(1/t_end) + " HZ")
        print("Current Joint Values: " + str(self.q_cur))
        return True
   
    def go_home(self, toInitialPos=False):
        self.q_cur = self.rob.getJointPositions()
        if toInitialPos:
            q_movement = TUBE_INITAL_POSITION-self.q_cur
        else:
            q_movement = -self.q_cur
        # print(self.q_cur)
        print(q_movement)
        if np.any(q_movement != np.zeros((6,))):
            self.rob.jointPTPLinearMotionSinglePoint(q_movement, sKurve=True, sKurveValue=0.004)
        print("Homing Finished")
        self.q_cur = self.rob.getJointPositions()
        self.q_init = self.q_cur
        # if len(self.cmd_stack) > 0:
        #     self.cmd_stack = []
        return True
    
    def error_compensation(self):
        print("[Error Correction] Starting correction process")
        
        # Try moving in the opposite direction
        corrective_delta = -0.5 * np.array(self.prev_q_delta)
        print(f"[Error Correction] Attempting corrective move: {corrective_delta}")
        
        if check_for_valid(self.tube_length, self.q_cur, corrective_delta):
            self.rob.jointPTPDirectMotion(corrective_delta)
            self.q_cur = self.rob.getJointPositions()
            print("[Error Correction] Successfully corrected out-of-bound pose.")
            return True
        else:
            print("[Error Correction] Correction also invalid. Manual intervention may be required.")
            return False


    def stop(self):
        print('-------------- close robot and measurement system --------------')
        # robot
        _ = self.go_home(toInitialPos=False)  
        if ROB_EXIST:
            self.rob.motorsOff()
            self.rob.disconnect()


if __name__ == '__main__':
    
    # Initialization



    ROB_EXIST = True
    TRACKER_EXIST = False
    n_measurements = 10
    break_time_measurement = 0.1

    up_bool = True
    in_bool = True
    


    ROB = InitRobot()
    # in ms?
    ROB._galilBoard.GTimeout(10)
    
    q = queue.Queue(maxsize=100)   
    joint_stack  = []    
    cmd_stack = []                     
    stack_lock   = threading.Lock()

    listener = SocketListenerThread(cmd_stack, joint_stack, stack_lock) 
    worker   = RobotWorkerThread(cmd_stack ,ROB, joint_stack, stack_lock, loop_hz=10)  

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n[Main] Ctrl‑C received, shutting down…")
        listener.stop()
        worker.stop()
    
