
import socket
from GalilRobot import GalilRobot
# from Aurora import Aurora
import numpy as np
import time

Tube_Length = [129, 82, 42]
DELTAQ = np.zeros((6,))

# Start Position 1
START_POS = [0, 59, 0, 37, 0, 22]
# Start Position 2
# START_POS = [0, 74, 0, 49, 0, 10]



"""


Galil robot use left-hand, means the positive rotation angle is counter clockwise, 
where the CTCR Solver is in right-hand. Therefore, for all angular joint values/updates,
the sign need to be inverted!!!

"""

def end_checking(tube_length, update_Q, deltaQ):
    
    l3, l2, l1 = tube_length
    _, f3, _, f2, _, f1 = update_Q

    _, d3, _, d2, _, d1 = deltaQ


    t3_end = f3 + d3 - l3
    t2_end = f2 + d2 - l2
    t1_end = f1 + d1 - l1

    if (t3_end > t2_end):
        return False
    if (t2_end > t1_end):
        return False
    if (t1_end > 0):
        return False
    return True

def front_checking(tube_length, update_Q, deltaQ):

    _, f3, _, f2, _, f1 = update_Q

    _, d3, _, d2, _, d1 = deltaQ

    tube3_front = f3 + d3
    tube2_front = f2 + d2
    tube1_front = f1 + d1
    
    if (tube1_front < 0):
        return False
    if (tube1_front > tube2_front):
        return False
    if (tube2_front > tube3_front):
        return False
    if (tube1_front > tube3_front):
        return False
    
    return True

def check_for_valid(update_Q, deltaQ):
    valid_front = front_checking(Tube_Length, update_Q, deltaQ)
    valid_back = end_checking(Tube_Length, update_Q, deltaQ)
    if not valid_front or not valid_back:
        print("Error Occurs on new update joints!!!")
        print("Current Delta Q", update_Q)
        print("Update Joint Values: ", deltaQ)
        print("Results: ", deltaQ + update_Q)
        return False
    return True
    
def GoHome(rob, update_Q):
    print("Homing the Robot")
    if np.any(update_Q != np.zeros((6,))):
        rob.jointPTPLinearMotionSinglePoint(-update_Q, sKurve=True, sKurveValue=0.004)
    print("Homing Finished")
    return update_Q - update_Q

def goStartPos(rob, update_Q, delta_q = None):
    # if np.any(update_Q != np.zeros((6,))):
    #     update_Q = GoHome(rob, update_Q)
    if delta_q == None:
        delta_q = START_POS - update_Q
    else:
        front_valid = check_for_valid(update_Q, delta_q)
        end_valid = check_for_valid(update_Q, delta_q)

        diff_l2 = np.linalg.norm(np.array(update_Q)  + np.array(delta_q) - np.array(START_POS))
        Go_To_Start_Pos_in_joint = not (front_valid and end_valid)
        if (diff_l2 > 1e-3):
            print("Invalid go homt joint values, difference: " + str(diff_l2))
            print("Est result: " + str(np.array(update_Q)  + np.array(delta_q)))
            Go_To_Start_Pos_in_joint = True
        if Go_To_Start_Pos_in_joint:
            delta_q = START_POS - update_Q

    result, update_Q = SendMovementCommand(rob, delta_q, update_Q)
    # if np.any(START_POS != np.zeros((6,))):
    #     print("Moving to Start Position")
    #     rob.jointPTPLinearMotionSinglePoint(START_POS, sKurve=True, sKurveValue=0.004)
    # update_Q += START_POS
    print("'-------------- Current Joint Info --------------")
    print(rob.getJointPositions())
    print()
    return result, update_Q

def InitRobot():
    if ROB_EXIST:
        print('-------------- initialization: robot --------------')
        rob = GalilRobot("GalilConfig/ConfigGalil_MiniTubuActu.xml")
        rob.connect()
        rob.motorsOn()
        rob.setHome()
        rob.printInfo(detailed=True) 
        return rob

def Connect(rob):
    
    port = 8190
    bytes_to_recieve = 53411
    update_Q = np.zeros((6,))
    
    # SOCK_STREAM means TCP socket
    # Connect to the server and send the data
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.bind(('0.0.0.0', port))
    sock.listen()
    print("waiting")
    conn, addr = sock.accept()
    with conn:
        print(f"Connected by {addr}")
        result, update_Q = goStartPos(rob, update_Q)
        try:
            while True:
                ts = time.time()
                data = conn.recv(1024)
                t_server = time.time() - ts + 0.00001
                # print("time for reading from socket: " + str(t_server) + "s ----" + str(1/t_server) + " HZ")
                if not data:
                    break
                # print(data.decode())
                delta_q, goStart = DecodeMessage(data)
                if (goStart):
                    result, update_Q = goStartPos(rob, update_Q, delta_q)
                else:
                    result, update_Q = SendMovementCommand(rob, delta_q, update_Q)
                # print(decoded_msg)
                conn.send(bytes((result,)))
            print("Quit!")     
        except:
            print("Error Occur During connection")
        Exit(rob, update_Q)

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
    if msg_lst[0] == "spos":
        print("Move To Position")
    elif msg_lst[0] == "home":
        goStart = True
        print("Move to Start Position")
    
    return MapToGalil(delta_q), goStart

def SendMovementCommand(rob, delta_q, update_Q):
    ts = time.time()
    if np.count_nonzero(delta_q) == 0:
        print(delta_q)
        print("No motion")
        return False, update_Q

    valid_move = check_for_valid(update_Q, delta_q)
    if not valid_move:
        print("Invalid Movement: No motion")
        return False, update_Q 
    
    rob.setMotorConstraints('MaxSpeed',300000)
    rob.setMotorConstraints('MaxAcc', 1500000)
    rob.setMotorConstraints('MaxDec', 1500000)
    print('-------------- Motion --------------')
    print(delta_q)
    #tracker._BEEP(1)    
    rob.jointPTPLinearMotionSinglePoint(delta_q, sKurve=True, sKurveValue=0.5)
    update_Q += delta_q
    # print("-------------- Current Joint Info --------------")
    # print(rob.getJointPositions())
    # print("-------------- update Q --------------" )
    # print(update_Q)
    t_end = time.time() - ts
    print("time for Galil To Move: " + str(t_end) + "s ----" + str(1/t_end) + " HZ")
    return True, update_Q

def Exit(rob, update_Q):
    print('-------------- close robot and measurement system --------------')
    # robot
    _ = GoHome(rob, update_Q)  
    if ROB_EXIST:
        rob.motorsOff()
        rob.disconnect()

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
    
    return galil_q


if __name__ == '__main__':
    
    # Initialization

  


    ROB_EXIST = True
    TRACKER_EXIST = False
    n_measurements = 10
    break_time_measurement = 0.1

    up_bool = True
    in_bool = True
    # ROB = None
    ROB = InitRobot()
    # in ms?
    ROB._galilBoard.GTimeout(30)
    Connect(rob=ROB)