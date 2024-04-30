
import socket
from GalilRobot import GalilRobot
# from Aurora import Aurora
import numpy as np
import time


DELTAQ = np.zeros((6,))
# START_POS = [0, 10, 0, 6, 0, 3]
START_POS = [0, 40, 0, 32, 0, 20]


def GoHome(rob, update_Q):
    print("Homing the Robot")
    if np.any(update_Q != np.zeros((6,))):
        rob.jointPTPLinearMotionSinglePoint(-update_Q, sKurve=True, sKurveValue=0.004)
    print("Homing Finished")
    return update_Q - update_Q

def goStartPos(rob, update_Q):
    if np.any(update_Q != np.zeros((6,))):
        update_Q = GoHome(rob, update_Q)
    print("Moving to Start Position")
    rob.jointPTPLinearMotionSinglePoint(START_POS, sKurve=True, sKurveValue=0.004)
    update_Q += START_POS
    print("'-------------- Current Joint Info --------------")
    print(rob.getJointPositions())
    return update_Q


def InitRobot():
    if ROB_EXIST:
        print('-------------- initialization: robot --------------')
        rob = GalilRobot("GalilConfig/ConfigGalil_MiniTubuActu.xml")
        rob.connect()
        rob.motorsOn()
        rob.setHome()
        time.sleep(1)
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
        goStartPos(rob, update_Q)
        try:
            while True:
                data = conn.recv(1024)
                if not data:
                    break
                # print(data.decode())
                delta_q = DecodeMessage(data)
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
    # Update Version
    elif msg_lst[0] == "spos":
        delta_q = [float(num) for num in msg_lst[1:]]
        print(delta_q)
    
    return MapToGalil(delta_q)


def SendMovementCommand(rob, delta_q, update_Q):
    if np.sum(delta_q) == 0:
        print(delta_q)
        print("No motion")
        return False, update_Q
    rob.setMotorConstraints('MaxSpeed', 150000)
    rob.setMotorConstraints('MaxAcc', 1000000)
    rob.setMotorConstraints('MaxDec', 1000000)
    print('-------------- Motion: {} --------------'.format(delta_q))
    #tracker._BEEP(1)
    
    update_Q += delta_q
    rob.jointPTPLinearMotionSinglePoint(delta_q, sKurve=True, sKurveValue=0.004)
    
    print("'-------------- Current Joint Info --------------")
    print(rob.getJointPositions())
    # time.sleep(.5)
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
    """
    galil_q = [0]*6
    # Inner tube in Galil  
    # galil_q[0] = 0
    # galil_q[1] = 0
    
    # Mid tube in Galil
    galil_q[0], galil_q[1] = delta_q[4], delta_q[5]
    galil_q[2], galil_q[3] = delta_q[2], delta_q[3]
    galil_q[4], galil_q[5] = delta_q[0], delta_q[1]
    
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
    Connect(rob=ROB)