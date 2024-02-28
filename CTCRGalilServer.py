
import socket
from GalilRobot import GalilRobot
# from Aurora import Aurora
import numpy as np
import time



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
    host, port = "100.112.77.0", 8190
    bytes_to_recieve = 53411

    # SOCK_STREAM means TCP socket
    # Connect to the server and send the data
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.bind(('0.0.0.0', port))
    sock.listen()
    print("waiting")
    conn, addr = sock.accept()
    with conn:
        print(f"Connected by {addr}")
        while True:
            data = conn.recv(1024)
            if not data:
                break
            # print(data.decode())
            delta_q = DecodeMessage(data)
            SendMovementCommand(rob, delta_q)
            # print(decoded_msg)
    
    print("Quit!")               
    Exit(rob)


def DecodeMessage(data):
    
    delta_q = [0] * 6
    
    msg_lst = data.decode().split(" ")
    if (len(msg_lst) != 7):
        print("Incorrect msg received")
        print(delta_q)

    # Update Version     
    elif msg_lst[0] == "spos":
        delta_q = [int(float(num)) for num in msg_lst[1:]]
        print(delta_q)
    return MapToGalil(delta_q)

def SendMovementCommand(rob, delta_q):
    if np.sum(delta_q) == 0:
        print("No motion")
        return
    rob.setMotorConstraints('MaxSpeed', 150000)
    rob.setMotorConstraints('MaxAcc', 1000000)
    rob.setMotorConstraints('MaxDec', 1000000)
    print('-------------- Motion: {} --------------'.format(delta_q))
    #tracker._BEEP(1)
    rob.jointPTPLinearMotionSinglePoint(delta_q, sKurve=True, sKurveValue=0.004)
    # time.sleep(.5)
    return


def Exit(rob):
    print('-------------- close robot and measurement system --------------')
    # robot
    if ROB_EXIST:
        rob.motorsOff()
        rob.disconnect()


def MapToGalil(delta_q):
    
    """
    The current setup does not have the 'inner most'
    
        |  Tube Num |   Sim   |  Galil 
        |-----------|---------|---------------
        |   Inner   |    3    |   2 (Mid) 
        |-----------|---------|---------------
        |   Middle  |    2    |   3 (Out)
        |-----------|---------|---------------
        |    Out    |    1    |   x (Inner)    
        |-----------|---------|---------------
    """
    galil_q = [0]*6
    # Inner tube in Galil  
    # galil_q[0] = 0
    # galil_q[1] = 0
    
    # Mid tube in Galil
    galil_q[2], galil_q[3] = delta_q[4], delta_q[5]
    galil_q[4], galil_q[5] = delta_q[2], delta_q[3]
    
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