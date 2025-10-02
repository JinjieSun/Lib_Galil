
import threading
import socket
import pygame
import sys
import time
import numpy as np
from CTCRController import CTCRController
from CTCRKinematics import CTCRKinematics
class PublishTransformThread(threading.Thread):
    def __init__(self, rate, ctcrController, HOST = '127.0.0.1' , PORT = 8190):
        super(PublishTransformThread, self).__init__()
        # 
       
        self.linear_velocity = np.zeros(3)
        self.angular_velocity = np.zeros(3)
        self.th = 0.0
        self.vel_increment = 0.0
        self.rot_increment = 0.0
        self.condition = threading.Condition()
        self.done = False
        self.ctcrController = ctcrController
        self.action = "gpos"
        self.dq = np.zeros(6,)
        self.galil_dq = np.zeros(6,)
        # Set timeout to None if rate is 0 (causes new_message to wait forever
        # for new data to publish)
        if rate != 0.0:
            self.timeout = 1.0 / rate
        else:
            self.timeout = None
        
        try:
            self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.client_socket.connect((HOST, PORT))
            print(f"Connected to server at {HOST}:{PORT}")
        except ConnectionRefusedError:
            print(f"Could not connect to server at {HOST}:{PORT}. Ensure the server is running.")
            pygame.quit()
            sys.exit()
        
        self.start()

    def encode_to_galil(self):
        self.galil_dq = np.zeros(6,)
        dalpha = self.dq[:3][::-1]
        dbeta = self.dq[3:][::-1]
        for i in range(3):
            self.galil_dq[i*2] = np.rad2deg(dalpha[i])
            self.galil_dq[i*2+1] = dbeta[i] * 1e3

    def update(self, x, y, z, rot_x, rot_y, rot_z, vel_increment, rot_increment, action):
        self.condition.acquire()
        self.linear_velocity = np.array([x,y,z]) * vel_increment
        self.angular_velocity = np.array([rot_x, rot_y, rot_z]) * rot_increment
        self.increment = increment
        self.rot_increment = rot_increment
        self.action = action
        self.dq = np.zeros(6,)
        if np.all(self.linear_velocity == 0) and np.all(self.angular_velocity == 0):
            self.dq = np.zeros(6,)
            self.galil_dq = np.zeros(6,)
        elif (action == "spos"):
            self.dq = self.ctcrController.get_delta_joints(self.linear_velocity, self.angular_velocity)
            self.encode_to_galil()
        # Notify publish thread that we have a new message.
        self.condition.notify()
        self.condition.release()

    def parse_joint_message(self, message):
        """
        Parse a joint message string and return alpha (int) and beta (float) arrays.

        Expected format: 'a3 b3 a2 b2 a1 b1'
        Returns: (array_a: [a1, a2, a3], array_b: [b1, b2, b3])
        """
        tokens = message.strip().split()

        if len(tokens) != 6:
            print("Invalid message length. Expected 6 values.")

            print(tokens)
            return None, None
        try:
            values = list(map(float, tokens))
            # Extract joint values in order: a3 b3 a2 b2 a1 b1
            # Goal: array_a = [a3, a2, a1] ), array_b = [b3, b2, b1] (float)
            array_a_rad = np.array([values[0], values[2], values[4]])
            array_b = [values[1], values[3], values[5]]
            return array_a_rad, array_b
        
        except ValueError as e:
            print("Failed to parse joint values:", e)
            return None, None
    


    def stop(self):
        self.condition.acquire()
        self.condition.notify()
        self.condition.release()

        self.done = True
        self.join()

    def run(self):
        while not self.done:
            self.condition.acquire()
            # Wait for a new message or timeout.
            self.condition.wait(self.timeout)
            # if np.all(self.galil_dq == 0):
            #     continue
            msg = f"{self.action} " + " ".join(str(x) for x in self.galil_dq)
            print(msg)
            self.condition.release()
            try:
                self.client_socket.sendall(msg.encode('utf-8'))
                print(f"Sent: {msg}")
                data = self.client_socket.recv(1024)
                msg = data.decode()
                alpha_galil, beta_galil = self.parse_joint_message(msg)
                beta = self.ctcrController.convert_from_galil_to_beta(beta_galil)
                self.ctcrController.update_robot_joint(alpha_galil, beta)
                # print("cur alpha: " + str(self.ctcrController.alpha))
                # print("cur beta: " + str(self.ctcrController.beta))
            except Exception as e:
                print(f"Error sending data: {e}")
            
msg = """
Reading from the keyboard and Publishing to Pose!
---------------------------
d : +x
a : -x
w : +y
s : -y
e : up (+z)
q : down (-z)
# hold SHIFT for rotation on orientation:
SHIFT+d : +roll
SHIFT+a : -roll
SHIFT+w : +pitch
SHIFT+s : -pitch
SHIFT+e : +yaw
SHIFT+q : -yaw
------
p : get current position
h : home the robot

CTRL-C to quit
"""


if __name__=="__main__":

    PORT = 8190
    increment = 0.0005 # m
    rot_increment = 0.05  #Rad
    rate = 20  # Hz
    repeat = 0.0

    print("linear velocity: ", increment, "m")
    print("angular velocity: ", rot_increment, "rad")
    print("rate: ", rate, "Hz")
    print("speed if you hold down keys: ", increment * rate, " m/s")


    dll_path = "./ctcr_kinematics/Kinematics_CLib.dll"
    xml_path = "./ctcr_kinematics/Galil42.xml"
    
    
    robot_length = [139*1e-3, 72*1e-3, 42*1e-3]
    alpha_rad = [0.0, 0.0, 0.0]   # degrees
    beta_m = [-0.062, -0.026, -0.018]  # meters
    
    ctcrKinematics = CTCRKinematics(dll_path, xml_path)
    ctcrController = CTCRController(alpha_rad, beta_m, ctcrKinematics, robot_length, robot_angle_rad=np.deg2rad(-35))

    pub_tf_thread = PublishTransformThread(repeat, ctcrController, PORT=PORT)


    pygame.init()
    screen = pygame.display.set_mode((640, 200))
    pygame.display.set_caption('Click Here and Press Keys to Teleop')
    pygame.mouse.set_visible(1)

    done = False
    clock = pygame.time.Clock()
    try:
        running = True
        while running:
            action = "spos"
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    quit = True
                    break
            keys = pygame.key.get_pressed()

            # Linear
            is_shift = keys[pygame.K_LSHIFT] 
            x = y = z = 0
            rot_x = rot_y = rot_z = 0

            if keys[pygame.K_w]:
                action = "spos"
                if is_shift:
                    rot_z = 1
                else:
                    z = 1
            if keys[pygame.K_s]:
                action = "spos"
                if is_shift:
                    rot_z = -1
                else:
                    z = -1
            if keys[pygame.K_a]:
                action = "spos"
                if is_shift:
                    rot_x = 1
                else:
                    x = 1
            if keys[pygame.K_d]:
                action = "spos"
                if is_shift:
                    rot_x = -1
                else:
                    x = -1
            if keys[pygame.K_q]:
                action = "spos"
                if is_shift:
                    rot_y = 1
                else:
                    y = 1
            if keys[pygame.K_e]:
                action = "spos"
                if is_shift:
                    rot_y = -1
                else:
                    y = -1
            # Get position
            
            if keys[pygame.K_p]:
                action = "gpos"           
            if keys[pygame.K_h]:
                action = "home"
            
            pub_tf_thread.update(x, y, z, rot_x, rot_y, rot_z, increment, rot_increment, action)   
            time.sleep(1/rate)
    except Exception as e:
        import traceback
        print("\n[Main] Ctrl+C received. Exiting cleanly...", e)
        traceback.print_exc()
    finally:
        print("[Main] Cleaning up...")
        pub_tf_thread.stop()
        pygame.quit()
        sys.exit()
