
import threading
import socket
import pygame
import sys
import time

class PublishTransformThread(threading.Thread):
    def __init__(self, rate):
        super(PublishTransformThread, self).__init__()
        # 
        
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.th = 0.0
        self.increment = 0.0
        self.rot_increment = 0.0
        self.condition = threading.Condition()
        self.done = False

        # Set timeout to None if rate is 0 (causes new_message to wait forever
        # for new data to publish)
        if rate != 0.0:
            self.timeout = 1.0 / rate
        else:
            self.timeout = None

        self.start()

    def update(self, b1, b2, b3, a1, a2, a3, increment, rot_increment):
        self.condition.acquire()
        self.b1 = b1
        self.b2 = b2
        self.b3 = b3
        self.a1 = a1
        self.a2 = a2
        self.a3 = a3

        self.increment = increment
        self.rot_increment = rot_increment
        # Notify publish thread that we have a new message.
        self.condition.notify()
        self.condition.release()

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

            # Copy state into twist message.

            self.linear_b1 = self.b1 * self.increment
            self.linear_b2 = self.b2 * self.increment
            self.linear_b3 = self.b3 * self.increment
            self.angular_a1 = self.a1 * self.rot_increment
            self.angular_a2 = self.a2 * self.rot_increment
            self.angular_a3 = self.a3 * self.rot_increment
            

            self.condition.release()

            # Publish.
            self.publisher.publish(msg)

class PublishJointStateThread(threading.Thread):
    def __init__(self, rate, HOST = '127.0.0.1', PORT = 5880):
        super(PublishJointStateThread, self).__init__()
        self.jpos = str
        self.increment = 0.0
        self.condition = threading.Condition()
        self.done = False
    
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

    def update(self, b1, b2, b3, a1, a2, a3, increment, rot_increment, action):
        self.condition.acquire()
        self.b1 = b1
        self.b2 = b2
        self.b3 = b3
        self.a1 = a1
        self.a2 = a2
        self.a3 = a3

        self.increment = increment
        self.rot_increment = rot_increment
        self.action = action
        # Notify publish thread that we have a new message.
        self.condition.notify()
        self.condition.release()

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
            
            db1 = self.b1 * self.increment
            db2 = self.b2 * self.increment
            db3 = self.b3 * self.increment
            da1 = self.a1 * self.rot_increment
            da2 = self.a2 * self.rot_increment
            da3 = self.a3 * self.rot_increment

            msg = f"{self.action} {da3} {db3} {da2} {db2} {da1} {db1}"
            # msg = f"{self.action} {da1} {db1} {da2} {db2} {da3} {db3}"
            self.condition.release()
            try:
                self.client_socket.sendall(msg.encode('utf-8'))
                print(f"Sent: {msg}")

                data = self.client_socket.recv(1024)
                msg = data.decode()
                print("Received: ", msg)
            except Exception as e:
                print(f"Error sending data: {e}")
            


msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
d : +x
a : -x
w : +y
s : -y
e : up (+z)
q : down (-z)
D : +rot_x
A : -rot_x
W : +rot_y
S : -rot_y
E : +rot_z
Q : -rot_z
------
o : jaw close
p : jaw open

CTRL-C to quit
"""

if __name__=="__main__":

    PORT = 8190
    increment = 0.5 # mm
    rot_increment = 1  #Deg
    rate = 10  # Hz
    repeat = 0.0

    print("velocity: ", increment, "mm")
    print("rate: ", rate, "Hz")
    print("speed if you hold down keys: ", increment * rate, " mm/s")

    # pub_tf_thread = PublishTransformThread(repeat)
    
    pub_js_thread = PublishJointStateThread(repeat, PORT=PORT)

    pygame.init()
    screen = pygame.display.set_mode((640, 200))
    pygame.display.set_caption('Click Here and Press Keys to Teleop')
    pygame.mouse.set_visible(1)

    done = False
    clock = pygame.time.Clock()
    try:
        running = True
        while running:
        # clock.tick(rate)
            b1 = 0
            b2 = 0
            b3 = 0
            a1 = 0
            a2 = 0
            a3 = 0
            action = "spos"
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    quit = True
                    break
            keys = pygame.key.get_pressed()
            get_position = False
            is_key_pressed = False
            # Linear
            if keys[pygame.K_q]:
                b1 = 1
                is_key_pressed = True
            if keys[pygame.K_a]:
                b1 = -1
                is_key_pressed = True
            if keys[pygame.K_w]:
                b2 = 1
                is_key_pressed = True
            if keys[pygame.K_s]:
                b2 = -1
                is_key_pressed = True
            if keys[pygame.K_e]:
                b3 = 1
                is_key_pressed = True
            if keys[pygame.K_d]:
                b3 = -1
                is_key_pressed = True
            
            # Rotation
            if keys[pygame.K_u]:
                a1 = 1
                is_key_pressed = True
            if keys[pygame.K_j]:
                a1 = -1
                is_key_pressed = True
            if keys[pygame.K_i]:
                a2 = 1
                is_key_pressed = True
            if keys[pygame.K_k]:
                a2 = -1
                is_key_pressed = True
            if keys[pygame.K_o]:
                a3 = 1
                is_key_pressed = True
            if keys[pygame.K_l]:
                a3 = -1
                is_key_pressed = True
            # Get position
            if keys[pygame.K_p]:
                get_position = True
            
            if keys[pygame.K_h]:
                action = "home"
            if(is_key_pressed):
                # print("x: {}, y: {}, z: {}".format(x, y, z))
                # print("rot_x: {}, rot_y: {}, rot_z: {}".format(rot_x, rot_y, rot_z))
                # print("jaw_inc: {}".format(jaw_inc))
                # pub_tf_thread.update(x, y, z, rot_x, rot_y, rot_z, increment, rot_increment)
                if get_position:
                    action = "gpos"
            pub_js_thread.update(b1, b2, b3, a1, a2, a3, increment, rot_increment, action)
            time.sleep(1/rate)
    except KeyboardInterrupt:
        print("\n[Main] Ctrl+C received. Exiting cleanly...")
    finally:
        print("[Main] Cleaning up...")
        pub_js_thread.stop()
        pygame.quit()
        sys.exit()
