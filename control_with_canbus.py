from pyPS4Controller.controller import Controller
from pySerialTransfer import pySerialTransfer
import pyvesc
from pyvesc.protocol.interface import encode

import time

class struct(object):
    d = 'L'
    x = 1
    
testStruct = struct

FRONT_DRIVE_RPM = 3100
BACK_DRIVE_RPM = 2900
FRONT_SPIN_RPM = 3300
BACK_SPIN_RPM = 3300

teensy_port = '/dev/ttyACM0'

link = pySerialTransfer.SerialTransfer('/dev/ttyACM0')
link.open()
time.sleep(2)

serial_port = '/dev/ttyACM1' # main  motor (second motor should be on canbus)

SAFETY_PRESSED = False

class SetRPM(metaclass=pyvesc.VESCMessage):
    """
    Sets the RPM on a CANBUS connected device. 
    Messages have to be sent repeatedly, as CANBUS has a timeout that's configurable
    within the VESC application. In my case, I set it to 5 seconds, which means the
    motor only works for 5 seconds after sending this signal.
    """
    id = 34
    fields = [
        ('motor_id', 'B'), # my slave is set to 1
        ('command', 'B'), # 8, thanks to this page: https://www.vesc-project.com/node/774
        ('rpm', 'i') # because we're assuming RPM setting
    ]


front_motor = pyvesc.VESC(serial_port=serial_port)

class RampController(Controller):

    def __init__(self, **kwargs):
        Controller.__init__(self, **kwargs)

    def on_x_press(self):
        print("x press")
        if SAFETY_PRESSED == False:
            front_motor.set_rpm(0)
            front_motor.write(encode(SetRPM(1, 8, 0)))
            time.sleep(.1)
            return
        front_motor.set_rpm(FRONT_DRIVE_RPM)
        front_motor.write(encode(SetRPM(1, 8, FRONT_DRIVE_RPM)))
        values = list(encode(SetRPM(1, 8, FRONT_DRIVE_RPM)))
        print(*values, "CANBUS MESSAGE")
        time.sleep(.1)
        return

    def on_up_arrow_press(self):
        print("up arrow press")
        if SAFETY_PRESSED == False:
            testStruct.x = 0
            sendSize = 0
            sendSize = link.tx_obj(testStruct.d, start_pos=sendSize)
            sendSize = link.tx_obj(testStruct.x, start_pos=sendSize)
            link.send(sendSize)
            time.sleep(.1)
            return
        testStruct.d = 'U'
        testStruct.x = 1
        sendSize = 0
        sendSize = link.tx_obj(testStruct.d, start_pos=sendSize)
        sendSize = link.tx_obj(testStruct.x, start_pos=sendSize)
        link.send(sendSize)
        return

    def on_up_arrow_release(self):
        testStruct.d = 'U'
        testStruct.x = 0
        sendSize = 0
        sendSize = link.tx_obj(testStruct.d, start_pos=sendSize)
        sendSize = link.tx_obj(testStruct.x, start_pos=sendSize)
        link.send(sendSize)
        time.sleep(.1)
        testStruct.d = 'D'
        sendSize = 0
        sendSize = link.tx_obj(testStruct.d, start_pos=sendSize)
        sendSize = link.tx_obj(testStruct.x, start_pos=sendSize)
        link.send(sendSize)
        time.sleep(.1)        
        return

    def on_down_arrow_press(self):
        print("down arrow press")
        if SAFETY_PRESSED == False:
            testStruct.x = 0
            sendSize = 0
            sendSize = link.tx_obj(testStruct.d, start_pos=sendSize)
            sendSize = link.tx_obj(testStruct.x, start_pos=sendSize)
            link.send(sendSize)
            time.sleep(.1)
            return
        testStruct.d = 'D'
        testStruct.x = 1
        sendSize = 0
        sendSize = link.tx_obj(testStruct.d, start_pos=sendSize)
        sendSize = link.tx_obj(testStruct.x, start_pos=sendSize)
        link.send(sendSize)

    def on_down_arrow_release(self):
        testStruct.d = 'U'
        testStruct.x = 0
        sendSize = 0
        sendSize = link.tx_obj(testStruct.d, start_pos=sendSize)
        sendSize = link.tx_obj(testStruct.x, start_pos=sendSize)
        link.send(sendSize)
        time.sleep(.1)
        testStruct.d = 'D'
        sendSize = 0
        sendSize = link.tx_obj(testStruct.d, start_pos=sendSize)
        sendSize = link.tx_obj(testStruct.x, start_pos=sendSize)
        link.send(sendSize)
        time.sleep(.1)        
        return

    def on_left_arrow_press(self):
        if SAFETY_PRESSED == False:
            testStruct.x = 0
            sendSize = 0
            sendSize = link.tx_obj(testStruct.d, start_pos=sendSize)
            sendSize = link.tx_obj(testStruct.x, start_pos=sendSize)
            link.send(sendSize)
            time.sleep(.1)
            return
        testStruct.d = 'L'
        testStruct.x = 1
        sendSize = 0
        sendSize = link.tx_obj(testStruct.d, start_pos=sendSize)
        sendSize = link.tx_obj(testStruct.x, start_pos=sendSize)
        link.send(sendSize)
        
        return

    def on_left_arrow_release(self):
        testStruct.d = 'R'
        testStruct.x = 0
        sendSize = 0
        sendSize = link.tx_obj(testStruct.d, start_pos=sendSize)
        sendSize = link.tx_obj(testStruct.x, start_pos=sendSize)
        link.send(sendSize)
        time.sleep(.1)
        testStruct.d = 'L'
        sendSize = 0
        sendSize = link.tx_obj(testStruct.d, start_pos=sendSize)
        sendSize = link.tx_obj(testStruct.x, start_pos=sendSize)
        link.send(sendSize)
        time.sleep(.1)        
        return

    def on_left_right_arrow_release(self):
        testStruct.d = 'R'
        testStruct.x = 0
        sendSize = 0
        sendSize = link.tx_obj(testStruct.d, start_pos=sendSize)
        sendSize = link.tx_obj(testStruct.x, start_pos=sendSize)
        link.send(sendSize)
        time.sleep(.1)
        testStruct.d = 'L'
        sendSize = 0
        sendSize = link.tx_obj(testStruct.d, start_pos=sendSize)
        sendSize = link.tx_obj(testStruct.x, start_pos=sendSize)
        link.send(sendSize)
        time.sleep(.1)
        return

    def on_up_down_arrow_release(self):
        print("up_down_arrow_release")
        testStruct.d = 'U'
        testStruct.x = 0
        sendSize = 0
        sendSize = link.tx_obj(testStruct.d, start_pos=sendSize)
        sendSize = link.tx_obj(testStruct.x, start_pos=sendSize)
        link.send(sendSize)
        time.sleep(.1)
        testStruct.d = 'D'
        sendSize = 0
        sendSize = link.tx_obj(testStruct.d, start_pos=sendSize)
        sendSize = link.tx_obj(testStruct.x, start_pos=sendSize)
        link.send(sendSize)
        time.sleep(.1)        
        return

    def on_right_arrow_press(self):
        print("right arrow press")
        if SAFETY_PRESSED == False:
            testStruct.x = 0
            sendSize = 0
            sendSize = link.tx_obj(testStruct.d, start_pos=sendSize)
            sendSize = link.tx_obj(testStruct.x, start_pos=sendSize)
            link.send(sendSize)
            return
        testStruct.d = 'R'
        testStruct.x = 1
        sendSize = 0
        sendSize = link.tx_obj(testStruct.d, start_pos=sendSize)
        sendSize = link.tx_obj(testStruct.x, start_pos=sendSize)
        link.send(sendSize)

    def on_right_arrow_release(self):
        testStruct.d = 'R'
        testStruct.x = 0
        sendSize = 0
        sendSize = link.tx_obj(testStruct.d, start_pos=sendSize)
        sendSize = link.tx_obj(testStruct.x, start_pos=sendSize)
        link.send(sendSize)
        testStruct.d = 'L'
        sendSize = 0
        sendSize = link.tx_obj(testStruct.d, start_pos=sendSize)
        sendSize = link.tx_obj(testStruct.x, start_pos=sendSize)
        link.send(sendSize)
        
        return

    def on_x_release(self):
        print("x release")
        front_motor.set_rpm(0)
        front_motor.write(encode(SetRPM(1, 8, 0)))
        time.sleep(.1)
        return

    def on_square_press(self):
        print("square press")
        if SAFETY_PRESSED == False:
            front_motor.set_rpm(0)
            front_motor.write(encode(SetRPM(1, 8, 0)))
            return
        front_motor.set_rpm(-FRONT_DRIVE_RPM)
        time.sleep(.01)
        front_motor.write(encode(SetRPM(1, 8, -FRONT_DRIVE_RPM)))
        time.sleep(.1)
        '''
        try:
            print(f"motor rpm: {back_motor.get_rpm()}")
        except:
            print("unable to read back motor rpm")
        '''
        return
    
    def on_square_release(self):
        print("square release")
        front_motor.set_rpm(0)
        front_motor.write(encode(SetRPM(1, 8, 0)))
        time.sleep(.1)
        return

    def on_circle_press(self):
        print("circle press")
        if SAFETY_PRESSED == False:
            front_motor.set_rpm(0)
            front_motor.write(encode(SetRPM(1, 8, 0)))
            time.sleep(.1)
            return
        front_motor.set_rpm(FRONT_DRIVE_RPM)
        front_motor.write(encode(SetRPM(1, 8, FRONT_DRIVE_RPM)))
        time.sleep(.1)
        '''
        try:
            print(f"motor rpm: {back_motor.get_rpm()}")
        except:
            print("unable to read back motor rpm")
        '''
        return
    
    def on_circle_release(self):
        front_motor.set_rpm(0)
        front_motor.write(encode(SetRPM(1, 8, 0)))
        time.sleep(.1)
        return

    def on_triangle_press(self):
        print("triangle press")
        if SAFETY_PRESSED == False:
            front_motor.set_rpm(0)
            time.sleep(.01)
            front_motor.write(encode(SetRPM(1, 8, 0)))
            return
        front_motor.set_rpm(-FRONT_DRIVE_RPM)
        time.sleep(.01)
        front_motor.write(encode(SetRPM(1, 8, -FRONT_DRIVE_RPM)))
        time.sleep(.1)
        '''
        try:
            print(f"motor rpm: {back_motor.get_rpm()}")
        except:
            print("unable to read back motor rpm")
        '''
        return
    
    def on_triangle_release(self):
        front_motor.set_rpm(0)
        front_motor.write(encode(SetRPM(1, 8, 0)))
        time.sleep(.1)
        return

    def on_L3_down(self, value):
        if SAFETY_PRESSED == False:
            front_motor.set_rpm(0)
            front_motor.write(encode(SetRPM(1, 8, 0)))
            return

        if value > 4000:
            front_motor.set_duty_cycle(.1)
            '''
            try:
                print(f"motor rpm: {front_motor.get_rpm()}")
            except:
                print("unable to read front motor rpm")
            '''
        if value < -1000:
            front_motor.set_duty_cycle(int(-.1))
        return

    def on_L3_y_at_rest(self):
        front_motor.set_rpm(0)
        front_motor.write(encode(SetRPM(1, 8, 0)))
        time.sleep(.1)
        return

    def on_L3_x_at_rest(self):
        front_motor.set_rpm(0)
        front_motor.write(encode(SetRPM(1, 8, 0)))
        time.sleep(.1)
        return

    def on_R1_press(self):
        global SAFETY_PRESSED
        SAFETY_PRESSED = True
        print(f"Safety pressed: {SAFETY_PRESSED}")

        return

    def on_R1_release(self):
        print("Safety Released")
        global SAFETY_PRESSED
        SAFETY_PRESSED = False
        front_motor.set_rpm(0)
        time.sleep(.1)
        return


controller = RampController(interface="/dev/input/js0", connecting_using_ds4drv=False)
controller.listen()

while True:
    time.sleep(.1)
    
time.sleep(.1)

