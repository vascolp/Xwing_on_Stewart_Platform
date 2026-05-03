#
# Name: xwplatform.py
#
# Description:
# Software to control a X-Wing on Stewart Platform made from LEGO set 42100.
# Uses two technic hubs, the center hub connects to 4 motors, the joystick hub connects to
# 3 motors.
#
# Instructions available here:
#
# To use it install Pybricks 3.6 (or above) on both hubs and run this program on both them.
# 
# To reconfigure BLE channels check below for constants CHAN1 and CHAN2.
#
# Version: 1.0
#
# Author VascoLP: vascolp.lego@gmail.com
#
# Date: April 2026
#

# 
# Hub button presses:
#  Joystick hub:
#   simple click - rr/pp - xx/yy - zz/ww
#   double click - goto 0
#   triple click - change startup mode
#   long click   - exit park
#   
#  Center hub:
#   simple click - open/close wings
#   double click - change follow mode
#   triple click - change startup mode
#   long click   - exit no park
#

# BLE communication, MGS_GOTO_SLICE_INFO: 
# byte 1: Light Color code (color codes below, LGT_*).
# byte 2: Number of slices the interface has. IMU uses 5 slices, Xbox uses 20.
# bytes 3-8: six bytes with the selected slice value for each R,P,W,X,Y,Z. The values should be bewtween -<byte2> and <byte2>


class xwg: # Globals encapsulated...
    hub=None
    sw=None
    comms=None
    legs=None
    cmd_queue=None
    xbox=None
    motion=None
    xwmotor=None
    xwmotor_init=None
    xwmotor_direction=None
    light=None

# Roll: r
# Pitch: p
# Yaw: w

# Hub connections:
# Main hub:
#   A: motor 3(XL, green)
#   B: motor 1(XL, red)
#   C: X
#   D: motor 2(L, yellow)
# Secondary hub:
#   A: motor 6(L green)
#   B: motor 5(XL, red)
#   C: motor wings (L, blue)
#   D: motor 4(L, yellow)

from micropython import const
from pybricks.iodevices import PUPDevice
from pybricks.hubs import ThisHub
from pybricks.pupdevices import Motor
from pybricks.parameters import Port, Direction, Stop, Color, Button
from pybricks.tools import wait, StopWatch, multitask, run_task
from pybricks.geometry import Matrix, vector
import umath as  math
from pybricks.iodevices import XboxController

CHAN1=const(223)
CHAN2=const(232)

DEFAULT_MAX_ANGLE=5050

sign = lambda x: math.copysign(1, x) 

IDLE_TIMEOUT=const(300000)
#IDLE_TIMEOUT=const(5400000) #1.5h, exibitions time

# Inter hub message definition
# ------------------------------------------------------------------------------------------------------------------------------------
MSG_ALIVE=const(99)
MGS_GOTO_SLICE_INFO=const(1)
MSG_TOGGLE_WINGS=const(2)
MSG_EXIT=const(3)
MSG_GOTO_HOME=const(4)
MSG_LIGHT=const(5)
MSG_DOUBLE_CLICK=const(6)
MSG_TRIPLE_CLICK=const(7)

# msgs_by_name={ # debug only
#     MSG_ALIVE: 'MSG_ALIVE',
#     MGS_GOTO_SLICE_INFO: 'MGS_GOTO_SLICE_INFO',
#     MSG_TOGGLE_WINGS: 'MSG_TOGGLE_WINGS',
#     MSG_EXIT: 'MSG_EXIT',
#     MSG_GOTO_HOME: 'MSG_GOTO_HOME',
#     MSG_LIGHT: 'MSG_LIGHT',
#     MSG_DOUBLE_CLICK: 'MSG_DOUBLE_CLICK',
#     MSG_TRIPLE_CLICK: 'MSG_TRIPLE_CLICK',
# }

# Lights definition
# ------------------------------------------------------------------------------------------------------------------------------------
LGT_OFF=const(0)
LGT_CALIB_HARD1=const(1)
LGT_CALIB_HARD2=const(2)
LGT_IMU_WAIT_STAT1=const(3)
LGT_IMU_WAIT_STAT2=const(4)
LGT_IMU_RP=const(5)
LGT_IMU_XY=const(6)
LGT_IMU_ZW=const(7)
LGT_EXITING=const(8)
LGT_GOINGHOME=const(9)
LGT_UI_IMU_STAY=const(10)
LGT_UI_IMU_FOLLOW=const(11)
LGT_UI_CONNECT_XBOX=const(12)
LGT_WAIT_FOR_OTHER_SIDE=const(13)
LGT_TOGGLE_STAY=const(14)
LGT_TOGGLE_FOLLOW=const(15)

HLGT_ON=const(0)
HLGT_BLINK=const(1)
HLGT_ANIMATE=const(2)

hub_lights=( 
    (HLGT_ON,      (Color.GRAY*0.3,)),                       # 0: off
    (HLGT_ANIMATE, ((Color.RED, Color.ORANGE*0.3), 250)),    # 1
    (HLGT_ANIMATE, ((Color.RED, Color.ORANGE*0.3), 150)),    # 2
    (HLGT_ON,      (Color.RED,)),                            # 3
    (HLGT_ON,      (Color.RED*0.6,)),                        # 4 
    (HLGT_ON,      (Color.GREEN,)),                          # 5
    (HLGT_ON,      (Color.CYAN,)),                           # 6
    (HLGT_ON,      (Color.MAGENTA,)),                        # 7
    (HLGT_ON,      (Color.RED,)),                            # 8
    (HLGT_ON,      (Color.RED*0.5,)),                        # 9
    (HLGT_BLINK,   (Color.YELLOW, (150,150, 50, 300))),      # 10
    (HLGT_BLINK,   (Color.CYAN*0.5,  (150,150, 50, 300))),   # 11
    (HLGT_BLINK,   (Color.WHITE, (200,200, 100, 400))),      # 12
    (HLGT_BLINK,   (Color.YELLOW*0.5, (300, 100))),          # 13
    (HLGT_ANIMATE, ((Color.BLUE, Color.YELLOW), 200)),       # 14
    (HLGT_ANIMATE, ((Color.BLUE, Color.CYAN*0.5), 200)),     # 15
)

#------------------------------------------------------------------------------------------------------------------------------------
def hub_light(lgt, set_global=True):
    if lgt == None:
        lgt=xwg.light
    (xwg.hub.light.on, xwg.hub.light.blink, xwg.hub.light.animate)[hub_lights[lgt][0]](*(hub_lights[lgt][1]))
    if set_global: 
        xwg.light=lgt

# Makes code easier to read.
# Poses are kept in a list of six elements in the order (roll, pitch, yaw, x, y, z).
rr=const(0)
pp=const(1)
ww=const(2)
xx=const(3)
yy=const(4)
zz=const(5)

# UI MODES (values stored in hub storage)
UIM_FOLLOW=b'XW1'
UIM_STAY  =b'XW2'
UIM_XBOX  =b'XW3'
UIM_BOGUS =b'XW0'

LEG_MIN_LENGTH_MM = const(240)
LEG_MAX_LENGTH_MM = const(280)

HOME_ZZ_MM = LEG_MIN_LENGTH_MM+(LEG_MAX_LENGTH_MM-LEG_MIN_LENGTH_MM)/4.0 # Z default position

SPEED_DEG_S  = const(2000) # motor speed for normal moves
ACCEL_DEG_S2 = const(2000) # motor acceleration

CALIB_SPEED  = const(600)
RUN_SPEED    = const(1500)
SAFETY_DELTA = const(100) # when the arms stall sometimes the LAs get stuck, so use a litle delta to prevent this

BASE_RADIUS_MM = const(155)
PLAT_RADIUS_MM = const(80)
BASE_JSA = const(8) # Base joint separaration angle, degrees
PLAT_JSA = const(8) # Plat joint separaration angle, degrees

ALL_ROTATION=const(30)
BASE_ANGLE_OFFSET_DEG = const(ALL_ROTATION + 0)
PLAT_ANGLE_OFFSET_DEG = const(ALL_ROTATION + 60)  # rotated by these degrees to interleave joints (typical)

BASE_ANGLES_DEG = [
    BASE_ANGLE_OFFSET_DEG - BASE_JSA +   0, BASE_ANGLE_OFFSET_DEG + BASE_JSA +   0,
    BASE_ANGLE_OFFSET_DEG - BASE_JSA + 120, BASE_ANGLE_OFFSET_DEG + BASE_JSA + 120,
    BASE_ANGLE_OFFSET_DEG - BASE_JSA + 240, BASE_ANGLE_OFFSET_DEG + BASE_JSA + 240,
]
PLAT_ANGLES_DEG = [
    PLAT_ANGLE_OFFSET_DEG - PLAT_JSA +   0, PLAT_ANGLE_OFFSET_DEG + PLAT_JSA +   0,
    PLAT_ANGLE_OFFSET_DEG - PLAT_JSA + 120, PLAT_ANGLE_OFFSET_DEG + PLAT_JSA + 120,
    PLAT_ANGLE_OFFSET_DEG - PLAT_JSA + 240, PLAT_ANGLE_OFFSET_DEG + PLAT_JSA + 240,
]
PAIRING_OFFSET=-1

XWING_MOTOR_TIMEOUT=const(1500)

#------------------------------------------------------------------------------------------------------------------------------------
# encode 16-bit int for system storage
def enc16(i):
    lb = i & 255
    hb = i >> 8
    return bytes([hb, lb])

#------------------------------------------------------------------------------------------------------------------------------------
# decode 16-bit int from system storage
def dec16(b):
    return b[0]*256+b[1]

#------------------------------------------------------------------------------------------------------------------------------------
def clamp(x, lo, hi):
    return max(lo, min(hi, x))

#------------------------------------------------------------------------------------------------------------------------------------
def polar_to_cartesian(r, theta_deg, z=0.0):
    theta_rad = math.radians(theta_deg)
    x = r * math.cos(theta_rad)
    y = r * math.sin(theta_rad)
    return [x, y, z]

#------------------------------------------------------------------------------------------------------------------------------------
class StewartPlatformLegs:
    #--------------------------------------------------------------------------------
    def __init__(self, leg_ports):
        self.B = [polar_to_cartesian(BASE_RADIUS_MM, BASE_ANGLES_DEG[i]) for i in range(6)]
        P_raw = [polar_to_cartesian(PLAT_RADIUS_MM, PLAT_ANGLES_DEG[i]) for i in range(6)]
        self.P = [P_raw[(i + (PAIRING_OFFSET % 6)) % 6] for i in range(6)]

        self.span_mm=LEG_MAX_LENGTH_MM-LEG_MIN_LENGTH_MM
        self.max_angle=DEFAULT_MAX_ANGLE

        self.motors=[]
        for lp in leg_ports:
            if lp == None: 
                m = None
            else:
                port, direction = lp
                m=Motor(port, positive_direction=direction, reset_angle=False)
                m.control.limits(speed=SPEED_DEG_S, acceleration=ACCEL_DEG_S2)
                m.stop()  # ensure stopped - not really nedded?
            self.motors.append(m)

    #--------------------------------------------------------------------------------
    def _valid_motors(self):
        for m in self.motors: 
            if m is not None: yield m

    #--------------------------------------------------------------------------------
    def _enum_valid_motors(self):
        for i,m in enumerate(self.motors): 
            if m is not None: yield i,m
        
    #--------------------------------------------------------------------------------
    def length_to_angle(self, length_mm):
        # Convert desired leg length to motor angle (deg).
        
        length_mm -= LEG_MIN_LENGTH_MM
        return (length_mm * self.max_angle) / self.span_mm

    #--------------------------------------------------------------------------------
    async def goto_lengths(self, lengths_mm):
        MIN_DELTA_TO_MOVE=200
        max_errors = 3
        error =0

        target_angles = []
        for i,m in self._enum_valid_motors():
            l = lengths_mm[i]
            l = clamp(l, LEG_MIN_LENGTH_MM, LEG_MAX_LENGTH_MM)
            target_angle = self.length_to_angle(l)
            current_angle = m.angle()
            if abs(target_angle-current_angle) < MIN_DELTA_TO_MOVE:
                continue
            m.run_target(SPEED_DEG_S, target_angle, Stop.COAST_SMART, wait=False)

    #--------------------------------------------------------------------------------
    async def goto_home(self):
        # Goes to a home (place where LAs are at minimum)
        for i,m in self._enum_valid_motors():
            # Stop.COAST is very important. If not, default is HOLD, the IMU will not be stationary...
            m.run_target(SPEED_DEG_S, 0, then=Stop.COAST, wait=False)
        done=[m is None for m in self.motors]
        while not all(done):
            for i,m in self._enum_valid_motors():
                done[i] = m.done()
                if m.stalled():
                    done[i]=True
                    m.stop()
            await wait(10)

    #--------------------------------------------------------------------------------
    @staticmethod
    def rotation_matrix_rpy(roll_deg, pitch_deg, yaw_deg):
        # Return rotation matrix R (3x3) from roll(X), pitch(Y), yaw(Z) in degrees.
        # R = Rz(yaw) * Ry(pitch) * Rx(roll), column-vector convention.
        rx = math.radians(roll_deg); ry = math.radians(pitch_deg); rz = math.radians(yaw_deg)
        cx = math.cos(rx); sx = math.sin(rx)
        cy = math.cos(ry); sy = math.sin(ry)
        cz = math.cos(rz); sz = math.sin(rz)
     
        Rx = Matrix([ [  1,   0,  0], [  0,  cx, -sx], [  0,  sx,  cx], ])
        Ry = Matrix([ [ cy,   0, sy], [  0,   1,   0], [-sy,   0,  cy], ])
        Rz = Matrix([ [ cz, -sz,  0], [ sz,  cz,   0], [  0,   0,   1], ])
     
        Rzy = Rz  * Ry
        R   = Rzy * Rx
        return R

    #--------------------------------------------------------------------------------
    def pose_to_lengths(self, roll_deg, pitch_deg, yaw_deg, x_mm, y_mm, z_mm):
        # Inverse kinematics: return 6 leg lengths (mm) for given pose.
        R = self.rotation_matrix_rpy(roll_deg, pitch_deg, yaw_deg)
        t = vector(x_mm, y_mm, z_mm)
        lengths = []
        for i in range(6):
            # platform joint in base frame: t + R * P[i]
            Rp = R * vector(*self.P[i])
            top = Rp + t
            Li = top - vector(*self.B[i])
            lengths.append(math.sqrt(Li[0]**2 + Li[1]**2 + Li[2]**2))
        return lengths

    #--------------------------------------------------------------------------------
    def _simple_wait_all_done(self):
        done=[m is None for m in self.motors]
        while not all(done):
            for i,m in self._enum_valid_motors():
                done[i] = m.done()
                if m.stalled():
                    done[i]=True
                    m.stop()
            wait(10)

    #--------------------------------------------------------------------------------
    def _simple_run_all_target(self, speed, target_angles):
        for i,m in self._enum_valid_motors():
            # Stop.COAST is very important. If not, default is HOLD, the IMU will not be stationary...
            m.run_target(speed, target_angles[i], then=Stop.COAST, wait=False)
        await self._simple_wait_all_done()

    #--------------------------------------------------------------------------------
    def _simple_calib(self, speed):
        for m in self._valid_motors():
            m.run(speed)

        done=[m is None for m in self.motors]
        while not all(done):
            wait(30)
            for i,m in self._enum_valid_motors():
                if not done[i] and m.stalled():
                    m.stop()
                    done[i]=True

    #--------------------------------------------------------------------------------
    def simple_hard_calibrate(self):
        # A simplified version of callibration that does not use comms

        hub_light(LGT_CALIB_HARD1)
        self._simple_calib(-CALIB_SPEED)
        for m in self._valid_motors():
            m.reset_angle(0)
        self._simple_run_all_target(CALIB_SPEED, (SAFETY_DELTA,)*6)
        for m in self._valid_motors():
            m.reset_angle(0)
        hub_light(LGT_CALIB_HARD2)
        self._simple_calib(CALIB_SPEED)
        self._simple_run_all_target(CALIB_SPEED, (self.max_angle,)*6)

# ------------------------------------------------------------------------------------------------------------------------------------
# A queue that avoids duplicates (sort of... it avoids to append if it is the same as last appended...)
class Queue:
    def __init__(self):
        self.fifo=[]

    # --------------------------------------------------------------------------------
    def append(self, element):
        if len(self.fifo) == 0 or self.fifo[-1] != element:
            self.fifo.append(element)

    # --------------------------------------------------------------------------------
    def get(self):
        if len(self.fifo) == 0:
            return None
        return self.fifo.pop(0)

    # --------------------------------------------------------------------------------
    def next(self):
        if len(self.fifo) == 0:
            return None
        return self.fifo[0]

#------------------------------------------------------------------------------------------------------------------------------------
class CommunicationsHandler:
    # BLE communication is tricky. Broadcast should be on at least 100ms.
    # So to be sure, we consider a long timeout.
    # This class is supposed to be a singleton but nothing is done to ensure it.
    COMMS_WAIT_TIME=const(300)
    STOP_SEND_TIMEOUT=const(1500)
    NO_CONNECTION_TIMEOUT=const(90000)

    #--------------------------------------------------------------------------------
    def __init__(self, i_am_main, observe_ch):
        self.send_counter=1100 if i_am_main else 2200
        self.observe_ch=observe_ch
        self.rec_last_counter=None
        self.last_sent_tick=None
        self.rec_queue=Queue()
        self.send_queue=Queue()

    #--------------------------------------------------------------------------------
    def wait_other_side_alive(self):
        # Tells that I am alive and wait for the other side.
        # To be used in the main program, not in tasks
        init_time=xwg.sw.time()
        
        xwg.hub.ble.broadcast((-1, MSG_ALIVE,))
        while True:
            wait(10)
            rec=xwg.hub.ble.observe(self.observe_ch)
            if rec and rec == (-1, MSG_ALIVE):
                break
            if xwg.sw.time() - init_time > self.NO_CONNECTION_TIMEOUT: # Timeout... other side did not start
                return False
        wait(self.COMMS_WAIT_TIME) # Give time to the other side read
        self.last_sent_tick=xwg.sw.time()
        return True

    #--------------------------------------------------------------------------------
    async def loop(self):
        while True:
            await wait(10)            
            rec=xwg.hub.ble.observe(self.observe_ch)
            # Ignore alive messages, used only at startup; Ignore already received messages.
            if rec is not None and rec[0] !=-1 and rec[0] != self.rec_last_counter:
                self.rec_last_counter=rec[0]
                self.rec_queue.append(rec)
                # print('\t\t\t\tRECEIVED:', rec[0], msgs_by_name[rec[1]], rec[2:])

            if self.send_queue.next():
                to_send=self.send_queue.get()[1:] # strips the ignorable flag
                # print('SENDING:', to_send[0], msgs_by_name[to_send[1]], to_send[2:])
                await xwg.hub.ble.broadcast(to_send)
                self.last_sent_tick=xwg.sw.time()
                await wait(self.COMMS_WAIT_TIME) # To be sure it is broadcasted properly, give it some time                
                
            if self.last_sent_tick and xwg.sw.time()-self.last_sent_tick > self.STOP_SEND_TIMEOUT:
                await xwg.hub.ble.broadcast(None)
                #print('Stop SEND!')
                self.last_sent_tick=None


    #--------------------------------------------------------------------------------
    def do_send(self, value, can_be_ignored=False):
        # If can_be_ignored is true the message is marked as can be ignored.
        # Platform messages often can be ignored because they are sent in a 
        # stream of positions which might cause the queue to overflow (memory error).
        # So we define a limit of messages that can be queued. On the other hand there are
        # messaged that can't be ignored, those are marked as can't be ignored.
        # Each message will be a tupple where the first item is the can_be_ignored flag
        # and the second item is a counter
        
        MAX_IGNORABLE_SIZE=4
        if can_be_ignored:
            ignorables=[i for i,v in enumerate(self.send_queue.fifo) if v[0]]
            if len(ignorables) == MAX_IGNORABLE_SIZE:
                #print('----                 ignoring:', self.send_queue.fifo[ignorables[0]])
                del self.send_queue.fifo[ignorables[0]]
            self.send_queue.append((True, self.send_counter,) + value)            
        else:
            self.send_queue.append((False, self.send_counter,) + value)

        self.send_counter+=1
        if self.send_counter > 100000000: self.send_counter=0
        
    #--------------------------------------------------------------------------------
    def check_received(self):
        return self.rec_queue.get()

# ------------------------------------------------------------------------------------------------------------------------------------
class CommandQueue(Queue):
    def __init__(self, i_am_main):
        super().__init__()
        self.i_am_main= i_am_main

    # --------------------------------------------------------------------------------
    async def process_queue(self):
        while True:
            await wait(10)
            slice_info=self.get() 
            if slice_info == None: continue
            if self.i_am_main:
                msg=(MGS_GOTO_SLICE_INFO, xwg.light) + tuple(slice_info)
                xwg.comms.do_send(msg, True) # platform position messages can be ignored 
            await xwg.legs.goto_lengths(xwg.motion.slices_info_to_lengths(slice_info))

    # --------------------------------------------------------------------------------
    def slice_info_msg_values(self, msg_data):
        return msg_data[0], msg_data[1:]

#------------------------------------------------------------------------------------------------------------------------------------
class RPY:
    # An object to maintain Roll (r), Pitch (p) and Yaw (w)  returned by hub.imu.orientation()

    # --------------------------------------------------------------------------------
    def __init__(self, r, p, w):
        self.r=r
        self.p=p
        self.w=w
        
    # --------------------------------------------------------------------------------
    def set_from_orientation(self, R):
        self.r=math.atan2( R[2, 1], R[2, 2])
        self.p=math.asin (-R[2, 0])
        self.w=math.atan2( R[1, 0], R[0, 0])

    # --------------------------------------------------------------------------------
    def __eq__ (self, other): return self.r==other.r and self.p==other.p and self.w==other.w

    def add(self, r,p,w): self.r+=r; self.p+=p; self.w+=w; return self
 
# ------------------------------------------------------------------------------------------------------------------------------------
class InputHandler:
    DOUBLE_CLICK_TIMEOUT=400
    LONG_CLICK_TIMEOUT=1000
    MINIMUM_MOV_TICK=100
    MAX_POSE_DEG=(45, 45, 35, 70, 70, 40) # [r p w x y z] w must be that small or it can fall in an unrecoverable position


    # --------------------------------------------------------------------------------
    def __init__(self):
        self.button_mode=0 
        self.first_press_tick=None
        self.second_press_tick=None

    # --------------------------------------------------------------------------------
    def slices_info_to_lengths(self, slice_info):
        n_slices=slice_info[0]
        slices=slice_info[1:]
        pose=[]
        for i in range(6):
            pose.append(slices[i]/n_slices * self.MAX_POSE_DEG[i])
        return xwg.legs.pose_to_lengths(pose[rr], pose[pp], pose[ww], pose[xx], pose[yy], HOME_ZZ_MM + pose[zz])

    # --------------------------------------------------------------------------------
    def check_green_button(self, this_tick):
        # Performs button related actions for the Button.CENTER button.
        # Returns: 
        #    0 - no action
        #    1 - simple click
        #    2 - double click
        #    3 - triple click
        #    4 - long click
        # Actions are associated with the press event, not the release. Two timeouts considered (variables of this class):
        #    DOUBLE_CLICK_TIMEOUT - maximum time between two press events to be considered double click.
        #    LONG_CLICK_TIMEOUT - minimum time that the button must be pressed to be considered a long click.
        # Simple click is sent only when enough time has passed to exclude double click, that is, only DOUBLE_CLICK_TIMEOUT after the press action.
        # Long click will start to blink hub light on orange after double_click_timeout.
        # If a long click is interrupted, a simple click is returned.
        #
        # button_mode coding:
        #         press  release
        # ------+       +-------+       +-------+       +-------....
        #       |       |       |       |       |       |
        #       +-------+       +-------+       +-------+
        #        1   10  100     2   20  200     3
        #   1 - first press
        #  10 - first press passed double_click_timeout
        # 100 - release after first press (10 not happening)
        #   2 - second press
        #  20 - second press passed double_click_timeout
        # 200 - release after second press (20 not happening)
        #   3 - third press
        #   4 - long press
        #

        pressed=xwg.hub.buttons.pressed()

        # First press
        if self.button_mode==0 and Button.CENTER in pressed: # press
            self.first_press_tick=this_tick
            self.button_mode=1
            return 0

        # Keep pressing
        elif self.button_mode==1 and Button.CENTER in pressed and this_tick-self.first_press_tick > self.DOUBLE_CLICK_TIMEOUT: # still press
            self.button_mode=10 # long start from first press
            xwg.hub.light.blink(Color.ORANGE, (50, 50))
            return 0
        # First release
        elif self.button_mode==1 and Button.CENTER not in pressed: # unpress before longer presses
            self.button_mode=100 # unpress, double click still can happen, save state
            return 0

        # Second press
        elif self.button_mode==100 and Button.CENTER in pressed and this_tick-self.first_press_tick <= self.DOUBLE_CLICK_TIMEOUT: # A second press
            self.second_press_tick=this_tick
            self.button_mode=2
            return 0
        # No second press, reset and call simple click
        elif self.button_mode==100 and Button.CENTER not in pressed and this_tick-self.first_press_tick > self.DOUBLE_CLICK_TIMEOUT:
            self.button_mode=0 # back to start
            return 1 # Do simple click action

        # Second press becomes longer so fall back to simple longer press (ie 10)
        elif self.button_mode==2 and Button.CENTER in pressed and this_tick-self.second_press_tick > self.DOUBLE_CLICK_TIMEOUT: # still press
            self.button_mode=20# long start from second press
            return 0
        # Second release
        elif self.button_mode==2 and Button.CENTER not in pressed: # unpress before longer presses
            self.button_mode=200 # unpress, triple click still can happen, save state
            return 0

        # Third press
        elif self.button_mode==200 and Button.CENTER in pressed and this_tick-self.second_press_tick <= self.DOUBLE_CLICK_TIMEOUT: # A second press
            self.button_mode=3
            return 3  # Do third click action
        # No third press, reset and call double click
        elif self.button_mode==200 and Button.CENTER not in pressed and this_tick-self.second_press_tick > self.DOUBLE_CLICK_TIMEOUT:
            self.button_mode=0 # third press tiemout, reset
            return 2 # Do double click action

        # Long press handling
        elif self.button_mode==10 and Button.CENTER in pressed and this_tick-self.first_press_tick > self.LONG_CLICK_TIMEOUT: # stil press long
            self.button_mode=4
            return 4 # long click
        elif self.button_mode==10 and Button.CENTER not in pressed: # long but not long enough
            self.button_mode=0
            return 1 # Do simple click action, no chance for second click now

        elif self.button_mode in (20, 3, 4) and Button.CENTER not in pressed:
            self.button_mode=0
            return 0
        return 0

#------------------------------------------------------------------------------------------------------------------------------------
class MotionHandler(InputHandler):
    # --------------------------------------------------------------------------------
    def __init__(self, n_slices, max_val):
        super().__init__()
        self.mode=0
        self.last_change_light_tmp_click=None
        self.msg_counter =-1

        self.n_slices=n_slices
        self.slices_limits=[0]
        s=max_val/self.n_slices

        # Linear distribution of slices
        for x in range(self.n_slices):
            self.slices_limits.append(s*(x+1)) 

        self.last_slices=[0]*6
        self.cur_slices =[0]*6

    # --------------------------------------------------------------------------------
    def value_to_slice(self, value):
        s=None
        vns=abs(value)
        for i in range(self.n_slices):
            if self.slices_limits[i] <= vns <= self.slices_limits[i+1]:
                s=i
                break
        if s == None:
            s=self.n_slices
        if value!=vns:
            s=-s
        return s

    # --------------------------------------------------------------------------------
    def send_slices(self):
        xwg.cmd_queue.append((self.n_slices,) + tuple(self.cur_slices))

    # --------------------------------------------------------------------------------
    def set_ui_mode(self, this_tick):
        # Sets the UI mode for the next startup
        ui_mode_storage = xwg.hub.system.storage(4, read=3)
        if ui_mode_storage == UIM_FOLLOW:
            ui_mode_storage=UIM_STAY  
            hub_light(LGT_UI_IMU_STAY, set_global=False)
        elif ui_mode_storage == UIM_STAY:
            ui_mode_storage=UIM_XBOX
            hub_light(LGT_UI_CONNECT_XBOX, set_global=False)
        # elif ui_mode_storage == UIM_XBOX:
        #     ui_mode_storage==UIM_FOLLOW
        else:
            ui_mode_storage=UIM_FOLLOW 
            hub_light(LGT_UI_IMU_FOLLOW, set_global=False)
        xwg.hub.system.storage(4, write=ui_mode_storage)

    # --------------------------------------------------------------------------------
    def handle_messages_from_other_hub(self, this_tick):
        # Returns: 0 - exit; 1 - something done, continue; 2 - continue, nothing done
        rec=xwg.comms.check_received()
    
        if not (rec is None or rec[0] == self.msg_counter): # Ignore already handled messages
            self.msg_counter = rec[0]
            if rec[1] == MSG_EXIT:
                return 0 # Exit without reseting hub position

            elif rec[1] == MSG_DOUBLE_CLICK:
                self.follow_mode = not self.follow_mode
                hub_light(LGT_TOGGLE_FOLLOW if self.follow_mode else LGT_TOGGLE_STAY, set_global=False)
                self.last_change_light_tmp_click = this_tick
                return 1

            elif rec[1] == MSG_TRIPLE_CLICK:
                self.set_ui_mode(this_tick)
                self.last_change_light_tmp_click = this_tick
                return 1

        if self.last_change_light_tmp_click != None and this_tick - self.last_change_light_tmp_click > 2000:
            self.last_change_light_tmp_click = None
            hub_light(None)

        return 2

#------------------------------------------------------------------------------------------------------------------------------------
class IMUHandler(MotionHandler):
    
    # --------------------------------------------------------------------------------
    def __init__(self, follow_mode=False):
        super().__init__(5, 0.27)
        self.follow_mode = follow_mode

    # --------------------------------------------------------------------------------
    async def wait_for_stationary(self):
        # Waits for a stationary position and saves initial RPY
        while not xwg.hub.imu.stationary():
            hub_light(LGT_IMU_WAIT_STAT1)
            await wait(30)
            hub_light(LGT_IMU_WAIT_STAT2)
            await wait(30)
        r=RPY(0,0,0)
        r.set_from_orientation(xwg.hub.imu.orientation())
        return r

    # --------------------------------------------------------------------------------
    async def loop(self):
        current_imu=RPY(0,0,0)

        init_imu=await self.wait_for_stationary()
        
        hub_light(LGT_IMU_RP)
        xwg.comms.do_send((MSG_LIGHT, LGT_IMU_RP))
        
        self.send_slices()
        last_action_tick=xwg.sw.time()

        move_tick=0
        while True:
            await wait(30)
            self.this_tick= xwg.sw.time()

            if self.this_tick - last_action_tick > IDLE_TIMEOUT:
                break

            #---------------------------------------------------------------------- Hub buttons
            btn = self.check_green_button(self.this_tick)
            if btn == 1: # Mode switching rr/pp - xx/yy - zz/ww
                self.mode += 1
                if self.mode > 2: self.mode=0
                lgt= (LGT_IMU_RP, LGT_IMU_XY, LGT_IMU_ZW)[self.mode]
                xwg.comms.do_send((MSG_LIGHT, lgt))
                hub_light(lgt)
                last_action_tick=self.this_tick
            elif btn == 2: # Goto 0 position
                self.last_slices=[0]*6
                self.cur_slices =[0]*6
                self.send_slices()
                last_action_tick=self.this_tick
                continue
            elif btn == 3: # Change next startup mode
                self.set_ui_mode(self.this_tick)
                self.last_change_light_tmp_click = self.this_tick
                continue
            elif btn == 4: # Exit
                return True # Exit reseting hub position

            #---------------------------------------------------------------------- Messages
            r=self.handle_messages_from_other_hub(self.this_tick)
            if r==0: # Exit no goto home
                return False
            elif r==1:
                last_action_tick=self.this_tick

            # The moving action is only done in bigger intervals because the platform takes its time to move. No need to flood it with changes.
            if self.this_tick - move_tick < self.MINIMUM_MOV_TICK:
                continue

            #---------------------------------------------------------------------- IMU
            current_imu.set_from_orientation(xwg.hub.imu.orientation())
            current_imu.add(-init_imu.r, -init_imu.p, -init_imu.w)
            rslice=self.value_to_slice(current_imu.r)
            pslice=self.value_to_slice(current_imu.p)

            # Convert joystick IMU motion to proper platform directions
            if   self.mode == 0: # rr/pp
                rslice,pslice=pslice,rslice
                bimode_rr=rr
                bimode_pp=pp
            elif self.mode == 1: # xx/yy
                pslice=-pslice
                bimode_rr=xx
                bimode_pp=yy
            elif self.mode == 2: # ww/zz
                rslice=-rslice
                bimode_rr=ww
                bimode_pp=zz
 
            if self.follow_mode: # Follow mode
                if   rslice>self.cur_slices[bimode_rr]: self.cur_slices[bimode_rr] +=1
                elif rslice<self.cur_slices[bimode_rr]: self.cur_slices[bimode_rr] -=1
                if   pslice>self.cur_slices[bimode_pp]: self.cur_slices[bimode_pp] +=1
                elif pslice<self.cur_slices[bimode_pp]: self.cur_slices[bimode_pp] -=1
            else: # Stay mode
                if   rslice> 0 and rslice>self.cur_slices[bimode_rr]: self.cur_slices[bimode_rr] +=1
                elif rslice< 0 and rslice<self.cur_slices[bimode_rr]: self.cur_slices[bimode_rr] -=1
                if   pslice> 0 and pslice>self.cur_slices[bimode_pp]: self.cur_slices[bimode_pp] +=1
                elif pslice< 0 and pslice<self.cur_slices[bimode_pp]: self.cur_slices[bimode_pp] -=1

            if self.cur_slices == self.last_slices:
                continue

            move_tick = self.this_tick
            self.send_slices()

            self.last_slices[bimode_rr]=self.cur_slices[bimode_rr]
            self.last_slices[bimode_pp]=self.cur_slices[bimode_pp]
            last_action_tick=self.this_tick


#------------------------------------------------------------------------------------------------------------------------------------
class XboxHandler(MotionHandler):

    # --------------------------------------------------------------------------------
    def __init__(self):
        super().__init__(20, 99)
        self.follow_mode = True
        self.xbox=XboxController()

    # --------------------------------------------------------------------------------
    async def loop(self):
        hub_light(LGT_IMU_RP)
        xwg.comms.do_send((MSG_LIGHT, LGT_IMU_RP))

        self.send_slices()
        last_action_tick=xwg.sw.time()

        move_tick=0
        last_pressed=()
        delta=const(1)
        last_action_tick=xwg.sw.time()
        dpad_last_tick=xwg.sw.time()
        dpad_double_click_timeout=100
        guide_last_tick=None
        uimode_last_tick=None
        while True:
            await wait(30)
            self.this_tick= xwg.sw.time()

            if self.this_tick - last_action_tick > IDLE_TIMEOUT:
                break

            #---------------------------------------------------------------------- Hub buttons
            if self.check_green_button(self.this_tick) == 4: # Exit
                return True # Exit reseting hub position
            
            #---------------------------------------------------------------------- Xbox buttons
            pressed=self.xbox.buttons.pressed()

            # Long press in GUIDE to exit
            if Button.GUIDE in pressed and guide_last_tick==None: guide_last_tick=self.this_tick
            if Button.GUIDE not in pressed:                       guide_last_tick=None
            if Button.GUIDE in pressed and guide_last_tick!=None and self.this_tick-guide_last_tick > self.LONG_CLICK_TIMEOUT:
                return True

            # A: toggle follow mode
            if Button.A in pressed and Button.A not in last_pressed:
                self.follow_mode = not self.follow_mode
                hub_light(LGT_TOGGLE_FOLLOW if self.follow_mode else LGT_TOGGLE_STAY, set_global=False)
                self.last_change_light_tmp_click = self.this_tick

            # Y: Goto 0 position
            if Button.Y in pressed and Button.Y not in last_pressed:
                self.last_slices=[0]*6
                self.cur_slices =[0]*6
                self.send_slices()
                last_action_tick=self.this_tick
                last_pressed=pressed
                continue

            # B: Long press in button B to change ui mode (for next run)
            if Button.B in pressed and uimode_last_tick==None: uimode_last_tick=self.this_tick
            if Button.B not in pressed:                        uimode_last_tick=None
            if Button.B in pressed and uimode_last_tick!=None and self.this_tick-uimode_last_tick > self.LONG_CLICK_TIMEOUT:
                self.set_ui_mode(self.this_tick)
                self.last_change_light_tmp_click = self.this_tick

            # LB: close wings; RB: open wings
            if Button.LB in pressed and Button.LB not in last_pressed:
                xwg.comms.do_send((MSG_TOGGLE_WINGS, 'C'))
            if Button.RB in pressed and Button.RB not in last_pressed:
                xwg.comms.do_send((MSG_TOGGLE_WINGS, 'O'))

            # X: Toggle X-Wings!
            if Button.X in pressed and Button.X not in last_pressed:
                xwg.comms.do_send((MSG_TOGGLE_WINGS, 'T'))

            # DPad buttons ZZ (with autorepeat)
            if Button.UP in pressed and self.cur_slices[zz] < self.n_slices and \
               (Button.UP not in last_pressed or self.this_tick-dpad_last_tick > dpad_double_click_timeout):
                self.cur_slices[zz] += delta
                dpad_last_tick=self.this_tick
            if Button.DOWN in pressed  and self.cur_slices[zz] > -self.n_slices and \
               (Button.DOWN not in last_pressed or self.this_tick-dpad_last_tick > dpad_double_click_timeout):
                self.cur_slices[zz] -= delta
                dpad_last_tick=self.this_tick

            # DPad buttons WW (with autorepeat)
            if Button.LEFT in pressed and self.cur_slices[ww] < self.n_slices and \
               (Button.LEFT not in last_pressed or self.this_tick-dpad_last_tick > dpad_double_click_timeout):
                self.cur_slices[ww] += delta
                dpad_last_tick=self.this_tick
            if Button.RIGHT in pressed and self.cur_slices[ww] > -self.n_slices and \
               (Button.RIGHT not in last_pressed or self.this_tick-dpad_last_tick > dpad_double_click_timeout):
                self.cur_slices[ww] -= delta
                dpad_last_tick=self.this_tick

            last_pressed=pressed

            #---------------------------------------------------------------------- Messages
            r=self.handle_messages_from_other_hub(self.this_tick)
            if r==0: # Exit no goto home
                return False
            elif r==1:
                last_action_tick=self.this_tick

            # The moving action is only done in bigger intervals because the platform takes its time to move. No need to flood it with changes.
            if self.this_tick - move_tick < self.MINIMUM_MOV_TICK:
                continue

            #---------------------------------------------------------------------- XBox joysticks
            
            rslice=self.value_to_slice( self.xbox.joystick_left() [1])
            pslice=self.value_to_slice( self.xbox.joystick_left() [0])
            xslice=self.value_to_slice( self.xbox.joystick_right()[0])
            yslice=self.value_to_slice(-self.xbox.joystick_right()[1])

            if self.follow_mode: # Follow mode
                if   rslice>self.cur_slices[rr]: self.cur_slices[rr] += delta
                elif rslice<self.cur_slices[rr]: self.cur_slices[rr] -= delta
                if   pslice>self.cur_slices[pp]: self.cur_slices[pp] += delta
                elif pslice<self.cur_slices[pp]: self.cur_slices[pp] -= delta
                if   xslice>self.cur_slices[xx]: self.cur_slices[xx] += delta
                elif xslice<self.cur_slices[xx]: self.cur_slices[xx] -= delta
                if   yslice>self.cur_slices[yy]: self.cur_slices[yy] += delta
                elif yslice<self.cur_slices[yy]: self.cur_slices[yy] -= delta
            else: # Stay mode
                if   rslice> 0 and rslice>self.cur_slices[rr]: self.cur_slices[rr] += delta
                elif rslice< 0 and rslice<self.cur_slices[rr]: self.cur_slices[rr] -= delta
                if   pslice> 0 and pslice>self.cur_slices[pp]: self.cur_slices[pp] += delta
                elif pslice< 0 and pslice<self.cur_slices[pp]: self.cur_slices[pp] -= delta
                if   xslice> 0 and xslice>self.cur_slices[xx]: self.cur_slices[xx] += delta
                elif xslice< 0 and xslice<self.cur_slices[xx]: self.cur_slices[xx] -= delta
                if   yslice> 0 and yslice>self.cur_slices[yy]: self.cur_slices[yy] += delta
                elif yslice< 0 and yslice<self.cur_slices[yy]: self.cur_slices[yy] -= delta
                
            if self.cur_slices == self.last_slices:
                continue

            move_tick = self.this_tick
            self.send_slices()

            for i in range(6):
                self.last_slices[i]=self.cur_slices[i]
            last_action_tick=self.this_tick

#------------------------------------------------------------------------------------------------------------------------------------
async def main_loop():
    if await xwg.motion.loop():
        hub_light(LGT_GOINGHOME)
        xwg.comms.do_send((MSG_GOTO_HOME, 1))
        await xwg.legs.goto_home()

    hub_light(LGT_EXITING)
    xwg.comms.do_send((MSG_EXIT,))
    await wait(700)

#------------------------------------------------------------------------------------------------------------------------------------
async def main_task():
    await multitask(xwg.comms.loop(), xwg.cmd_queue.process_queue(), main_loop(), race=True)
    await wait(300)
    
#------------------------------------------------------------------------------------------------------------------------------------
def main_main():
    # Check mode
    ui_mode_storage = xwg.hub.system.storage(4, read=3)
    if ui_mode_storage == UIM_FOLLOW:
        xwg.motion = IMUHandler(follow_mode=True)
    elif ui_mode_storage == UIM_STAY:
        xwg.motion = IMUHandler(follow_mode=False)
    elif ui_mode_storage == UIM_XBOX:
        # Force the storage to an unknown value so that if not able to connect to an XBox controller
        # falls back into the default IMU. This because XboxController() has no timeout argument.
        xwg.hub.system.storage(4, write=UIM_BOGUS) 
        hub_light(LGT_UI_CONNECT_XBOX)
        xwg.motion = XboxHandler()
        xwg.hub.system.storage(4, write=UIM_XBOX) 
    else:
        xwg.hub.system.storage(4, write=UIM_FOLLOW)
        xwg.motion = IMUHandler(follow_mode=True)

    hub_light(LGT_WAIT_FOR_OTHER_SIDE)
    if not xwg.comms.wait_other_side_alive():
        hub_light(LGT_EXITING)
        wait(1000)
        return # Secondary hub not connected

    xwg.legs.simple_hard_calibrate()

    run_task(main_task())

#------------------------------------------------------------------------------------------------------------------------------------
def toggle_wings(this_tick, option):
    if not ((option == 'C' and xwg.xwmotor_direction == True)  or 
            (option == 'O' and xwg.xwmotor_direction == False) or 
             option in ('T', 'F')):
        return

    if xwg.xwmotor_init != None:
        xwg.xwmotor.stop()
    xwg.xwmotor_init=this_tick
    xwg.xwmotor.run((1 if xwg.xwmotor_direction or option=='F' else -1)*1000)
    # False: opens, True: closes: which means: False closed, True: opened
    xwg.xwmotor_direction= not xwg.xwmotor_direction

#------------------------------------------------------------------------------------------------------------------------------------
async def secondary_loop():
    msg_counter =-1

    last_action_tick=xwg.sw.time()
    while True:
        await wait(10)
        this_tick= xwg.sw.time()
        
        if this_tick - last_action_tick > IDLE_TIMEOUT:
            break

        btn = xwg.motion.check_green_button(this_tick)
        if btn == 1:
            toggle_wings(this_tick, 'T')
            last_action_tick=this_tick
        elif btn == 2:
            xwg.comms.do_send((MSG_DOUBLE_CLICK,))
            last_action_tick=this_tick
        elif btn == 3:
            xwg.comms.do_send((MSG_TRIPLE_CLICK,))
            last_action_tick=this_tick
        elif btn == 4:
            hub_light(LGT_EXITING)
            xwg.comms.do_send((MSG_EXIT,))
            await wait(700)
            break

        if xwg.xwmotor_init != None and this_tick - xwg.xwmotor_init > XWING_MOTOR_TIMEOUT:
            xwg.xwmotor.stop()
            xwg.xwmotor_init=None

        if btn != 0: # Reset last light... buttons might change it.
            hub_light(None)
                
        rec=xwg.comms.check_received()
        if rec is None or rec[0] == msg_counter: # Ignore already handled messages
            continue

        msg_counter = rec[0]
        msg=rec[1]
        msg_args=rec[2:]
 
        if msg == MGS_GOTO_SLICE_INFO:
            light, slice_info = xwg.cmd_queue.slice_info_msg_values(msg_args)
            if light!= xwg.light:
                hub_light(light)
            xwg.cmd_queue.append(slice_info)
            last_action_tick=this_tick

        elif msg == MSG_TOGGLE_WINGS:
            toggle_wings(this_tick, msg_args[0])
            last_action_tick=this_tick

        elif msg == MSG_GOTO_HOME:
            if len(msg_args):
                hub_light(LGT_GOINGHOME)
            await xwg.legs.goto_home()
            last_action_tick=this_tick

        elif msg == MSG_LIGHT:
            hub_light(msg_args[0])
            last_action_tick=this_tick

        elif msg == MSG_EXIT:
            toggle_wings(this_tick, 'F')
            hub_light(LGT_EXITING)
            await wait(XWING_MOTOR_TIMEOUT)
            break

#------------------------------------------------------------------------------------------------------------------------------------
async def secondary_task():
    await multitask(xwg.comms.loop(), xwg.cmd_queue.process_queue(), secondary_loop(), race=True)

#------------------------------------------------------------------------------------------------------------------------------------
def secondary_main():
    xwg.xwmotor=Motor(WING_PORT, positive_direction=Direction.CLOCKWISE)
    xwg.xwmotor.control.limits(speed=2000, acceleration=2000)
    xwg.xwmotor_direction=False
    xwg.xwmotor_init=None

    hub_light(LGT_WAIT_FOR_OTHER_SIDE)
    if xwg.comms.wait_other_side_alive():
        xwg.legs.simple_hard_calibrate()
        xwg.motion = InputHandler()
        hub_light(LGT_IMU_RP)
        run_task(secondary_task())
    else:
        hub_light(LGT_EXITING)
        wait(1000)

#------------------------------------------------------------------------------------------------------------------------------------
# MAIN
#------------------------------------------------------------------------------------------------------------------------------------
try:
    # Secondary hub
    _ = PUPDevice(Port.C)
    OBSERVE_CH=CHAN2
    BROADCAST_CH=CHAN1
    LEG_PORTS = (None, None, None, (Port.D, Direction.COUNTERCLOCKWISE), (Port.B, Direction.COUNTERCLOCKWISE), (Port.A, Direction.COUNTERCLOCKWISE))
    WING_PORT=Port.C
    I_AM_MAIN=False
except OSError as ex:
    # Main hub
    OBSERVE_CH=CHAN1
    BROADCAST_CH=CHAN2
    LEG_PORTS = ((Port.B, Direction.COUNTERCLOCKWISE), (Port.D, Direction.COUNTERCLOCKWISE), (Port.A, Direction.COUNTERCLOCKWISE), None, None, None)
    WING_PORT=None
    I_AM_MAIN=True

# Set globals
xwg.hub = ThisHub(observe_channels=[OBSERVE_CH], broadcast_channel=BROADCAST_CH)
xwg.hub.system.set_stop_button(None)
xwg.sw=StopWatch()
xwg.comms = CommunicationsHandler(i_am_main=I_AM_MAIN, observe_ch=OBSERVE_CH)
xwg.legs=StewartPlatformLegs(LEG_PORTS)
xwg.cmd_queue= CommandQueue(i_am_main=I_AM_MAIN)
xwg.light=LGT_IMU_RP

if I_AM_MAIN:
    main_main()
else:
    secondary_main()
