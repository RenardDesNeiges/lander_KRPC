import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np
from matplotlib import cm
import krpc
import math
import time

from mpl_toolkits.mplot3d import Axes3D

def sym_sqrt(val):
    if val < 0:
        return - math.sqrt(-val)
    else:
        return math.sqrt(val)

def circle(x,y,r):
    x_vals = []
    y_vals = []

    for i in range(51):
        x_vals.append(x + r * math.cos(i * math.pi/25))
        y_vals.append(y + r * math.sin(i * math.pi/25))

    return (x_vals,y_vals)


class PID_controller:
    def __init__(self,kp,ki,kd,arw = 1,log = False):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.acc = 0
        self.past_val = 0
        self.dwait = True
        self.arw = arw
        self.log = log

        self.vACC = []
        self.pACC = []
        self.dACC = []
        self.iACC = []

        self.past_time = time.time()

    def update(self,val,obj):
        delta_t = time.time() - self.past_time
        self.past_time = time.time()
        if(delta_t != 0):
            diff = (obj(val) - self.past_val) * (delta_t/0.15)
        else:
            diff = 0
        if self.dwait:
            self.acc += obj(val)
            ctrl = self.kp * obj(val) + self.ki * self.acc
            self.dwait = False
        else:
            self.acc += obj(val)
            self.past_val = obj(val)
            ctrl = self.kp * obj(val) + self.ki * self.acc + self.kd * diff
        if ctrl > self.arw or ctrl < -self.arw :
            self.acc -= obj(val)
            ctrl -= self.ki*self.acc

        if self.log:
            self.vACC.append(ctrl)
            self.pACC.append(self.kp*obj(val))
            self.dACC.append(self.kd*diff)
            self.iACC.append(self.ki*self.acc)
        return ctrl

    def plot(self):
        if self.log:

            plt.axhline(y=0, color='k', linestyle='dashed')
            plt.axhline(y=1, color='k', linestyle='dashed')
            plt.axhline(y=-1, color='k', linestyle='dashed')

            k_l = mpatches.Patch(color='k', label='Output Value')
            g_l = mpatches.Patch(color='g', label='Proportional')
            b_l = mpatches.Patch(color='b', label='Integrator')
            r_l = mpatches.Patch(color='r', label='Derivator')
            plt.legend(handles=[k_l,g_l,b_l,r_l])

            plt.plot(
                range(len(self.vACC)),
                self.vACC,
                'k',
                range(len(self.pACC)),
                self.pACC,
                'g',
                range(len(self.iACC)),
                self.iACC,
                'b',
                range(len(self.dACC)),
                self.dACC,
                'r',
            )
            plt.show()


class Attitude_control:
    def __init__(self,vessel,conn):
        attitude_kp = (.2/20)
        attitude_kpd = 10
        attitude_kpi = (.2 / 6)

        heading_kp = .015
        heading_kpd = 6
        heading_kpi = 0

        self.vessel = vessel
        self.conn = conn

        self.trg_pitch = 0
        self.trg_roll = 0

        self.pitch = conn.add_stream(getattr, self.vessel.flight(), 'pitch')
        self.roll = conn.add_stream(getattr, self.vessel.flight(), 'roll')
        self.heading = conn.add_stream(getattr, self.vessel.flight(), 'heading')

        self.pitchOutput = []
        self.pitchError = []

        self.headingOutput = []
        self.headingError = []

        self.rollOutput = []
        self.rollError = []

        self.pitchCTRL = PID_controller(attitude_kp,
                                        attitude_kpi * attitude_kp,
                                        attitude_kpd * attitude_kp, 30, True)

        self.rollCTRL = PID_controller(attitude_kp, attitude_kpi * attitude_kp,
                                       attitude_kpd * attitude_kp, 30, True)

        self.headingCTRL = PID_controller(heading_kp,
                                          heading_kpi * heading_kp,
                                          heading_kpd * heading_kp, 30, True)


    def pitchObj(self,val):
        return (self.trg_pitch - val) + 0.01 * (self.trg_pitch - val) * (
            self.trg_pitch - val) * (self.trg_pitch - val)

    def rollObj(self,val):
        return (self.trg_roll - val) + 0.01 * (self.trg_roll - val) * (
            self.trg_roll - val) * (self.trg_roll - val)

    def headingObj(self, val):
        target = 0
        diff = target-val
        if diff > 180:
            diff = -(180-diff)
        elif diff < -180:
            diff = 360+diff
        return diff

    def update(self,trg_pitch,trg_roll):

        self.trg_pitch = trg_pitch
        self.trg_roll = trg_roll

        pitch_control_val = self.pitchCTRL.update(self.pitch(),self.pitchObj)
        roll_control_val = self.rollCTRL.update(self.roll(), self.rollObj)
        heading_control_val = self.headingCTRL.update(self.heading(), self.headingObj)

        #print("pitch_error=" + str(self.pitchObj(self.vessel.flight().pitch))+ " ; ")
        #print("pitch_control_val="+str(pitch_control_val))

        self.pitchOutput.append(pitch_control_val)
        self.pitchError.append(
            (self.pitchObj(self.pitch()))/90.0)

        self.rollOutput.append(roll_control_val)
        self.rollError.append(
            (self.rollObj(self.roll()))/90.0)

        self.headingOutput.append(heading_control_val)
        self.headingError.append(self.headingObj(self.heading()))

        self.vessel.control.pitch = pitch_control_val
        self.vessel.control.roll = roll_control_val
        self.vessel.control.yaw = heading_control_val

    def plot_pitch(self):

        plt.axhline(y=0, color='k', linestyle='dashed')
        plt.axhline(y=1, color='k', linestyle='dashed')
        plt.axhline(y=-1, color='k', linestyle='dashed')

        r_l = mpatches.Patch(color='r', label='Output Value')
        k_l = mpatches.Patch(color='k', label='Measured Value')
        plt.legend(handles=[k_l,r_l])

        plt.plot(range(len(self.pitchOutput)), self.pitchOutput, 'r',
                 range(len(self.pitchError)), self.pitchError, 'k')
        plt.show()

        # print("Average error is : " + str(sum(self.pitchError)/len(self.pitchError)))

    def plot_roll(self):

        plt.axhline(y=0, color='k', linestyle='dashed')
        plt.axhline(y=1, color='k', linestyle='dashed')
        plt.axhline(y=-1, color='k', linestyle='dashed')

        r_l = mpatches.Patch(color='r', label='Output Value')
        k_l = mpatches.Patch(color='k', label='Measured Value')
        plt.legend(handles=[k_l, r_l])

        plt.plot(range(len(self.rollOutput)), self.rollOutput, 'r',
                 range(len(self.rollError)), self.rollError, 'k')
        plt.show()

        # print("Average error is : " +
        #       str(sum(self.rollError) / len(self.rollError)))

    def plot_heading(self):

        plt.axhline(y=0, color='k', linestyle='dashed')
        plt.axhline(y=1, color='k', linestyle='dashed')
        plt.axhline(y=-1, color='k', linestyle='dashed')

        r_l = mpatches.Patch(color='r', label='Output Value')
        k_l = mpatches.Patch(color='k', label='Measured Value')
        plt.legend(handles=[k_l, r_l])

        plt.plot(range(len(self.headingOutput)), self.headingOutput, 'r',
                 range(len(self.headingError)), self.headingError, 'k')
        plt.show()

        # print("Average error is : " +
        #       str(sum(self.rollError) / len(self.rollError)))


class VSpeed_control:
    def __init__(self,vessel,body,alt,conn):
        self.conn = conn
        self.vessel = vessel
        self.body = body
        self.prev_alt = alt()
        kp = 0.2
        kpi = .01
        kpd = .2

        self.val = []
        self.ctrl = []
        self.thrst = []
        self.trg = []

        self.mass = self.conn.add_stream(getattr,self.vessel,'mass')
        self.max_thrust = self.conn.add_stream(getattr,self.vessel,'max_thrust')
        self.g = self.body.surface_gravity

        self.v_speed_CTRL = PID_controller(kp,kpi*kp,kpd*kp,arw = 10,log=True)

        self.past_time = time.time()

    def v_speed(self,alt):
        delta_t = time.time() - self.past_time
        self.past_time = time.time()
        if delta_t != 0:
            vspd = (alt - self.prev_alt)*(delta_t/0.15)
        else:
            vspd = 0
        return vspd

    def model_control(self):
        return ( self.mass() * self.g) / self.max_thrust()


    def update(self,speed,alt):

        #print("v-speed : " + str(self.v_speed(alt))),
        #print(" v-speed error = " + str(speed-self.v_speed(alt)))
        v_speed_ctrl_val = self.v_speed_CTRL.update(self.v_speed(alt),lambda x : speed - x) + self.model_control()

        if v_speed_ctrl_val > 1:
            self.thrst.append(1)
        elif v_speed_ctrl_val < 0:
            self.thrst.append(0)
        else:
            self.thrst.append(v_speed_ctrl_val)
        self.trg.append(speed)

        self.vessel.control.throttle = v_speed_ctrl_val
        #print(" ; thrst-ctrls = " + str(v_speed_ctrl_val)),
        self.ctrl.append(self.model_control())
        self.val.append(self.v_speed(alt))
        self.prev_alt = alt

    def plot(self):

        plt.axhline(y=0, color='k', linestyle='dashed')
        plt.axhline(y=.5, color='k', linestyle='dashed')
        plt.axhline(y=-.5, color='k', linestyle='dashed')

        k_l = mpatches.Patch(color='k', label='Vertical Speed')
        r_l = mpatches.Patch(color='r', label='Model Control Input')
        g_l = mpatches.Patch(color='g', label='Thrust')
        b_l = mpatches.Patch(color='b', label='Target Speed')
        plt.legend(handles=[k_l, r_l, g_l,b_l])

        plt.plot(range(len(self.ctrl)), self.ctrl, 'r',
                 range(len(self.val)), self.val, 'k',
                 range(len(self.thrst)),self.thrst,'g',
                 range(len(self.trg)), self.trg, 'b' )


        plt.show()

class Alt_control:
    def __init__(self,vessel,body,conn):
        self.vessel = vessel
        self.body = body
        self.conn = conn

        self.alt = conn.add_stream(getattr,self.vessel.flight(),'mean_altitude')

        self.vspeedCTRL = VSpeed_control(self.vessel,self.body,self.alt,self.conn)

        kp = .03
        kpi = .001
        kpd = 0


        self.vals = []
        self.inputs = []
        self.pid = []
        self.errors = []
        self.trgACC = []

        self.altitude_CTRL = PID_controller(kp, kpi * kp, kpd * kp,log=True)

    def update(self):


        trgAlt = 100
        val = self.alt()
        self.vals.append(val)

        error = trgAlt - val

        self.trgACC.append(trgAlt)

        self.errors.append(error/50)

        out = self.altitude_CTRL.update(error, lambda x: x)
        ctrl = out
        if out  < -0.5:
            ctrl = -0.5
        elif out > 0.5:
            ctrl = 0.5

        # print("alt-error : " + str(error)),
        # print(" ; trg_speed : " + str(ctrl))
        self.inputs.append(ctrl)
        self.pid.append(out)
        alt = self.alt()
        self.vspeedCTRL.update(ctrl, alt)

    def plot(self):

        plt.axhline(y=0, color='k', linestyle='dashed')
        plt.axhline(y=.5, color='k', linestyle='dashed')
        plt.axhline(y=-.5, color='k', linestyle='dashed')

        b_l = mpatches.Patch(color='b', label='Altitude Error')
        r_l = mpatches.Patch(color='r', label='Target Speed')
        g_l = mpatches.Patch(color='g', label='Raw PID Output')
        plt.legend(handles=[b_l, r_l, g_l])

        plt.plot(range(len(self.errors)), self.errors, 'b',
                 range(len(self.inputs)), self.inputs, 'r',
                 range(len(self.pid)),self.pid, 'g')
        plt.show()
        #self.vspeedCTRL.plot()

    def alt_plot(self):

        r_l = mpatches.Patch(color='r', label='Altitude')
        k_l = mpatches.Patch(color='k', label='Target Altitude')
        plt.legend(handles=[k_l, r_l])

        plt.plot(
            range(len(self.vals)),
            self.vals,
            'r',
            range(len(self.trgACC)),
            self.trgACC,
            'k',
        )
        plt.show()



class Position_Controller:
    def __init__(self, vessel,conn):
        self.conn = conn
        self.vessel = vessel
        self.zero = self.vessel.position(self.vessel.orbit.body.reference_frame)
        self.pos = self.conn.add_stream(vessel.position,
                                        self.vessel.orbit.body.reference_frame)

        self.att_controller = Attitude_control(vessel, conn)

        self.x = 0
        self.y = 0
        self.z = 0

        self.past_x = 0
        self.past_y = 0
        self.past_z = 0

        self.v_x = 0
        self.v_y = 0

        self.v_xs = []
        self.v_ys = []

        self.xACC = []
        self.yACC = []
        self.zACC = []

        self.tvx = 0
        self.tvy = 0

        self.ctrl_vxACC = []
        self.ctrl_vyACC = []

        self.ctrl_xACC = []
        self.ctrl_yACC = []

        self.past_time = time.time()

        kvp = 8
        kvpi = 0
        kvpd = 0.5

        kp = 20
        kpi = 10
        kpd = 4

        self.vxPID = PID_controller(kvp, kvp * kvpi , kvp * kvpd , log=True)
        self.vyPID = PID_controller(kvp, kvp * kvpi , kvp * kvpd , log=True)

        self.xPID = PID_controller(kp, kp * kpi, kp * kpd, log=True)
        self.yPID = PID_controller(kp, kp * kpi, kp * kpd, log=True)

    def set_zero(self):
        self.zero = self.vessel.position( self.vessel.orbit.body.reference_frame)

    def get_map_speed(self):

        delta_t = time.time() - self.past_time
        self.past_time = time.time()

        if delta_t != 0:
            self.v_x = (self.x - self.past_x)*(delta_t/0.15)
            self.v_y = (self.y - self.past_y)*(delta_t/0.15)
        else:
            self.v_x = 0
            self.v_y = 0

        #print("vx = "+ str(self.v_x) + " ; vy = " + str(self.v_y))

        self.v_xs.append(self.v_x)
        self.v_ys.append(self.v_y)

        self.past_x = self.x
        self.past_y = self.y

    def objVX(self, val):
        return abs(self.tvx - val) * (self.tvx - val )

    def objVY(self, val):
        return abs(self.tvy - val) * (self.tvy - val )

    def objX(self, val):
        return (-val)

    def objY(self, val):
        return (-val)

    def update_relative_pos(self):
        cpos = self.pos()
        for i in range(len(cpos)):
            if i == 0:
                self.x = cpos[i] - self.zero[i]
                self.xACC.append(self.x)
            elif i == 1:
                self.y = cpos[i] - self.zero[i]
                self.yACC.append(self.y)
            elif i == 2:
                self.z = -(cpos[i] - self.zero[i])
                self.zACC.append(self.z)

        self.get_map_speed()


        #print(str(self.x) + " , " + str(self.y) + " , " + str(self.z))
        #self.pos = lpos
        #print(pos)
        #return pos

    def update(self):
        #pass
        self.update_relative_pos()

        controlX = self.xPID.update(self.x, self.objX)
        controlY = self.yPID.update(self.y, self.objY)

        controlVY = self.vxPID.update(self.v_x, self.objVX)
        controlVX = self.vyPID.update(self.v_y, self.objVY)

        if controlVX > 1.0:
            controlVX = 1.0
        elif controlVX < -1.0:
            controlVX = -1.0

        if controlVY > 1.0:
            controlVY = 1.0
        elif controlVY < -1.0:
            controlVY = -1.0

        self.ctrl_vxACC.append(controlVX)
        self.ctrl_vyACC.append(controlVY)

        self.ctrl_xACC.append(controlX)
        self.ctrl_yACC.append(controlY)

        self.vessel.control.forward = controlVX
        self.vessel.control.right = controlVY
        self.att_controller.update(0,0)
        #print(self.vessel.position(self.vessel.orbit.body.reference_frame)[0])

    def trace(self):
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.set_aspect('equal')
        ax.plot(self.xACC, self.yACC, self.zACC, color = 'b')


        #maxVal = abs(max([max(self.xACC,key=abs),max(self.yACC,key=abs),max(self.zACC,key=abs)]))
        maxVal = 100

        ax.set_xlim3d([-maxVal,+maxVal])
        ax.set_ylim3d([-maxVal,+maxVal])
        ax.set_zlim3d([0,2*maxVal])
        plt.show()

    def plotX(self):

        plt.axhline(y=0, color='k', linestyle='dashed')

        k_l = mpatches.Patch(color='k', label='X Value')
        r_l = mpatches.Patch(color='r', label='Control Value')
        plt.legend(handles=[k_l, r_l])

        plt.plot(
            range(len(self.xACC)),
            self.xACC,
            'k',
            range(len(self.ctrl_xACC)),
            self.ctrl_xACC,
            'r',
        )
        plt.show()
        #self.vspeedCTRL.plot()

    def plotY(self):

        plt.axhline(y=0, color='k', linestyle='dashed')

        k_l = mpatches.Patch(color='k', label='Y Value')
        r_l = mpatches.Patch(color='r', label='Control Value')
        plt.legend(handles=[k_l, r_l])

        plt.plot(
            range(len(self.yACC)),
            self.yACC,
            'k',
            range(len(self.ctrl_yACC)),
            self.ctrl_yACC,
            'r',
        )
        plt.show()
        #self.vspeedCTRL.plot()

    def plotVX(self):

        plt.axhline(y=0, color='k', linestyle='dashed')

        k_l = mpatches.Patch(color='k', label='VX Value')
        r_l = mpatches.Patch(color='r', label='Control Value')
        plt.legend(handles=[k_l, r_l])

        plt.plot(
            range(len(self.v_xs)),
            self.v_xs,
            'k',
            range(len(self.ctrl_vxACC)),
            self.ctrl_vxACC,
            'r',
        )
        plt.show()
        #self.vspeedCTRL.plot()

    def plotVY(self):

        plt.axhline(y=0, color='k', linestyle='dashed')

        k_l = mpatches.Patch(color='k', label='VY Value')
        r_l = mpatches.Patch(color='r', label='Control Value')
        plt.legend(handles=[k_l, r_l])

        plt.plot(
            range(len(self.v_ys)),
            self.v_ys,
            'k',
            range(len(self.ctrl_vyACC)),
            self.ctrl_vyACC,
            'r',
        )
        plt.show()
        #self.vspeedCTRL.plot()

    def plot_map(self):

        ax = plt.gca()
        ax.set_aspect(1)

        plt.plot(
            self.xACC,
            self.yACC,
            'r',
            circle(0,0,5)[0],
            circle(0,0,5)[1],
            'k',
        )
        plt.show()
        #self.vspeedCTRL.plot()
