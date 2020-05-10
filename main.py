import krpc
import time
import math
import thread

from control import *


def to_percent(val):
    return str(int(val*10000)/100) + "%"

#defining constants
idle_throttle = 0.3

#connecting to the server
print("Connecting ...")
conn = krpc.connect(name='boii',
                    address='192.168.0.108',
                    rpc_port=50000,
                    stream_port=50001)
vessel = conn.space_center.active_vessel

ref_frame = vessel.surface_reference_frame

print("Connection established !\n")

print("Preparing ship ...")

# vessel.control.throttle = idle_throttle



r_x = (0,1,0)
r_z = (1, 0, 0)
r_y = (0, 0, 1)




#vessel.control.activate_next_stage()


altCTRL = Alt_control(vessel,conn.space_center.bodies["Kerbin"],conn)
posCTRL = Position_Controller(vessel,conn)


a_list = []

raw_input("PRESS ENTER TO START")


def input_thread(a_list):
    raw_input()
    a_list.append(True)

thread.start_new_thread(input_thread, (a_list, ))

vessel.control.rcs = True
vessel.control.throttle = 0
vessel.control.activate_next_stage()

p0 = vessel.position(vessel.orbit.body.reference_frame)

ptime = time.time() * 1000
delta_t_s = []

while not a_list:
    #alt_control_val = altCTRL.update(vessel.flight().surface_altitude,altObj)
    #print("surface_alt: " + str(vessel.flight().surface_altitude))
    #vessel.control.throttle = alt_control_val/10
    #conn.drawing.add_direction(vessel.flight().direction,
    #vessel.surface_reference_frame)
    altCTRL.update()
    posCTRL.update()

    #print("time elapsed : " + str(time.time() * 1000 - ptime))
    delta_t_s.append(time.time() * 1000 - ptime)
    ptime = time.time() * 1000

a_list = []


flag = True

print("Average delay : " + str(sum(delta_t_s)/len(delta_t_s)))

print("post-flight dataVis ==>")



while flag:
    line = ""
    line = raw_input("    Please enter command : ")
    if line == "stop" or line == "quit":
        flag = False
    elif line == "pitchPID":
        posCTRL.att_controller.pitchCTRL.plot()
    elif line == "rollPID":
        posCTRL.att_controller.rollCTRL.plot()
    elif line == "pitch":
        posCTRL.att_controller.plot_pitch()
    elif line == "roll":
        posCTRL.att_controller.plot_roll()
    elif line == "heading":
        posCTRL.att_controller.plot_heading()
    elif line == "headingPID":
        posCTRL.att_controller.headingCTRL.plot()
    elif line == "alt":
        altCTRL.alt_plot()
    elif line == "altCTRL":
        altCTRL.plot()
    elif line == "altPID":
        altCTRL.altitude_CTRL.plot()
    elif line == "vspeed":
        altCTRL.vspeedCTRL.plot()
    elif line == "vspeedPID":
        altCTRL.vspeedCTRL.v_speed_CTRL.plot()
    elif line == "trace":
        posCTRL.trace()
    elif line == "x":
        posCTRL.plotX()
    elif line == "xPID":
        posCTRL.xPID.plot()
    elif line == "y":
        posCTRL.plotY()
    elif line == "yPID":
        posCTRL.yPID.plot()
    elif line == "vx":
        posCTRL.plotVX()
    elif line == "vy":
        posCTRL.plotVY()
    elif line == "vxPID":
        posCTRL.vxPID.plot()
    elif line == "vyPID":
        posCTRL.vyPID.plot()
    elif line == "map":
        posCTRL.plot_map()
    elif line == "delta_t":
        plt.plot( range(len(delta_t_s)), delta_t_s, )
        plt.show()
    else:
        print("    Incorrect command, please try again")