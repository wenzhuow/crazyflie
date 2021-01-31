    # -*- coding: utf-8 -*-
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2016 Bitcraze AB
#
#  Crazyflie Nano Quadcopter Client
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software
#  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
#  MA  02110-1301, USA.
"""
Simple example that connects to one crazyflie (check the address at the top
and update it to your crazyflie address) and send a sequence of setpoints,
one every 5 seconds.

This example is intended to work with the Loco Positioning System in TWR TOA
mode. It aims at documenting how to set the Crazyflie in position control mode
and how to send setpoints.
"""
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.crazyflie.extpos import Extpos
import natfile.NatNetClient
from cflib.crazyflie.swarm import Swarm
import threading
# URI to the Crazyflie to connect to
uri = 'radio://0/80/2M'

# Change the sequence according to your setup
#             x    y    z  YAW
sequence = [
    (1, 1, 1, 0),
##    (1.5, 2.5, 1.2, 0),
##    (2.5, 2.0, 1.2, 0),
##    (3.5, 2.5, 1.2, 0),
##    (2.5, 3.0, 1.2, 0),
##    (2.5, 2.5, 1.2, 0),
##    (2.5, 2.5, 0.4, 0),
]
l=[]

def wait_for_position_estimator(scf):
    print('Waiting for estimator to find position...')

    log_config = LogConfig(name='Kalman Variance', period_in_ms=500)
    log_config.add_variable('kalman.varPX', 'float')
    log_config.add_variable('kalman.varPY', 'float')
    log_config.add_variable('kalman.varPZ', 'float')

    var_y_history = [1000] * 10
    var_x_history = [1000] * 10
    var_z_history = [1000] * 10

    threshold = 0.001

    with SyncLogger(scf, log_config) as logger:
        for log_entry in logger:
            data = log_entry[1]


            print(data['kalman.varPX'],data['kalman.varPY'],data['kalman.varPZ'])
            var_x_history.append(data['kalman.varPX'])
            var_x_history.pop(0)
            var_y_history.append(data['kalman.varPY'])
            var_y_history.pop(0)
            var_z_history.append(data['kalman.varPZ'])
            var_z_history.pop(0)

            min_x = min(var_x_history)
            max_x = max(var_x_history)
            min_y = min(var_y_history)
            max_y = max(var_y_history)
            min_z = min(var_z_history)
            max_z = max(var_z_history)

##            print(var_x_history)
##            print(var_y_history)
##            print(var_z_history)
##            print("{} {} {}".
##                format(max_x - min_x, max_y - min_y, max_z - min_z))

            if (max_x - min_x) < threshold and (
                    max_y - min_y) < threshold and (
                    max_z - min_z) < threshold:
                break
    print('Finished for estimator to find position')

def reset_estimator(scf):
    cf = scf.cf
    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')

    wait_for_position_estimator(cf)


def set_current_position(scf, x, y, z, yaw_deg):
    scf.cf.param.set_value('kalman.stateX', str(x))
    scf.cf.param.set_value('kalman.stateY', str(y))
    scf.cf.param.set_value('kalman.stateZ', str(z))

    
def position_callback(timestamp, data, logconf):
##    x = data['kalman.stateX']
##    y = data['kalman.stateY']
##    z = data['kalman.stateZ']
    x = data['ext_pos.X']
    y = data['ext_pos.Y']
    z = data['ext_pos.Z']
##    for key in logconf.__dict__["variables"]:
##        print(key.__dict__)
    print('ext_pos: ({}, {}, {})'.format(x, y, z))


def start_position_printing(scf):
    log_conf = LogConfig(name='Position', period_in_ms=500)
    log_conf.add_variable('kalman.stateX', 'float')
    log_conf.add_variable('kalman.stateY', 'float')
    log_conf.add_variable('kalman.stateZ', 'float')
    scf.cf.log.add_config(log_conf)
    log_conf.data_received_cb.add_callback(position_callback)
    log_conf.start()



def receiveNewFrame( frameNumber, markerSetCount, unlabeledMarkersCount, rigidBodyCount, skeletonCount,
                    labeledMarkerCount, timecode, timecodeSub, timestamp, isRecording, trackedModelsChanged ):
    pass
##    print( "Received frame", frameNumber )

# This is a callback function that gets connected to the NatNet client. It is called once per rigid body per frame


def receiveRigidBodyFrame( id, position, rotation ):
##    print( "Received frame for rigid body", id, position, rotation )
#   store position in a list for thread to send
    l.append(position)
    print(position)



def run_sequence(scf, sequence):
    cf = scf.cf
    
    for position in sequence:
        
        for i in range(100):
            print('Setting position {}'.format(position))
##            pass
            cf.commander.send_setpoint(0,0,0,32767)
##            cf.commander.send_position_setpoint(position[0],
##                                                position[1],
##                                                position[2],
##                                                position[3])
##            cf.commander.send_position_setpoint(1,0,2,0)
            time.sleep(0.1)
##
    cf.commander.send_stop_setpoint()
    # Make sure that the last packet leaves before the link is closed
    # since the message queue is not flushed before closing
    time.sleep(0.1)


def thre(scf,ext,stop):
        fly = True
        while fly==True:
            try:
                global l
                loca = l[-1]
                l = l[1:]
##                print('loca\n')
##                print(loca[0],loca[1],loca[2])
                ext.send_extpos(loca[0],loca[1],loca[2])
##                fly=False
                if stop():
                    fly=False
            except:
                pass

if __name__ == "__main__":
    streamingClient = natfile.NatNetClient.NatNetClient()
    streamingClient.newFrameListener = receiveNewFrame
    streamingClient.rigidBodyListener = receiveRigidBodyFrame


    cflib.crtp.init_drivers(enable_debug_driver=False)

    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
#   if kalman filter is enabled, the reset estimator will loop because
#   the PX,PY,PZ continuously increase, so disable this function.
##        reset_estimator(scf)
        print("start CF")
#   create a table to store the external position data. 
        logPos = LogConfig(name='position', period_in_ms=10)
        logPos.add_variable('ext_pos.X', 'float')
        logPos.add_variable('ext_pos.Y', 'float')
        logPos.add_variable('ext_pos.Z', 'float')
        with SyncLogger(scf, logPos) as logger:
            print("start logger")
##            endTime = time.time() + 10
##            scf.cf.loc.send_extpos([0,0,0])
##            scf.cf.high_level_commander.takeoff(0.8,0,10)
##            time.sleep(1)
            scf.cf.loc.send_extpos([1,2,0.8])
            
            
            scf.cf.log.add_config(logPos)
            logPos.data_received_cb.add_callback(position_callback)
            logPos.start()
            
            
##            start_position_printing(scf)

            try:
###   create an object which could send the external position to CF
##            ext = Extpos(scf.cf)
###   start receiving the position data from Optitrack
                streamingClient.run()
                print("start streaming")
###   create a thrread to send external position to CF
                thread1 = threading.Thread(target = thre, args = (scf,ext,lambda : stop_threads))
                thread1.start()
                stop_threads = True
                thread1.join() 
                while True:
                    print(logger.next())
##            for i in range(50):
##                ext.send_extpos(1,1,1)
##                scf.cf.commander.send_position_setpoint(1,1,2,0)
##                time.sleep(0.1)
##        run_sequence(scf, sequence)
            except:
                pass
            scf.cf.close_link()
