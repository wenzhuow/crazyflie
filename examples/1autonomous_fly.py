import logging
import time
from threading import Thread

import cflib
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
import natfile.NatNetClient
from cflib.crazyflie.swarm import Swarm
import threading


logging.basicConfig(level=logging.ERROR)


pos=[]




    
##    with SyncLogger(scf, log_config) as logger:
##        for log_entry in logger:
##            data = log_entry[1]
##
##            var_x_history.append(data['kalman.varPX'])
##            var_x_history.pop(0)
##            var_y_history.append(data['kalman.varPY'])
##            var_y_history.pop(0)
##            var_z_history.append(data['kalman.varPZ'])
##            var_z_history.pop(0)
##
##            min_x = min(var_x_history)
##            max_x = max(var_x_history)
##            min_y = min(var_y_history)
##            max_y = max(var_y_history)
##            min_z = min(var_z_history)
##            max_z = max(var_z_history)
##
##            # print("{} {} {}".
##            #       format(max_x - min_x, max_y - min_y, max_z - min_z))
##
##            if (max_x - min_x) < threshold and (
##                    max_y - min_y) < threshold and (
##                    max_z - min_z) < threshold:
##                break


def reset_estimator(scf):
    cf = scf.cf
    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')

    wait_for_position_estimator(cf)

    
class AltHoldExample:

    def __init__(self, link_uri):
        """ Initialize and run the example with the specified link_uri """

        self._cf = Crazyflie()

        self._cf.connected.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)

        self._cf.open_link(link_uri)
        self.is_connected = True
        print('Connecting to %s' % link_uri)

        self.pos = []
        self.streamingClient = natfile.NatNetClient.NatNetClient()
        self.streamingClient.newFrameListener = self.receiveNewFrame
        self.streamingClient.rigidBodyListener = self.receiveRigidBodyFrame


    def receiveNewFrame(self, frameNumber, markerSetCount, unlabeledMarkersCount, rigidBodyCount, skeletonCount,
                    labeledMarkerCount, timecode, timecodeSub, timestamp, isRecording, trackedModelsChanged ):
        pass
##    print( "Received frame", frameNumber )
        
    def receiveRigidBodyFrame(self, id, position, rotation ):
    ##    print( "Received frame for rigid body", id, position, rotation )
    #   store position in a list for thread to send
##        self.pos.append(position)
        pos.append(position)
##        print(position)
##        print(self.pos)
    
    
    def _connected(self, link_uri):
        """ This callback is called form the Crazyflie API when a Crazyflie
        has been connected and the TOCs have been downloaded."""
        print('Connected to %s' % link_uri)
        Thread(target=self._hover_test).start()

    def _connection_failed(self, link_uri, msg):
        """Callback when connection initial connection fails (i.e no Crazyflie
        at the specified address)"""
        print('Connection to %s failed: %s' % (link_uri, msg))

    def _connection_lost(self, link_uri, msg):
        """Callback when disconnected after a connection has been made (i.e
        Crazyflie moves out of range)"""
        print('Connection to %s lost: %s' % (link_uri, msg))

    def _disconnected(self, link_uri):
        """Callback when the Crazyflie is disconnected (called in all cases)"""
        print('Disconnected from %s' % link_uri)
        self.is_connected = False

    def position_callback(self,timestamp, data, logconf):
##        x = data['kalman.varPX']
##        y = data['kalman.varPY']
##        z = data['kalman.varPZ']
        x = data['ext_pos.X']
        y = data['ext_pos.Y']
        z = data['ext_pos.Z']
    ##    for key in logconf.__dict__["variables"]:
    ##        print(key.__dict__)
##        print('ext_pos: ({}, {}, {})'.format(x, y, z))

        
    def VARP_callback(self,timestamp, data, logconf):
        x = data['kalman.varPX']
        y = data['kalman.varPY']
        z = data['kalman.varPZ']
##        x = data['ext_pos.X']
##        y = data['ext_pos.Y']
##        z = data['ext_pos.Z']
    ##    for key in logconf.__dict__["variables"]:
    ##        print(key.__dict__)
        print('varP xyz: ({}, {}, {})'.format(x, y, z))


    def wait_for_position_estimator(self,cf):
        print('Waiting for estimator to find position...')

        log_config = LogConfig(name='Kalman Variance', period_in_ms=1000)
        log_config.add_variable('kalman.varPX', 'float')
        log_config.add_variable('kalman.varPY', 'float')
        log_config.add_variable('kalman.varPZ', 'float')

        var_y_history = [1000] * 10
        var_x_history = [1000] * 10
        var_z_history = [1000] * 10

        threshold = 0.001


        cf.log.add_config(log_config)
        log_config.data_received_cb.add_callback(self.VARP_callback)
        log_config.start()
    
    def thre(self,cf):
        print("start")
        fly = True
        t=0
        logPos = LogConfig(name='position', period_in_ms=10)
        logPos.add_variable('ext_pos.X', 'float')
        logPos.add_variable('ext_pos.Y', 'float')
        logPos.add_variable('ext_pos.Z', 'float')
        cf.log.add_config(logPos)
        logPos.data_received_cb.add_callback(self.position_callback)
        logPos.start()
        while fly==True:
            try:
                loca = pos[-1]
##                loca = self.pos[-1]
##                self.pos = self.pos[1:]
##                print('loca\n')
                
                cf.loc.send_extpos([loca[2],loca[0],loca[1]])
##                print('loca ',loca[0],loca[1],loca[2],t)
##                print(logger.next())
##                t+=0.1
                time.sleep(0.01)
            except:
                pass
##                print("no data")
                
    def _hover_test(self):

        self.streamingClient.run()
        thread1 = threading.Thread(target = self.thre, args = (self._cf,))
        thread1.start()
##        thread2 = threading.Thread(target = self.wait_for_position_estimator, args = (self._cf,))
##        thread2.start()
##        time.sleep(1)
##        print(pos)
##        self.wait_for_position_estimator(self._cf)
##        self._cf.high_level_commander.takeoff(0.5,0,2)
##        self._cf.commander.send_position_setpoint(pos[-1][0],pos[-1][1],pos[-1][2]+0.2,0)


##        time.sleep(2)
        
##        self._cf.loc.send_extpos([pos[-1][0],pos[-1][1],pos[-1][2]])
        
        self._cf.commander.send_setpoint(0,0,0,32767);
        time.sleep(0.1);

        print("putting in althold")
        self._cf.param.set_value("flightmode.althold","True")
##        self._cf.param.set_value("kalman.initialX",pos[-1][0])
##        self._cf.param.set_value("kalman.initialY",pos[-1][1])
##        self._cf.param.set_value("kalman.initialZ",pos[-1][2])
##        
        print("Stay in althold for 3s")
        time.sleep(0.1);
        it=0

##        op = pos[-1]
##        print("original pos: ",op)


        while it<300:
##            self._cf.commander.send_position_setpoint(0,0,0.5,0)
##            cur = pos[-1]
##            self._cf.loc.send_extpos([cur[0],cur[1],cur[2]])

##            self._cf.commander.send_setpoint(0,0,0,32767)
            self._cf.param.set_value("flightmode.althold","True")

            
            self._cf.commander.send_setpoint(0,0,0,32767)
##            self._cf.commander.send_position_setpoint(op[2],op[0],op[1]+0.2,0)
            
            time.sleep(0.01)
            it+=1


        print("Close connection")
        self._cf.commander.send_setpoint(0,0,0,0)
        self._cf.close_link()


if __name__ == '__main__':
    
    
    cflib.crtp.init_drivers(enable_debug_driver=False)
    print('Scanning interfaces for Crazyflies...')
    available = cflib.crtp.scan_interfaces()
    print('Crazyflies found:')
    for i in available:
        print(i[0])

    if len(available) > 0:
        le = AltHoldExample(available[0][0])
    else:
        print('No Crazyflies found, cannot run example')
