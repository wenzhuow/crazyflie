import natfile.NatNetClient
import threading
import time
import math
import tkinter
from collections import defaultdict 
side = 48

def calibration_from_location(x,y):
    return x,y
            
def calculate_draw(x,y):
    return x-10,y-10,x+10,y+10

def calculate_oncanvas(location):
    x1,y1 = calibration_from_location(location[0][0],location[0][1])
    x2,y2 = calibration_from_location(location[1][0],location[1][1])

    return math.floor(x1/side)*side+side/2,math.floor(y1/side)*side+side/2,math.floor(x2/side)*side+side/2,math.floor(y2/side)*side+side/2


            
class Projection:
    def __init__(self):
        self.pos = {}
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
        self.pos.append(position)
        if id in self.pos.keys():
            self.pos[id].append(position)
        else:
            self.pos[id]=[]
            self.pos[id].append(position)
            
        
    def locating():
        self.streamingClient.run()


                
        
class draw_environment:

    def __init__(self):
        self._root_window = tkinter.Tk()

##fullscreen map, F11 to enter fullscreen, Esc to quit
        self._root_window.attributes('-fullscreen', True)
        self._root_window.bind("<F11>", lambda event: self._root_window.attributes("-fullscreen",
                                    not self._root_window.attributes("-fullscreen")))
        self._root_window.bind("<Escape>", lambda event: self._root_window.attributes("-fullscreen", False))
        
        
        self.pro = Projection()

##virtual quadcopter intial location
        self.test_d = {}
        self.test_d[0] = (60,85)


        
        self._canvas = tkinter.Canvas(
            master = self._root_window, width = 700, height = 700,
            background = 'green')
        self._canvas.bind('<Button-1>', self._left_click)
        self._canvas.pack(fill='both', expand=True)

##init map, 0 means normal, 1 means selected
        self._side = side
        self._v_map = defaultdict(dict)         

##init path list
        self._path = defaultdict(list) 
        
##        l = []
##        for i in range(math.floor(1536/self._side)):
##            l.append(0)
##        for i in range(math.floor(864/self._side)):
##            self._v_map.append(l)
            
##        for i in range(int(864/self._side)):
##            print(v_map[i])
                

##virtual quadcopter flying, updating location
        thread1 = threading.Thread(target = self.flying, args = ())
        thread1.start()

        
        self.draw_background()
        
##draw all location on canvas
        self.update_map()
        
    def run(self) -> None:
        self._root_window.mainloop()

    def draw_background(self):
##        print("here")
##        print(self._root_window.winfo_screenwidth())
##        print(self._root_window.winfo_screenheight())
        
        self._side = side
        for i in range(1,math.ceil(1920/self._side)):
            self._canvas.create_line(self._side*i,0,self._side*i,1000,width=2, fill='black')
        for i in range(1,math.ceil(1024/self._side)):
            self._canvas.create_line(0,self._side*i,1920,self._side*i,width=2, fill='black')



    def _left_click(self, event: tkinter.Event) -> None:
        global x,y
        x = event.x
        y = event.y
        self._v_map[math.floor(x/self._side)][math.floor(y/self._side)] = 1
##        if self._v_map[math.floor(x/self._side)] != None or self._v_map[math.floor(x/self._side)] != []:
##            self._v_map[math.floor(x/self._side)].append(math.floor(y/self._side))
##        else:
##            self._v_map[math.floor(x/self._side)] = [math.floor(y/self._side)]
##        print(self._v_map)
        
        
    def flying(self):
##update virtual quadcopter
        x = 4.4
        y = 0
        self._path[0].append((self.test_d[0][0],self.test_d[0][1]))
        catching = False
        time.sleep(2)
        while True:
##        self.test_d[1][0]< self._canvas.winfo_width() and self.test_d[1][1]< self._canvas.winfo_height():
            for i in range(0,1):


                if not catching:
##                    print(self._v_map.keys())
##                    print(self.test_d)
                    if len(self._v_map.keys()) == 0:
                        
                        new_pos = (self.test_d[i][0]+x,self.test_d[i][1]+y)
##                        try:    
##                            if self._v_map[math.floor(new_pos[0]/self._side)][math.floor(new_pos[1]/self._side)]==1:
##                                del self._v_map[math.floor(new_pos[0]/self._side)][math.floor(new_pos[1]/self._side)]
##                            if len(self._v_map[math.floor(new_pos[0]/self._side)]) == 0:
##                                del self._v_map[math.floor(new_pos[0]/self._side)]
##                        except:
##                            pass
                        
                        if new_pos[0]>=1536-self._side or new_pos[0]<=self._side:
                            x= -x
                        if new_pos[1]>=864-self._side or new_pos[1]<=self._side:
                            y= -y
                            
                    else:
##                        print(self._v_map)
                        for target_x in self._v_map.keys():
                            for target_y in self._v_map[target_x].keys():
                                temp_x = (target_x+0.5)*self._side - self.test_d[i][0]
                                temp_y = (target_y+0.5)*self._side - self.test_d[i][1]
                                print(target_x,target_y)
                                print(temp_x,temp_y)
                                large = max(abs(int(temp_x/3)),abs(int(temp_y/3)))
                                x,y = temp_x/large    ,temp_y/large
                            catching = True
                            break
                        break
                    

                else:
##                    print(x,y)
                    new_pos = (self.test_d[i][0]+x,self.test_d[i][1]+y)

                    if math.floor(new_pos[0]/self._side) in self._v_map.keys():
                        if math.floor(new_pos[1]/self._side) in self._v_map[math.floor(new_pos[0]/self._side)].keys():
                        
                            if self._v_map[math.floor(new_pos[0]/self._side)][math.floor(new_pos[1]/self._side)] ==1:

                            
                                del self._v_map[math.floor(new_pos[0]/self._side)][math.floor(new_pos[1]/self._side)]
                                catching = False
                        if len(self._v_map[math.floor(new_pos[0]/self._side)]) == 0:
                            del self._v_map[math.floor(new_pos[0]/self._side)]
           
                    

                self.test_d[i] = new_pos
                time.sleep(0.04)

           
    def update_map(self):

##draw start location of quadcopter

        
##        self._canvas.create_rectangle(calculate_oncanvas(self.test_d[1]),width=2, fill="#fb1")


##clean previous drawing
        
        for o in self._canvas.find_withtag('special'):
            self._canvas.delete(o)


##test for virtual quadccopter location
            
##        for t in range(0,30):
##            time.sleep(0.5)
        for k in self.test_d.keys():
##            print(calculate_oncanvas(self.test_d[k]))

            pos_in_map = (math.floor(self.test_d[k][0]/side)*side+side/2,math.floor(self.test_d[k][1]/side)*side+side/2)

            
            if pos_in_map != self._path[k][-1]:
                self._path[k].append(pos_in_map)
            if len(self._path[k])>50:
                self._path[k] = self._path[k][1:]
            for i in range(len(self._path[k])-1):
                self._canvas.create_line(calculate_oncanvas((self._path[k][i],self._path[k][i+1])),width=2,fill="#fb1",tag = 'special')


        for x in list(self._v_map.keys()):
            for y in list(self._v_map[x].keys()):
                try:
                    if self._v_map[x][y]==1:
                        self._canvas.create_rectangle(x*self._side,y*self._side,(x+1)*self._side,(y+1)*self._side,fill= "red",tag='special')
                except:
                    pass

                
##draw location of each real quadcopter on canvas

##        for k in self.pro.pos.keys():
##            loca = self.pro.pos[k][-1]
##            x1,y1,x2,y2 = calculate_oncanvas(loca)
##            canvas.create_rectangle(x1,y1,x2,y2,outline="#fb0", fill="#fb0")



##repeat drawing new location

        self._root_window.after(5, self.update_map)




if __name__ == '__main__':
    simulated_environment = draw_environment()
    simulated_environment.run()
