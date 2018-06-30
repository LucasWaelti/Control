import tkinter
import math
import time
import random

global tk,canvas
global width,height

class Physics():
    ''' This class provides the physics for the simulation'''
    def __init__(self):
        self.m = 0.1  # kg
        self.l = 0.1  # m
        self.g = 9.81 # m/s^2
        self.phi = -0.3  # rad (small offset to get out of equilibrium)
        self.phi_dot = 0    # rad/s
        self.phi_ddot = 0   # rad/s^2
        self.fallen = False # Has the pendulum fallen?
        self.out_of_range = False # Has the support gone too far?

        self.acc = 0  # m/s^2 (acceleration of the support)
        self.v = 0    # m/s (speed of support)
        self.p = 0    # m (position of support)
        self.M = 0    # Nm (disturbence momentum)

        self.t = 0.01 # sec (sampling time)
        return
    
    def calculate_phi_ddot(self):
        self.phi_ddot = self.g/self.l * math.sin(self.phi)
        self.phi_ddot += 1/self.l * math.cos(self.phi) * self.acc
        self.phi_ddot += 1/(self.m*self.l**2) * self.M
    def update_phi_dot(self):
        self.phi_dot += self.phi_ddot * self.t
    def update_phi(self):
        self.phi +=  self.phi_dot * self.t
        #return # Uncomment to let the pendulum swing
        if(self.phi >= math.pi/2):
            self.phi= math.pi/2
            self.fallen = True
        elif(self.phi <= -math.pi/2):
            self.phi = -math.pi/2
            self.fallen = True

    def update_speed(self):
        self.v += self.acc * self.t
    def update_position(self):
        self.p += self.v * self.t
        if(math.fabs(self.p) > width/2/1000):
            self.out_of_range = True
            
    def generate_mass_coord(self):
        x = self.p - self.l * math.sin(self.phi)
        y = self.l * math.cos(self.phi)
        return x,y

    def reset(self):
        random.seed()
        r = random.getrandbits(8)/255
        s = random.getrandbits(1) - 0.5
        if(s >= 0):
            s = 1
        else:
            s = -1
        r *= s
        self.phi = r  # rad (small offset to get out of equilibrium)
        self.phi_dot = 0    # rad/s
        self.phi_ddot = 0   # rad/s^2
        self.fallen = False # Has the pendulum fallen?
        self.out_of_range = False # Has the support gone too far?
        self.acc = 0  # m/s^2 (acceleration of the support)
        self.v = 0    # m/s (speed of support)
        self.p = 0    # m (position of support)
        self.M = 0    # Nm (disturbence momentum)
        return

class PIDcontroller():
    '''
    Stability conditions for the pendulum: 
    - "I/D < (P-g)/l" 
    - "P > g"
    '''
    def __init__(self):
        self.target = 0
        self.error = 0
        self.pre_error = 0
        self.P = 0
        self.I = 0
        self.i = 0 # Integral value
        self.D = 0
        self.action = 0
        self.acc_max = 0
        return

    def set_PID(self,p=50,i=1,d=1.9):
        self.P = p
        self.I = i
        self.D = d
    def set_target(self, target=0):
        self.target = target
    def set_max_acceleration(self,acc=4):
        self.acc_max = acc

    def control(self,value,t):
        self.error = self.target - value 
        p = self.error * self.P
        d = (self.error - self.pre_error)/t * self.D
        self.i += self.error*t 

        self.action = p + d + self.i * self.I
        self.pre_error = self.error

        if(self.action > self.acc_max):
            self.action = self.acc_max
        elif(self.action < -self.acc_max):
            self.action = -self.acc_max

        return self.action

    def reset(self):
        self.i = 0
        self.error = 0
        self.pre_error = 0

def cc(x,y):
    '''Coordinates Converter, from centered to window'''
    global width,height
    n = []
    n.append(x + width/2)
    n.append(-y + height/2)
    return n
def mp(m):
    '''Convert meter to pixel'''
    p = 1000*m
    return p

def setup_window():
    global tk,canvas,width,height
    tk = tkinter.Tk()
    width = 1500#500
    height = 400
    canvas = tkinter.Canvas(tk,width=width,height=height)
    canvas.pack()
    
def create_support():
    global canvas
    support = canvas.create_rectangle(-30,10,30,-10,fill="grey")
    return support
def create_stick():
    global canvas
    stick = canvas.create_line(10,10,10,110,width=5,fill="blue")
    return stick
def create_rail():
    global canvas
    rail = canvas.create_line(0,height/2,width,height/2,width=2,fill="black")
    zero = canvas.create_line(width/2,height/2-5,width/2,height/2+5,width=2,fill="black")
    return rail
def create_ball():
    global canvas
    ball = canvas.create_oval(-20,-20,20,20,fill="orange")
    return ball

def update_display(sim,stick,ball,support):
    global tk,canvas
    # Support position 
    x = sim.p
    y = 0
    x = mp(x)
    y = mp(y)
    sup_pos = cc(x,y)

    # Ball position 
    x,y = sim.generate_mass_coord()
    x = mp(x)
    y = mp(y)
    ball_pos = cc(x,y)

    # Update display
    canvas.coords(stick,sup_pos[0],sup_pos[1],ball_pos[0],ball_pos[1])
    canvas.coords(ball,ball_pos[0]-20,ball_pos[1]-20,ball_pos[0]+20,ball_pos[1]+20)
    canvas.coords(support,sup_pos[0]-30,sup_pos[1]+10,sup_pos[0]+30,sup_pos[1]-10)
    tk.update()


def main():
    global tk,canvas
    setup_window()
    
    #Create objects (order matters)
    rail = create_rail()
    support = create_support()
    stick = create_stick()
    ball = create_ball()

    # Initialize the physics
    sim = Physics()

    # Initialize the controller for the pendulum
    pid = PIDcontroller()
    pid.set_PID(p=50,i=1000,d=10)
    pid.set_max_acceleration(acc=20)
    # Initialize the controller for the support
    p_sup = PIDcontroller()
    p_sup.set_PID(p=10,i=0,d=0)
    p_sup.set_max_acceleration(acc=4)

    #Build the inverted pendulum
    update_display(sim,stick,ball,support)

    timer = 0

    # Run the simulation
    while(True):
        # Calculate control commands
        sim.acc = pid.control(value=sim.phi,t=sim.t)
        if(timer % 2 == 0 and 0):
            sim.acc += p_sup.control(value=sim.p,t=sim.t)

        # Update the physics
        sim.calculate_phi_ddot()
        sim.update_phi_dot()
        sim.update_phi()
        sim.update_speed()
        sim.update_position()

        update_display(sim,stick,ball,support)
        
        time.sleep(sim.t*1)

        if(sim.out_of_range or sim.fallen or timer >= 100):# and math.fabs(sim.p) < 0.01):
            if(sim.fallen):
                time.sleep(1)
            sim.reset()
            pid.reset()
            p_sup.reset()
            update_display(sim,stick,ball,support)
            timer = 0
            time.sleep(1)

        if(math.fabs(sim.phi) < 0.01):
            timer += 1

    tk.mainloop()

if __name__ == "__main__":
    main()
