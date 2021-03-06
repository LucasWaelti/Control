import tkinter
import math
import time
import random

global tk,canvas
global width,height

class Physics():
    ''' This class provides the physics for the simulation
        of a single inverted pendulum'''
    def __init__(self):
        self.m = 0.1  # kg
        self.l = 0.1  # m
        self.g = 9.81 # m/s^2
        self.phi = -0.2  # rad (small offset to get out of equilibrium)
        self.phi_dot = 0    # rad/s
        self.phi_ddot = 0   # rad/s^2
        self.fallen = False # Has the pendulum fallen?
        self.out_of_range = False # Has the support gone too far?
        self.friction = True
        self.mu = 0.0008 # Friction coefficient

        self.acc = 0  # m/s^2 (acceleration of the support)
        self.v = 0    # m/s (speed of support)
        self.p = 0    # m (position of support)
        self.M = 0    # Nm (disturbence momentum)

        self.t = 0.01 # sec (sampling time)
        return

    def calculate_friction(self):
        self.M = -self.mu * self.phi_dot
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
            #self.phi= math.pi/2
            self.fallen = True
        elif(self.phi <= -math.pi/2):
            #self.phi = -math.pi/2
            self.fallen = True

    def update_speed(self):
        self.v += self.acc * self.t
    def update_position(self):
        self.p += self.v * self.t
        if(math.fabs(self.p) > width/2/1000):
            self.out_of_range = True

    def update_physics(self):
        if(self.friction is True):
            self.calculate_friction()
        self.calculate_phi_ddot()
        self.update_phi_dot()
        self.update_phi()
        self.update_speed()
        self.update_position()
            
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
        self.max_val = 0
        return

    def set_PID(self,p=50,i=1,d=1.9):
        self.P = p
        self.I = i
        self.D = d
    def set_target(self, target=0):
        self.target = target
    def set_max_control_value(self,max_val=4):
        self.max_val = max_val

    def control(self,value,t):
        self.error = self.target - value 
        p = self.error * self.P
        d = (self.error - self.pre_error)/t * self.D
        self.i += self.error*t 

        self.action = p + d + self.i * self.I
        self.pre_error = self.error

        if(self.action > self.max_val):
            self.action = self.max_val
        elif(self.action < -self.max_val):
            self.action = -self.max_val

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
    tk.title("Inverted Pendulum Simulation")
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

def control_end_of_process(sim,pid,stick,ball,support,timer):
    if(sim.out_of_range or sim.fallen or timer >= 100):
            if(sim.fallen):
                # Condition for stability
                limit = pid.max_val * sim.t / 2
                # Stop the cart
                while(math.fabs(sim.v) > limit):
                    if(sim.out_of_range):
                        break
                    if(sim.v > 0):
                        sim.acc = -pid.max_val
                    else:
                        sim.acc = pid.max_val
                    sim.update_physics()
                    update_display(sim,stick,ball,support)
                    time.sleep(sim.t)
                sim.acc = 0
                sim.v = 0
                # Let the pendulum swing down for animation
                for _ in range(300):
                    if(sim.out_of_range):
                        break
                    sim.update_physics()
                    update_display(sim,stick,ball,support)
                    time.sleep(sim.t)
            # Reset everything
            sim.reset()
            pid.reset()
            update_display(sim,stick,ball,support)
            time.sleep(1)
            return 0
    elif(math.fabs(sim.phi) < 0.01):
            timer += 1
    return timer

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
    pid.set_PID(p=200,i=4000,d=15) # p=50,i=1000,d=10
    pid.set_target(target=0)
    pid.set_max_control_value(max_val=20)

    #Build the inverted pendulum
    update_display(sim,stick,ball,support)

    timer = 0

    # Run the simulation
    while(True):
        # Compute control commands
        sim.acc = pid.control(value=sim.phi,t=sim.t)
        # Update physics
        sim.update_physics()
        # Update graphics
        update_display(sim,stick,ball,support)
        
        time.sleep(sim.t*1)

        timer = control_end_of_process(sim,pid,stick,ball,support,timer)

    tk.mainloop()

if __name__ == "__main__":
    main()
