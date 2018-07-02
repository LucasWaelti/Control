import tkinter
import math
import time
import random

global tk,canvas
global width,height
global reset_phi

class Physics():
    ''' 
        This class provides the physics for the simulation
        of a single inverted pendulum
    '''
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
        global reset_phi
        self.phi = reset_phi  # rad (small offset to get out of equilibrium)
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
        self.P = 200
        self.I = 4000
        self.i = 0 # Integral value
        self.D = 15
        self.action = 0
        self.max_val = 20
        return

    def set_PID(self,p=200,i=4000,d=15): # p=50,i=1000,d=10
        self.P = p
        self.I = i
        self.D = d
    def set_target(self, target=0):
        self.target = target
    def set_max_control_value(self,max_val=20):
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

class Graphics():
    ''' 
        This class provides everything needed 
        for the display of a single pendulum 
    '''
    def __init__(self,color="orange"):
        global canvas
        # Graphics parameters
        self.color = color
        self.support_dim = (30,10)
        self.stick_dim = 100
        self.ball_dim = 20
        # Graphical objects
        self.support = canvas.create_rectangle(-self.support_dim[0],self.support_dim[1],self.support_dim[0],-self.support_dim[1],fill="grey")
        self.stick = canvas.create_line(0,0,0,self.stick_dim,width=5,fill="blue")
        self.ball = canvas.create_oval(-self.ball_dim,-self.ball_dim,self.ball_dim,self.ball_dim,fill=self.color)

    def cc(self,x,y):
        '''Coordinates Converter, from centered to window'''
        global width,height
        n = []
        n.append(x + width/2)
        n.append(-y + height/2)
        return n
    def mp(self,m):
        '''Convert meter to pixel'''
        p = 1000*m
        return p

    def update_display(self,sim:Physics):
        global tk,canvas
        # Support position 
        x = sim.p
        y = 0
        x = self.mp(x)
        y = self.mp(y)
        sup_pos = self.cc(x,y)

        # Ball position 
        x,y = sim.generate_mass_coord()
        x = self.mp(x)
        y = self.mp(y)
        ball_pos = self.cc(x,y)

        # Update display
        canvas.coords(self.stick,sup_pos[0],sup_pos[1],ball_pos[0],ball_pos[1])
        canvas.coords(self.ball,ball_pos[0]-self.ball_dim,ball_pos[1]-self.ball_dim,ball_pos[0]+self.ball_dim,ball_pos[1]+self.ball_dim)
        canvas.coords(self.support,sup_pos[0]-self.support_dim[0],sup_pos[1]+self.support_dim[1],sup_pos[0]+self.support_dim[0],sup_pos[1]-self.support_dim[1])
        tk.update()

class Pendulum():
    def __init__(self,controller,color="orange"):
        self.sim = Physics()
        self.controller = controller()
        self.graphics = Graphics(color)
        self.timer = 0
        self.failed = False # Indicates if stabilisation occured or failed
        self.succeeded = False
        self.is_done = False

        # Display the pendulum in window
        self.graphics.update_display(self.sim)

    def step_simulation(self,control_enabled=True):
        if(control_enabled):
            # Compute control commands
            self.sim.acc = self.controller.control(value=self.sim.phi,t=self.sim.t)
        # Update physics
        self.sim.update_physics()
        if(not control_enabled):
            self.sim.acc = 0
            self.sim.v = 0
        # Update graphics
        self.graphics.update_display(self.sim)

    def check_if_done(self):
        if(self.sim.out_of_range or self.sim.fallen):
            self.failed = True
            self.is_done = True
        elif(math.fabs(self.sim.phi) < 0.01):
                self.timer += 1

        if(self.timer >= 100):
            self.succeeded = True
            self.is_done = True
            
    def step(self):
        if(not self.failed and not self.succeeded):
            self.step_simulation()
            self.check_if_done()
        elif(self.failed):
            self.step_simulation(control_enabled=False)
        elif(self.succeeded):
            self.step_simulation(control_enabled=True)

    def reset(self):
        # Reset everything
        self.sim.reset()
        self.controller.reset()
        self.graphics.update_display(self.sim)

        self.failed = False
        self.succeeded = False
        self.is_done = False
        self.timer = 0


def setup_window():
    global tk,canvas,width,height
    tk = tkinter.Tk()
    tk.title("Inverted Pendulum Simulation")
    width = 1500#500
    height = 400
    canvas = tkinter.Canvas(tk,width=width,height=height)
    canvas.pack()
    # Create the rail
    rail = canvas.create_line(0,height/2,width,height/2,width=2,fill="black")
    zero = canvas.create_line(width/2,height/2-5,width/2,height/2+5,width=2,fill="black")

def generate_random_start():
    global reset_phi
    reset_phi = random.getrandbits(8)/255
    s = random.getrandbits(1) - 0.5
    if(s >= 0):
        s = 1
    else:
        s = -1
    reset_phi *= s

def run(*p):
    global reset_phi

    n = len(p)
    random.seed()
    while(True):
        # Update all pendulums
        for i in range(0,n):
            p[i].step()
        
        time.sleep(p[0].sim.t*1)

        # Check if reset is needed
        for i in range(0,n):
            done = p[i].is_done
            if(not done):
                break
        if(done):
            generate_random_start()
            for i in range(0,n):
                p[i].reset()
            time.sleep(1)

def main():
    global tk,canvas

    # Initialise the window
    setup_window()
    
    # Create pendulums
    p1 = Pendulum(PIDcontroller)            # 1: Use default settings p=200,i=4000,d=15
    p2 = Pendulum(PIDcontroller,"blue")      
    p2.controller.set_PID(p=39.91,i=1010,d=10.2) # 2: N(s) = (s+1+10i)(s+1-10i)(s+100)
    p3 = Pendulum(PIDcontroller,"red")        
    p3.controller.set_PID(p=1009.81,i=0,d=20) # 3: N(s) = s(s+100)^2

    # Run the simulation
    run(p1,p2,p3)

    tk.mainloop()

if __name__ == "__main__":
    main()
