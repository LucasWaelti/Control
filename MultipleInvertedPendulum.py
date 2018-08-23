# http://python-control.readthedocs.io/en/latest/generated/control.lqr.html
from control import lqr
from numpy import matrix,array
import tkinter
import math
import time
import random

global tk,canvas
global width,height
global reset_phi

class Physics():
    ''' 
        DEPRECATED -> will no longer work.
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

class CartPendulumPhysics():
    ''' 
        This class provides the physics for the simulation
        of a single inverted pendulum with a cart with a mass.
        The dimensions respect the benchmark dimensions from
        www.robotbenchmark.net/inverted_pendulum
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
        self.mu = 0.0008 # Friction coefficient for the sliding axis
        self.nu = 0.0008 # Friction coefficient for the rotating axis
        self.de = 0.0008 # Friction coefficient due to air resistance

        self.Mass = 0.16 # kg (mass of the cart)
        self.acc = 0  # m/s^2 (acceleration of the support)
        self.v = 0    # m/s (speed of support)
        self.p = 0    # m (position of support)

        self.M = 0    # Nm (disturbence/control momentum)
        self.F = 0    # N  (disturbance/control force)

        self.command = 0 # m/s**2 or N (command computed by the controller)
        self.controlled_entity = "a"

        self.t = 0.01 # sec (sampling time)
        return

    def update_command(self,c):
        self.command = c

    def apply_force_command(self):
        if(self.controlled_entity == "F"):
            self.F += self.command
    def apply_acceleration_command(self):
        if(self.controlled_entity == "a"):
            self.acc = self.command

    def calculate_friction(self):
        self.M += -self.nu * self.phi_dot + self.de * self.v 
        self.F += -self.mu * self.v 
    def calculate_phi_ddot(self):
        phi_ddot = self.g/self.l * math.sin(self.phi)
        phi_ddot += 1/self.l * math.cos(self.phi) * self.acc
        phi_ddot += 1/(self.m*self.l**2) * self.M
        return phi_ddot
    def update_phi_dot(self):
        self.phi_dot += self.phi_ddot * self.t
    def update_phi(self):
        self.phi +=  self.phi_dot * self.t
        if(self.phi >= math.pi/2):
            #self.phi= math.pi/2
            self.fallen = True
        elif(self.phi <= -math.pi/2):
            #self.phi = -math.pi/2
            self.fallen = True

    def calculate_acc(self):
        C = self.m*self.l/(self.Mass + self.m)
        acc = C * math.cos(self.phi) * self.phi_ddot
        acc += C * math.sin(self.phi) * self.phi_dot**2
        acc += 1/(self.Mass + self.m) * self.F 
        return acc 
    def update_speed(self):
        self.v += self.acc * self.t
    def update_position(self):
        self.p += self.v * self.t
        if(math.fabs(self.p) > width/2/1000):
            self.out_of_range = True

    def update_physics(self):
        self.M = 0
        self.F = 0

        # Construct external forces
        if(self.friction is True):
            self.calculate_friction()
        if(self.controlled_entity == "F"):
            self.apply_force_command()

        # Compute new accelerations 
        if(self.controlled_entity == "a"):
            self.apply_acceleration_command()
            self.phi_ddot = self.calculate_phi_ddot()
        else:
            # (make sure to consider values of the same time interval!!)
            phi_ddot_new = self.calculate_phi_ddot()
            self.acc = self.calculate_acc()
            self.phi_ddot = phi_ddot_new

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
        Controller will directly act on the speed of the cart. 
    '''
    def __init__(self):
        self.variable = "phi" # or "p" (Indicates what is to be regulated)
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

    def select_variable(self,var="phi"):
        self.variable = var

    def control(self,sim):
        # Autonomously compute the error
        t = sim.t
        if(self.variable == "phi"):
            value = sim.phi
        elif(self.variable == "p"):
            value = sim.p
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

    def applyCommand(self,sim):
        sim.acc += self.action

    def reset(self):
        self.i = 0
        self.error = 0
        self.pre_error = 0

class ForcePIDcontroller():
    '''
        Controller will act on the Forces applied to the cart. 
    '''
    def __init__(self):
        self.variable = "phi" # or "p" (Indicates what is to be regulated)
        self.target = 0
        self.error = 0
        self.pre_error = 0
        self.P = 3.981
        self.I = 10
        self.i = 0 # Integral value
        self.D = 0.3
        self.action = 0
        self.max_val = 100
        return

    def set_PID(self,p=3.981,i=10,d=.3): # p=50,i=1000,d=10
        self.P = p
        self.I = i
        self.D = d
    def set_target(self, target=0):
        self.target = target
    def set_max_control_value(self,max_val=20):
        self.max_val = max_val

    def select_variable(self,var="phi"):
        self.variable = var

    def control(self,sim):
        # Autonomously compute the error
        t = sim.t
        if(self.variable == "phi"):
            value = sim.phi
        elif(self.variable == "p"):
            value = sim.p
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

    def applyCommand(self,sim):
        sim.F += self.action

    def reset(self):
        self.i = 0
        self.error = 0
        self.pre_error = 0

class AnglePositionPIDcontroller():
    '''
        Control phi and p as well
    '''
    def __init__(self):
        self.phiPID = PIDcontroller()
        self.phiPID.set_PID(p=219.81,i=1000,d=12) #p=149.81,i=800,d=7
        self.supPID = PIDcontroller()
        a = 100
        b = 10
        c = 1
        self.supPID.set_PID(p=a*b+a*c+b*c,i=a*b*c,d=a+b+c) # p=-2,i=0,d=-0.08 works good
        self.supPID.select_variable("p")
        self.supPID.set_target(0)
        self.K = 1 # Conversion gain coefficient (positive acc needs negative angle)
        self.action = 0

    def control(self,sim):
        target = self.supPID.control(sim)
        target *= self.K

        '''lim = 0.3
        if(target > lim):
            target = lim
        elif(target < -lim):
            target = -lim'''
        self.phiPID.target = target
        self.action = self.phiPID.control(sim)
        return self.action

    def applyCommand(self,sim):
        sim.acc += self.action

    def reset(self):
        self.phiPID.reset()
        self.supPID.reset()

class AnglePositionLQcontroller():
    '''
        Linear Quadratic Controller for angle and position control.
    '''
    def __init__(self):
        self.A = None
        self.B = None
        self.Q = None
        self.R = None

        self.K = None
        self.S = None
        self.E = None

        self.action = 0

    def build_A_matrix(self,sim):
        a22 = sim.de/(sim.Mass*sim.l) - sim.mu/sim.Mass 
        a23 = sim.m*sim.g/sim.Mass
        a24 = -sim.nu/(sim.Mass*sim.l)
        a42 = sim.de*(sim.Mass + sim.m)/(sim.Mass*sim.m*sim.l**2) - sim.mu/(sim.Mass*sim.l)
        a43 = sim.g*(sim.Mass + sim.m)/(sim.Mass*sim.l)
        a44 = -sim.nu*(sim.Mass + sim.m)/(sim.Mass*sim.m*sim.l**2)
        self.A = matrix([[0,1,0,0],[0,a22,a23,a24],[0,0,0,1],[0,a42,a43,a44]])
    def build_B_matrix(self,sim):
        b21 = 1/sim.Mass
        b22 = 1/(sim.Mass*sim.l)
        b41 = 1/(sim.Mass*sim.l)
        b42 = (sim.Mass + sim.m)/(sim.Mass*sim.m*sim.l**2)
        self.B = matrix([[0],[b21],[0],[b41]]) # Force control
        # self.B = matrix([[0,0],[b21,b22],[0,0],[b41,b42]]) # Force and Torque control
    def build_Q_matrix(self):
        self.Q = matrix("2 0 0 0; 0 2 0 0; 0 0 2 0; 0 0 0 2")
    def build_R_matrix(self):
        self.R = matrix("2.5")

    def build_K_matrix(self,sim):
        self.build_A_matrix(sim)
        self.build_B_matrix(sim)
        self.build_Q_matrix()
        self.build_R_matrix()
        self.K,self.S,self.E = lqr(self.A,self.B,self.Q,self.R)
        # K = [[-1.13137085 -1.70276008 12.50134201 1.52645134]]

    def control(self,sim):
        # Build state vector
        x = matrix([[sim.p],[sim.v],[sim.phi],[sim.phi_dot]])
        a = - self.K * x
        self.action = float(a)
        #print(self.action)
        
        return self.action 

    def applyCommand(self,sim):
        sim.F += self.action

    def reset(self):
        self.action = 0

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

    def update_display(self,sim):
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
    def __init__(self,controller,physics,controlled_entity="a",color="orange"):
        self.sim = physics()
        self.sim.controlled_entity = controlled_entity
        if(controller is not None):
            self.controller = controller()
        else:
            self.controller = None
        self.graphics = Graphics(color)
        self.timer = 0
        self.failed = False # Indicates if stabilisation failed
        self.succeeded = False # Indicates if stabilisation occured 
        self.is_done = False

        # Display the pendulum in window
        self.graphics.update_display(self.sim)

    def step_simulation(self,control_enabled=True):
        if(control_enabled and self.controller is not None):
            # Compute control commands
            #self.sim.acc = self.controller.control(self.sim)
            self.sim.update_command(self.controller.control(self.sim))
            # Update physics
            self.sim.update_physics()
        elif(not control_enabled):
            # Update physics first
            self.sim.update_command(0)
            self.sim.update_physics()
            #self.sim.acc = 0
            #self.sim.v = 0

        # Update graphics
        self.graphics.update_display(self.sim)

    def check_if_done(self):
        if(self.sim.out_of_range or self.sim.fallen):
            self.failed = True
            self.is_done = True
            self.sim.v = 0
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
        if(self.controller is not None):
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
    reset_phi = random.getrandbits(8)/255/1.0
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
        
        time.sleep(p[0].sim.t*1) # DEBUG 100

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
    p1 = Pendulum(PIDcontroller,CartPendulumPhysics) 
    #p1.sim.Mass = 0 # Remove mass from the cart
    p1.controller.set_PID(p=219.81,i=1000,d=12) # 1: N(s) = (s+100)(s+10)^2
    #N(s) = (s+1+10i)(s+1-10i)(s+100) -> p=39.91,i=1010,d=10.2
    #N(s) = s(s+100)^2 -> p=1009.81,i=0,d=20
    
    p2 = Pendulum(AnglePositionPIDcontroller,CartPendulumPhysics,color="green")
    #p2.sim.Mass = 0 # Remove mass from the cart
    p2.controller.phiPID.set_PID(p=219.81,i=1000,d=12)
    p2.controller.supPID.set_PID(p=-2,i=-1.1,d=-0.065) 
    
    p3 = Pendulum(AnglePositionLQcontroller,CartPendulumPhysics,controlled_entity="F",color="blue")
    p3.controller.build_K_matrix(p3.sim)
    #p3 = Pendulum(ForcePIDcontroller,CartPendulumPhysics,controlled_entity="F",color="blue")
    

    # Run the simulation
    run(p1,p2,p3)

    tk.mainloop()

if __name__ == "__main__":
    main()
