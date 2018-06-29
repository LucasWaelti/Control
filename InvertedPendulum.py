import tkinter
import math
import time

global tk,canvas
global width,height

class Physics():
    ''' This class provides the physics for the simulation'''
    def __init__(self):
        self.m = 0.1  # kg
        self.l = 0.1  # m
        self.g = 9.81 # m/s^2
        self.phi = -0.01  # rad (small offset to get out of equilibrium)
        self.phi_dot = 0    # rad/s
        self.phi_ddot = 0   # rad/s^2
        self.fallen = False # Has the pendulum fallen?
        self.out_of_range = False # Has the support gone too far?

        self.acc = 0.1  # m/s^2 (acceleration of the support)
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
        return # Uncomment to let the pendulum swing
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
        self.phi = 0.0001  # rad (small offset to get out of equilibrium)
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
    def __init__(self):
        return

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

def setup():
    global tk,canvas,width,height
    tk = tkinter.Tk()
    width = 500
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
    return rail
def create_ball():
    global canvas
    ball = canvas.create_oval(-20,-20,20,20,fill="orange")
    return ball

def build_pendulum(rail,support,stick,ball):
    #Build the inverted pendulum
    global tk,canvas
    c = cc(0,0)
    c1 = cc(0,100)
    canvas.move(support,c[0],c[1])
    canvas.coords(stick,c[0],c[1],c1[0],c1[1])
    canvas.move(ball,c1[0],c1[1])
    tk.update()

def update_display(sim,stick,ball,support):
    global tk,canvas
    # Support position 
    x = sim.p
    y = 0
    x = mp(x)
    y = mp(y)
    sup_pos = cc(x,y)

    # Ball position in support reference (meters)
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
    setup()
    
    #Create objects (order matters)
    rail = create_rail()
    support = create_support()
    stick = create_stick()
    ball = create_ball()

    #Build the inverted pendulum
    build_pendulum(rail,support,stick,ball)
    
    # Initialize the physics
    sim = Physics()

    # Run the simulation
    while(True):
        # Update the physics
        sim.calculate_phi_ddot()
        sim.update_phi_dot()
        sim.update_phi()
        sim.update_speed()
        sim.update_position()

        update_display(sim,stick,ball,support)
        
        time.sleep(sim.t*1)

        if(sim.out_of_range or sim.fallen):
            build_pendulum(rail,support,stick,ball)
            sim.reset()

    tk.mainloop()

if __name__ == "__main__":
    main()
