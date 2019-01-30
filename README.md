# Control Theory 
This repository is about Control Theory and aims at making experimentation simple with classical control problems such as the inverted pendulum. 

# Matrix and LQR Implementation in C++
The class Matrix ([`Matrix.cpp/hpp`](https://github.com/LucasWaelti/Control/blob/master/MatrixImplementationC%2B%2B/matrix.cpp)) enables the user to work with matrices by making available tools such as matrix inversion, transpose, concatenation, matrices addition, subtraction and multiplication, rank computation. The class is internally based on the `std::vector` class. 

The LQR functions ([`lqr.cpp/hpp`](https://github.com/LucasWaelti/Control/blob/master/MatrixImplementationC%2B%2B/lqr.cpp)) allow to compute the feedback gain for a controller and an observer. They rely on the Matrix class to run the computations. You can make use of these functions as follows:
```C++
// Gain K for a state feedback controller
Matrix K = LQR::lqr(Phi,Gamma,Q1,Q2);

// Gain L for an observer
Matrix L = LQR::lqr_observer(Phi,C,Q1,Q2);
// or
Matrix Ltrans = LQR::lqr(Phi.trans(),C.trans(),Q1,Q2);
Matrix L = Ltrans.trans();
```
Methods to compute the controllability and observability of a discrete system are also available:
```C++
Matrix G = LQR::controllability(Phi,Gamma);
Matrix Q = LQR::observability(Phi,C);
```

# Multiple Pendulum Simulation in Python
The file [MultipleInvertedPendulum.py](https://github.com/LucasWaelti/Control/blob/master/MultipleInvertedPendulum.py) contains the simulation of three pendulums with identical physical characteristics but with different controllers. 

- Orange: a PID controls the acceleration of the cart. The closed loop has 3 poles located in -100, -10 and -10. The controller is very stable and relatively efficient. 
- Green: the same controller is used than in the first case but is controlled by a second PID controller that set a target angle to direct the cart towards a target postion (center of the window). The approach is naive and pretty unstable. 
- Blue: this is the best controller. It is actually a Linear Quadratic Controller (LQ) controlling the pendulum angle and cart positon in an optimal way. 

## Import the module "control"
The module "control" requires the module "slycot", which is a wrapper to SLICOT, a Fortran written software, allowing the use of MIMO system. 

A first option to install would be: 
```
>>> pip install slycot
>>> pip install control
```
Slycot will need a Fortran compiler, if none are present on your machine, the installation will fail. You can either install a compiler and rerun the installation once everything is set up, or you can **avoid compiling anything** by doing as follows: 

### Download *all* DLLs and Libs from there:
http://icl.cs.utk.edu/lapack-for-windows/libraries/VisualStudio/3.4.1/Dynamic-MINGW/Win32/
and place them in the correct directories:
- C:\Users\\"UserName"\AppData\Local\Programs\Python\Python36-32\libs
- C:\Users\\"UserName"\AppData\Local\Programs\Python\Python36-32\DLLs

### Download the Wheel file that corresponds to your system
Download it from here: https://www.lfd.uci.edu/~gohlke/pythonlibs/#slycot.
For me, the correct version was `slycot-0.3.3-cp36-cp36m-win32.whl` according to the command :
```
>>> Python
Python 3.6.3 [...][... 32 bit ...] on win32
```
Then run the command after placing the wheel in an appropriate folder, for example:
```
>>> pip install "C:\Users\YourUserName\AppData\Local\Programs\Python\Python36-32\slycot-0.3.3-cp36-cp36m-win32.whl"
```
Then slycot is installed!
> Note that you might need to run again `pip install control` as slycot might need to be installed first. 
