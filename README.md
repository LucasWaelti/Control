# Control Theory 
This repository is about Control Theory and aims at making experimentation simple with classical control problems such as the inverted pendulum. 

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
