# Inverse Kinematics and Skinning (C++)
IK and skinning tool as part of USC CSCI 520 in April 2020.


## Features
#### 1) Skinning
- Linear Blend Skinning
p = SUM( w * M * p_rest )

#### 2) Forward Kinematics (FK)
Mg_child = Mg_parent * Ml_child 
where g is global and l is local.

#### 3) Inverse Kinematics (IK)
- Damped Least Squares
min_theta ( 1/2 * | J*dTheta - db |^2 + 1/2 * alpha * | dTheta |^2 )
where J is a Jacobian matrix.


## Extra Features
#### 1) Dual-Quaternion Skinning
#### 2) Pseudo-Inverse IK
#### 3) Recursive Subdivision for IK solvers
- Damped Least Squares
- Pseudo-Inverse


## Environment
Implemented on macOS Catalina (version 10.15.3)


## Dependencies
Please download XQuartz from [here](ttps://www.xquartz.org/)


## Libraries
- FreeGLUT
- OpenGL
- Eigen
- ADOL-C

## Building
#### 1) Open Terminal and go to the root directory.

#### 2) Compile ADOL-C:
- A) Install necessary tools to compile ADOL-C:
```
brew install autoconf automake libtool
```
- B) Go to the ADOL-C folder:
```
cd adolc/sourceCode/
```
- C) Run:
```
autoreconf -fi
```
- D) Create a configure script:
```
./configure
```
- E) Create a Makefile:
```
make
```
- F) Finally run:
```
make install
```

#### 3) Setup OpenGL:
- A) Install freeglut:
```
brew install freeglut
```

#### 4) Finally, Compile the program in the root directory:
```
cd ../..
make
```


## Usage
#### 1) In the root directory, execute:
```
sh run.sh
```
#### 2) Change model
- You can change the model in run.sh.
- Uncomment the model name at the top of the file.

#### 3) Interactions:
- ESC: exit program
- Left mouse button: display IK handle
- Drag left mouse button on IK handle: move IK handle
- Drag middle mouse button: camera zoom in/out
- Drag right mouse button: Camera control


## Demo
![Demo](readme_content/demo.gif)


## Course Links
1) USC Viterbi School of Engineering [CSCI 520](http://barbic.usc.edu/cs520-s20/)
2) [Assignment 3](http://barbic.usc.edu/cs520-s20/assign3/)

