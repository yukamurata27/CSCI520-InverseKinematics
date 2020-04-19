CSCI 520 Computer Animation and Simulation
Homework 3 - Inverse Kinematics with Skinning

Date: April 18, 2020

* Development environment:
	macOS Catalina (version 10.15.3)

* Steps to compile the projects
1) Compile ADOL-C (please follow the instruction below)
2) Setup OpenGL (please follow the instruction below)
3) In Makefile, some modifications may be needed to link paths to load libraries (for OPENGL_LIBS)
   In my case, libGL.dylib is stored in /opt/X11/lib. So I added the library path.
   This path is still left in my Makefile.

* Run the program
1) In the project directory, execute:
	sh run.sh

* Steps to compile ADOL-C:
1) First, we need to install some necessary tools for compiling ADOL-C. Run:
	$brew install autoconf automake libtool
2) Next, enter the ADOL-C folder: <starter code folder>/adolc/sourceCode/, run command:
	$autoreconf -fi
3) To create a configure script. If no errors are reported, run:
	$./configure
4) to create a Makefile. If no errors are reported, run
	$make
5) Finally run:
	$make install
   to install ADOL-C at <your account's home folder>/adolc_base/.

* Steps to install OpenGL for this project
1) use Homebrew to install freeglut, which is an implementation of GLUT:
	$brew install freeglut

//////////////////////////////////////////////////////////////////////////////////////////////////

Core Features
* FK
* IK (Damped least squares)
* Skinning (Linear blend skinning)
* Animation frames (please see the animation folder)

Extra Features
* Recursive subdivision for IK when the IK handle moves too far (Both for DLS and PI)
  -> Please see IK.h and IK.cpp for this implementation.
* IK (Pseudo-inverse)
  -> Please see the solveIK function in IK.cpp.
     You can choose DLS or PI by setting IKSolver at line 158.
* Skinning (Dual-quaternion skinning)
  -> Please see skinning.h and skinning.cpp.
     You can set the skinning method by setting useLBS (true/false) in the applySkinning function.
* Comparison between damped least squares and pseudo-inverse (see blow)
* Comparison between linear blend skinning and dual-quaternion skinning (see below)

//////////////////////////////////////////////////////////////////////////////////////////////////

Comparison of IK solvers (damped least squares vs pseudo-inverse)
-----------------------------------------------------------------
* Damped least squares is more stable but slow as it gradually solves the IK problem.
* Pseudo-inverse can be unstable, especially when the jacobian, J, is almost singular.
* Pseudo-inverse updates the vertex positions faster (I would say it's more intuitive).
* In conclusion, Damped least squares seems like the better solution for IK because this is more stable and looks correct.

Comparison of skinning methods (linear blend vs dual-quaternion)
----------------------------------------------------------------
* Linear blend skinning is easy and simple to implement.
* Linear blend skinning causes weird twists (not smooth).
* Dual-quaternion skinning can be computationally expensive.
* Dual-quaternion skinning seems to blend rotations better.
* In conclusion,
  - Linear blend skinning is better when the change of joint location is small (small change in rotation) because it's faster.
    -> Good for games
  _ Dual-quaternion skinning is better when you need a more accurate and natural blending (but slower).
    -> Good for film production

//////////////////////////////////////////////////////////////////////////////////////////////////

Comments
--------
This was a pretty fun assignment! The result is so cool and I am happy that I now know how FK, IK, and skinning are working inside of graphics tool such as Maya. Thank you so much for grading my assignments throughout the semester. This course is indeed one of my favorite courses at USC. :)

If you face any issues especially compiling my program or modifying Makefile, please feel free to reach out to me at ymurata@usc.edu.

Thank you!




