CSCI 520 Computer Animation and Simulation
Homework 3 - Inverse Kinematics with Skinning

Date: April 15, 2020

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

Comments
* I completed the core part of the projects.
* I implemented recursive subdivision when the IK handle moves too far.
  -> Please see IK.h and IK.cpp for this implementation.
