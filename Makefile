########################################
##
## Makefile
## LINUX compilation
##
##############################################


#FLAGS
C++FLAG = -g -std=c++11 -Wall $(shell pkg-config --cflags pcl_common-1.8 eigen3)
MATH_LIBS = -lm

EXEC_DIR=.
LDFLAGS = $(shell pkg-config --libs pcl_common-1.8 eigen3) -lpcl_io -lpcl_octree -lpcl_kdtree -lflann_cpp --verbose 


.cpp.o:
	g++ $(C++FLAG) $(INCLUDES) -c $< -o $@

#Including
INCLUDES=  -I.

#-->All libraries (without LEDA)
LIBS_ALL =  -L/usr/lib -L/usr/local/lib


Cpp_OBJ1= main.cpp Pcl_reg.o
PROGRAM_1=p1
$(PROGRAM_1): $(Cpp_OBJ1)
	g++ $(C++FLAG) -o $(EXEC_DIR)/$@ $(Cpp_OBJ1) $(INCLUDES) $(LIBS_ALL) $(LDFLAGS)


all:
	make $(PROGRAM_1)


clean:
	(rm -f *.o;)

(:
