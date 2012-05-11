#############################################################################
# Makefile for building: libicr.a
# Robert Krug 2011-08-10
#############################################################################

####### Compiler, tools and options

CC       = gcc
CXX      = g++
CFLAGS   = -pipe -Wall -W -O2 -fPIC 
CXXFLAGS = -pipe -Werror -Wall -Wcast-qual -W  -Wextra -Wwrite-strings -Wno-sign-conversion -O3 -g -fPIC -msse2 -std=gnu++0x 
AR       = ar cqs 
DEL_FILE = rm -f

####### Output directory

ODIR = obj

####### Include directories for library headers

INCPATH  = -I ./tools/eigen-eigen-3.0.2/ #eigen headers
INCPATH += -I /usr/local/include/libqhullcpp  #qhull headers
INCPATH += -I /usr/local/include #libobj headers

####### Files for compiling the code

ICR_HEADERS = 	./include/utilities.h \
		./include/contact_point.h \
		./include/target_object.h \
                ./include/object_loader.h \
		./include/wrench_cone.h \
		./include/limit_surface.h \
		./include/ows.h \
		./include/contact_model.h \
		./include/grasp.h \
		./include/finger.h \
		./include/wrench_space.h \
		./include/search_zones.h \
		./include/independent_contact_regions.h 

ICR_SOURCES = 	./src/utilities.cpp \
		./src/contact_point.cpp \
		./src/target_object.cpp \
		./src/object_loader.cpp	\
		./src/wrench_cone.cpp \
		./src/limit_surface.cpp	\
                ./src/ows.cpp \
		./src/contact_model.cpp \
		./src/grasp.cpp \
		./src/finger.cpp \
		./src/wrench_space.cpp \
		./src/search_zones.cpp \
		./src/independent_contact_regions.cpp 

ICR_OBJECTS =	./obj/utilities.o \
		./obj/contact_point.o \
		./obj/target_object.o \
		./obj/object_loader.o \
		./obj/wrench_cone.o \
		./obj/limit_surface.o \
		./obj/ows.o \
		./obj/contact_model.o \
		./obj/grasp.o \
		./obj/finger.o \
		./obj/wrench_space.o \
		./obj/search_zones.o \
		./obj/independent_contact_regions.o 

####### Implicit rules

.SUFFIXES: .c .o .cpp .cc .cxx .C

.cpp.o:
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o $@ $<

.cc.o:
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o $@ $<

.cxx.o:
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o $@ $<

.C.o:
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o $@ $<

.c.o:
	$(CC) -c $(CFLAGS) $(INCPATH) -o $@ $<

LIBICR=./lib/libicr.a

####### Build rules for libicr.a

all: $(ICR_OBJECTS) 
	$(AR) $(LIBICR) $(ICR_OBJECTS) 

####### Clean up	

clean:
	-$(DEL_FILE) $(ICR_OBJECTS) $(LIBICR) 
	-$(DEL_FILE) *~ core *.core

####### Compile the source code

$(ODIR)/utilities.o: ./src/utilities.cpp ./include/utilities.h 
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o $(ODIR)/utilities.o ./src/utilities.cpp

$(ODIR)/contact_point.o: ./src/contact_point.cpp ./include/contact_point.h 
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o $(ODIR)/contact_point.o ./src/contact_point.cpp


$(ODIR)/target_object.o: ./src/target_object.cpp ./include/target_object.h 
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o $(ODIR)/target_object.o ./src/target_object.cpp

$(ODIR)/object_loader.o: ./src/object_loader.cpp ./include/object_loader.h 
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o $(ODIR)/object_loader.o ./src/object_loader.cpp 

$(ODIR)/wrench_cone.o: ./src/wrench_cone.cpp ./include/wrench_cone.h 
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o $(ODIR)/wrench_cone.o ./src/wrench_cone.cpp

$(ODIR)/limit_surface.o: ./src/limit_surface.cpp ./include/limit_surface.h 
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o $(ODIR)/limit_surface.o ./src/limit_surface.cpp

$(ODIR)/ows.o: ./src/ows.cpp ./include/ows.h 
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o $(ODIR)/ows.o ./src/ows.cpp

$(ODIR)/contact_model.o: ./src/contact_model.cpp ./include/contact_model.h 
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o $(ODIR)/contact_model.o ./src/contact_model.cpp

$(ODIR)/grasp.o: ./src/grasp.cpp ./include/grasp.h 
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o $(ODIR)/grasp.o ./src/grasp.cpp

$(ODIR)/finger.o: ./src/finger.cpp ./include/finger.h 
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o $(ODIR)/finger.o ./src/finger.cpp

$(ODIR)/wrench_space.o: ./src/wrench_space.cpp ./include/wrench_space.h 
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o $(ODIR)/wrench_space.o ./src/wrench_space.cpp

$(ODIR)/search_zones.o: ./src/search_zones.cpp ./include/search_zones.h 
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o $(ODIR)/search_zones.o ./src/search_zones.cpp

$(ODIR)/independent_contact_regions.o: ./src/independent_contact_regions.cpp ./include/independent_contact_regions.h 
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o $(ODIR)/independent_contact_regions.o ./src/independent_contact_regions.cpp


