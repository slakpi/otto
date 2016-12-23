ROOT_DIR:=$(shell dirname $(realpath $(lastword $(MAKEFILE_LIST))))
CC=gcc
CFLAGS=-I$(ROOT_DIR)/../ -I$(ROOT_DIR) -g -O2 -Wall
CXX=g++
CXXFLAGS=-I$(ROOT_DIR)/../ -I$(ROOT_DIR) -g -O2 -std=c++11 -Wall
LIBS=-lspatialite -lsqlite3 -lwiringPi -lpthread -lm
LDFLAGS=
OBJDIR=$(ROOT_DIR)/obj/
BINDIR=$(ROOT_DIR)/bin/
OBJFILES=$(OBJDIR)Autopilot.o $(OBJDIR)AveragingBuffer.o $(OBJDIR)DataSource.o\
 $(OBJDIR)FlightDirector.o $(OBJDIR)GISDatabase.o $(OBJDIR)Utilities.o\
 $(OBJDIR)RpiAutopilot.o $(OBJDIR)RpiDataSource.o $(OBJDIR)LSM6DS33.o\
 $(OBJDIR)LIS3MDL.o $(OBJDIR)NMEA.o $(OBJDIR)Madgwick_AHRS.o\
 $(OBJDIR)Arduino.o $(OBJDIR)HD44780.o

all: otto rdbtool

tests: test_gps test_imu test_mag test_arduino

$(OBJDIR)%.o: ./%.c
	@echo Compiling $<...
	@mkdir -p $(@D)
	@$(CC) $(CFLAGS) -c -o $@ $<

$(OBJDIR)%.o: ../%.cpp
	@echo Compiling $<...
	@mkdir -p $(@D)
	@$(CXX) $(CXXFLAGS) -c -o $@ $<

$(OBJDIR)%.o: ./%.cpp
	@echo Compiling $<...
	@mkdir -p $(@D)
	@$(CXX) $(CXXFLAGS) -c -o $@ $<

$(OBJDIR)%.o: ./tests/%.cpp
	@echo Compiling $<...
	@mkdir -p $(@D)
	@$(CXX) $(CXXFLAGS) -c -o $@ $<

$(OBJDIR)%.o: ../nav/%.cpp
	@echo Compiling $<...
	@mkdir -p $(@D)
	@$(CXX) $(CXXFLAGS) -c -o $@ $<

otto: $(OBJFILES) $(OBJDIR)rasppi.o
	@echo Linking $@...
	@mkdir -p $(BINDIR)
	@$(CXX) -o $(BINDIR)$@ $^ $(LDFLAGS) $(LIBS)

test_gps: $(OBJFILES) $(OBJDIR)test_gps.o
	@echo Linking $@...
	@mkdir -p $(BINDIR)
	@$(CXX) -o $(BINDIR)$@ $^ $(LDFLAGS) $(LIBS)

test_imu: $(OBJFILES) $(OBJDIR)test_imu.o
	@echo Linking $@...
	@mkdir -p $(BINDIR)
	@$(CXX) -o $(BINDIR)$@ $^ $(LDFLAGS) $(LIBS)

test_mag: $(OBJFILES) $(OBJDIR)test_mag.o
	@echo Linking $@...
	@mkdir -p $(BINDIR)
	@$(CXX) -o $(BINDIR)$@ $^ $(LDFLAGS) $(LIBS)

test_arduino: $(OBJFILES) $(OBJDIR)test_arduino.o
	@echo Linking $@...
	@mkdir -p $(BINDIR)
	@$(CXX) -o $(BINDIR)$@ $^ $(LDFLAGS) $(LIBS)

rdbtool: $(OBJDIR)recoverydb.o
	@echo Linking $@...
	@mkdir -p $(BINDIR)
	@$(CXX) -o $(BINDIR)$@ $^ $(LDFLAGS) $(LIBS)

recovery_db: rdbtool
	@echo Building recovery database...
	@rm -f $(BINDIR)recovery.db
	@$(BINDIR)rdbtool $(BINDIR)recovery.db ../nav/recovery.csv

clean:
	@echo Cleaning...
	@rm -rf $(BINDIR) $(OBJDIR)
