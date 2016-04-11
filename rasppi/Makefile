CC=gcc
CCFLAGS=-O3 -c -Wall -Wno-strict-aliasing
CXX=g++
CXXFLAGS=-O3 -c -std=c++11 -Wall
LDFLAGS=-lspatialite -lsqlite3 -lwiringPi -lpthread
OBJDIR=./obj/
BINDIR=./bin/
OBJFILES=$(OBJDIR)Autopilot.o $(OBJDIR)AveragingBuffer.o $(OBJDIR)DataSource.o\
 $(OBJDIR)FlightDirector.o $(OBJDIR)GISDatabase.o $(OBJDIR)Utilities.o\
 $(OBJDIR)RpiAutopilot.o $(OBJDIR)RpiDataSource.o $(OBJDIR)LSM6DS33.o\
 $(OBJDIR)LIS3MDL.o $(OBJDIR)LS20031.o $(OBJDIR)NMEA.o $(OBJDIR)Madgwick_AHRS.o
EXTOBJFILES=$(OBJDIR)HD44780.o

all: otto tests rdbtool recoverydb

tests: test_imu test_gps test_clr

directories:
	@echo Making output directories...
	@mkdir -p $(OBJDIR)
	@mkdir -p $(BINDIR)

$(OBJDIR)%.o: ../%.c
	@echo Compiling $<...
	@$(CC) $(CCFLAGS) -o $@ $<

$(OBJDIR)%.o: ./%.c
	@echo Compiling $<...
	@$(CC) $(CCFLAGS) -I../ -o $@ $<

$(OBJDIR)%.o: ../%.cpp
	@echo Compiling $<...
	@$(CXX) $(CXXFLAGS) -o $@ $<

$(OBJDIR)%.o: ./%.cpp
	@echo Compiling $<...
	@$(CXX) $(CXXFLAGS) -I../ -o $@ $<

otto: directories $(OBJFILES) $(OBJDIR)rasppi.o
	@echo Linking otto...
	@$(CXX) $(OBJFILES) $(OBJDIR)rasppi.o -o $(BINDIR)otto $(LDFLAGS)

test_gps: directories $(OBJFILES) $(EXTOBJFILES) $(OBJDIR)test_gps.o
	@echo Linking test_gps...
	@$(CXX) $(OBJFILES) $(OBJDIR)HD44780.o $(OBJDIR)test_gps.o -o $(BINDIR)test_gps $(LDFLAGS)

test_imu: directories $(OBJFILES) $(EXTOBJFILES) $(OBJDIR)test_imu.o
	@echo Linking test_imu...
	@$(CXX) $(OBJFILES) $(OBJDIR)HD44780.o $(OBJDIR)test_imu.o -o $(BINDIR)test_imu $(LDFLAGS)

test_clr: directories $(OBJFILES) $(EXTOBJFILES) $(OBJDIR)test_clr.o
	@echo Linking test_clr...
	@$(CXX) $(OBJFILES) $(OBJDIR)HD44780.o $(OBJDIR)test_clr.o -o $(BINDIR)test_clr $(LDFLAGS)

rdbtool: directories
	@echo Building recoverydb tool...
	@$(CXX) $(CXXFLAGS) -o $(OBJDIR)recoverydb.o ../nav/recoverydb.cpp
	@$(CXX) $(OBJDIR)recoverydb.o -o $(BINDIR)recoverydb $(LDFLAGS)

recoverydb: directories rdbtool
	@echo Building recovery database...
	@$(BINDIR)recoverydb $(BINDIR)recovery.db ../nav/recovery.csv

clean:
	@echo Cleaning...
	@rm -rf $(BINDIR) $(OBJDIR)
