CXX=g++
CXXFLAGS=-O3 -c -std=c++11 -Wall
LDFLAGS=-lspatialite -lsqlite3 -lwiringPi -lpthread
OBJDIR=./obj/
BINDIR=./bin/
OBJFILES=$(OBJDIR)Autopilot.o $(OBJDIR)AveragingBuffer.o $(OBJDIR)DataSource.o\
 $(OBJDIR)FlightDirector.o $(OBJDIR)GISDatabase.o $(OBJDIR)TimerSource.o\
 $(OBJDIR)Utilities.o $(OBJDIR)RpiAutopilot.o $(OBJDIR)RpiDataSource.o\
 $(OBJDIR)RpiTimerSource.o $(OBJDIR)LSM6DS33.o $(OBJDIR)LIS3MDL.o\
 $(OBJDIR)LS20031.o $(OBJDIR)NMEA.o

all: directories otto rdbtool recoverydb

directories:
	@echo Making output directories...
	@mkdir -p $(OBJDIR)
	@mkdir -p $(BINDIR)

$(OBJDIR)%.o: ../%.cpp
	@echo Compiling $<...
	@$(CXX) $(CXXFLAGS) -o $@ $<

$(OBJDIR)%.o: ./%.cpp
	@echo Compiling $<...
	@$(CXX) $(CXXFLAGS) -I../ -o $@ $<

otto: directories $(OBJFILES)
	@echo Compiling rasppi.cpp...
	@$(CXX) $(CXXFLAGS) -I../ -o $(OBJDIR)rasppi.o rasppi.cpp
	@echo Linking otto...
	@$(CXX) $(OBJFILES) $(OBJDIR)rasppi.o -o $(BINDIR)otto $(LDFLAGS)

test: directories $(OBJFILES)
	@echo Compiling test.cpp...
	@$(CXX) $(CXXFLAGS) -I../ -o $(OBJDIR)test.o test.cpp
	@echo Linking otto_test...
	@$(CXX) $(OBJFILES) $(OBJDIR)test.o -o $(BINDIR)otto_test $(LDFLAGS)

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
