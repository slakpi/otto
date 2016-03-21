CXX=g++
CXXFLAGS=-O3 -c -std=c++11 -Wall
LDFLAGS=-lspatialite -lsqlite3 -lwiringPi
OBJDIR=./obj/
BINDIR=./bin/
OBJFILES=$(OBJDIR)Autopilot.o $(OBJDIR)AveragingBuffer.o $(OBJDIR)DataSource.o\
 $(OBJDIR)FlightDirector.o $(OBJDIR)GISDatabase.o $(OBJDIR)TimerSource.o\
 $(OBJDIR)Utilities.o $(OBJDIR)RpiAutopilot.o $(OBJDIR)RpiDataSource.o\
 $(OBJDIR)RpiTimerSource.o $(OBJDIR)LSM6DS33.o $(OBJDIR)LIS3MDL.o

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

otto: $(OBJFILES)
	@echo Linking otto...
	@$(CXX) $(OBJFILES) rasppi.o -o $(BINDIR)otto $(LDFLAGS)

test: directories $(OBJFILES)
	@echo Compiling test.cpp...
	@$(CXX) $(CXXFLAGS) -I../ -o $(OBJDIR)test.o test.cpp
	@echo Linking otto_test...
	@$(CXX) $(OBJFILES) $(OBJDIR)test.o -o $(BINDIR)otto_test $(LDFLAGS)

rdbtool:
	@echo Building recoverydb tool...
	@$(CXX) $(CXXFLAGS) -o $(OBJDIR)recoverydb.o ../nav/recoverydb.cpp
	@$(CXX) $(OBJDIR)recoverydb.o -o $(BINDIR)recoverydb $(LDFLAGS)

recoverydb:
	@echo Building recovery database...
	@$(BINDIR)recoverydb $(BINDIR)recovery.db ../nav/recovery.csv

clean:
	@echo Cleaning...
	@rm -rf $(BINDIR) $(OBJDIR)
