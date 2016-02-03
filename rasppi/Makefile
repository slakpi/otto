CXX=g++
CXXFLAGS=-c -std=c++11 -O3 -Wall
LDFLAGS=-lspatialite -lsqlite3
OBJFILES=$(OBJDIR)Autopilot.o $(OBJDIR)AveragingBuffer.o $(OBJDIR)DataSource.o\
 $(OBJDIR)FlightDirector.o $(OBJDIR)GISDatabase.o $(OBJDIR)TimerSource.o\
 $(OBJDIR)Utilities.o $(OBJDIR)RpiAutopilot.o $(OBJDIR)RpiDataSource.o\
 $(OBJDIR)RpiTimerSource.o $(OBJDIR)rasppi.o
SRCDIR=../
OBJDIR=./obj/
BINDIR=./bin/

all: directories otto rdbtool recoverydb

directories:
	@mkdir -p $(OBJDIR)
	@mkdir -p $(BINDIR)

otto: $(OBJFILES)
	@echo Linking otto...
	@$(CXX) $(OBJFILES) -o $(BINDIR)otto $(LDFLAGS)

rdbtool:
	@echo Building recoverydb tool...
	@$(CXX) $(CXXFLAGS) -o $(OBJDIR)recoverydb.o $(SRCDIR)nav/recoverydb.cpp
	@$(CXX) $(OBJDIR)recoverydb.o -o $(BINDIR)recoverydb $(LDFLAGS)

recoverydb:
	@echo Building recovery database...
	@$(BINDIR)recoverydb $(BINDIR)recovery.db $(SRCDIR)nav/recovery.csv

$(OBJDIR)Autopilot.o:
	@echo Compiling Autopilot.cpp...
	@$(CXX) $(CXXFLAGS) -o $(OBJDIR)Autopilot.o $(SRCDIR)Autopilot.cpp

$(OBJDIR)AveragingBuffer.o:
	@echo Compiling AveragingBuffer.cpp...
	@$(CXX) $(CXXFLAGS) -o $(OBJDIR)AveragingBuffer.o $(SRCDIR)AveragingBuffer.cpp

$(OBJDIR)DataSource.o:
	@echo Compiling DataSource.cpp...
	@$(CXX) $(CXXFLAGS) -o $(OBJDIR)DataSource.o $(SRCDIR)DataSource.cpp

$(OBJDIR)FlightDirector.o:
	@echo Compiling FlightDirector.cpp...
	@$(CXX) $(CXXFLAGS) -o $(OBJDIR)FlightDirector.o $(SRCDIR)FlightDirector.cpp

$(OBJDIR)GISDatabase.o:
	@echo Compiling GISDatabase.cpp...
	@$(CXX) $(CXXFLAGS) -o $(OBJDIR)GISDatabase.o $(SRCDIR)GISDatabase.cpp

$(OBJDIR)TimerSource.o:
	@echo Compiling TimerSource.cpp...
	@$(CXX) $(CXXFLAGS) -o $(OBJDIR)TimerSource.o $(SRCDIR)TimerSource.cpp

$(OBJDIR)Utilities.o:
	@echo Compiling Utilities.cpp...
	@$(CXX) $(CXXFLAGS) -o $(OBJDIR)Utilities.o $(SRCDIR)Utilities.cpp

$(OBJDIR)RpiAutopilot.o:
	@echo Compiling RpiAutopilot.cpp...
	@$(CXX) $(CXXFLAGS) -I$(SRCDIR) -o $(OBJDIR)RpiAutopilot.o RpiAutopilot.cpp

$(OBJDIR)RpiDataSource.o:
	@echo Compiling RpiDataSource.cpp...
	@$(CXX) $(CXXFLAGS) -I$(SRCDIR) -o $(OBJDIR)RpiDataSource.o RpiDataSource.cpp

$(OBJDIR)RpiTimerSource.o:
	@echo Compiling RpiTimerSource.cpp...
	@$(CXX) $(CXXFLAGS) -I$(SRCDIR) -o $(OBJDIR)RpiTimerSource.o RpiTimerSource.cpp

$(OBJDIR)rasppi.o:
	@echo Compiling rasppi.cpp...
	@$(CXX) $(CXXFLAGS) -o $(OBJDIR)rasppi.o rasppi.cpp

clean:
	@echo Cleaning...
	@rm -f $(OBJFILES) $(BINDIR)otto $(BINDIR)recoverydb $(BINDIR)recovery.db
