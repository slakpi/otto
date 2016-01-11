CXX=g++
CXXFLAGS=-c -std=c++11 -O3 -Wall
OBJFILES=$(OBJDIR)Autopilot.o $(OBJDIR)AveragingBuffer.o $(OBJDIR)DataSource.o\
 $(OBJDIR)FlightDirector.o $(OBJDIR)TimerSource.o $(OBJDIR)Utilities.o\
 $(OBJDIR)rasppi.o
SRCDIR=../
OBJDIR=./obj/
BINDIR=./bin/

all: directories otto

directories:
	@mkdir -p $(OBJDIR)
	@mkdir -p $(BINDIR)

otto: $(OBJFILES)
	@echo Linking otto...
	@$(CXX) $(OBJFILES) -o $(BINDIR)otto

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

$(OBJDIR)TimerSource.o:
	@echo Compiling TimerSource.cpp...
	@$(CXX) $(CXXFLAGS) -o $(OBJDIR)TimerSource.o $(SRCDIR)TimerSource.cpp

$(OBJDIR)Utilities.o:
	@echo Compiling Utilities.cpp...
	@$(CXX) $(CXXFLAGS) -o $(OBJDIR)Utilities.o $(SRCDIR)Utilities.cpp

$(OBJDIR)rasppi.o:
	@echo Compiling rasppi.cpp...
	@$(CXX) $(CXXFLAGS) -o $(OBJDIR)rasppi.o rasppi.cpp

clean:
	@echo Cleaning...
	@rm -f $(OBJFILES) $(BINDIR)otto
