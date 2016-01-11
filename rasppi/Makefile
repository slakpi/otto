CXX=g++
CXXFLAGS=-c -std=c++11 -O3 -Wall
OBJFILES=$(OUTDIR)Autopilot.o $(OUTDIR)AveragingBuffer.o $(OUTDIR)DataSource.o\
 $(OUTDIR)FlightDirector.o $(OUTDIR)TimerSource.o $(OUTDIR)Utilities.o\
 $(OUTDIR)rasppi.o
SRCDIR=../
OUTDIR=./

all: otto

otto: $(OBJFILES)
	@echo Linking otto...
	@$(CXX) $(OBJFILES) -o $(OUTDIR)otto

Autopilot.o:
	@echo Compiling Autopilot.cpp...
	@$(CXX) $(CXXFLAGS) -o $(OUTDIR)Autopilot.o $(SRCDIR)Autopilot.cpp

AveragingBuffer.o:
	@echo Compiling AveragingBuffer.cpp...
	@$(CXX) $(CXXFLAGS) -o $(OUTDIR)AveragingBuffer.o $(SRCDIR)AveragingBuffer.cpp

DataSource.o:
	@echo Compiling DataSource.cpp...
	@$(CXX) $(CXXFLAGS) -o $(OUTDIR)DataSource.o $(SRCDIR)DataSource.cpp

FlightDirector.o:
	@echo Compiling FlightDirector.cpp...
	@$(CXX) $(CXXFLAGS) -o $(OUTDIR)FlightDirector.o $(SRCDIR)FlightDirector.cpp

TimerSource.o:
	@echo Compiling TimerSource.cpp...
	@$(CXX) $(CXXFLAGS) -o $(OUTDIR)TimerSource.o $(SRCDIR)TimerSource.cpp

Utilities.o:
	@echo Compiling Utilities.cpp...
	@$(CXX) $(CXXFLAGS) -o $(OUTDIR)Utilities.o $(SRCDIR)Utilities.cpp

rasppi.o:
	@echo Compiling rasppi.cpp...
	@$(CXX) $(CXXFLAGS) -o $(OUTDIR)rasppi.o rasppi.cpp

clean:
	@echo Cleaning...
	@rm -f $(OBJFILES) $(OUTDIR)otto
