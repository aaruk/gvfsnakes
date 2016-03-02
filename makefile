# Include Directory
IDIR = ./include
ODIR = build
SRC_DIR = src

# Compile and Link Flags
CC = g++
CFLAGS = -I$(IDIR)

# Compilation: Add Flags as needed
CXXFLAGS += `pkg-config --cflags opencv`
CXXFLAGS += -std=c++11

# Linking: Add Flags as needed
LIBS += `pkg-config --libs opencv`

_OBJ = main.o snakes.o mathOps.o imutils.o
OBJ = $(patsubst %, $(ODIR)/%, $(_OBJ))

#_DEPS = disp.h file_ops.h
#DEPS = $(patsubst %, $(IDIR)/%, $(_DEPS))

# Name Executables here
all	= $(ODIR)/snake

# Target Executable Dependencies
all:$(all)

debug: CXXFLAGS += -DNDEBUG -ggdb3 -DDEBUG -Wall -Wextra -std=c++11 -O0 -g -Wno-reorder
debug: $(all)

release: CXXFLAGS += -O3 -std=c++11
release: $(all)

$(all): $(OBJ)
	$(CC) -o $@ $^ $(CXXFLAGS) $(LIBS) $(CFLAGS)

# Target Object File Dependencies:
# Main Object, Draw Object depend on multiple headers
 $(ODIR)/main.o: $(SRC_DIR)/main.cpp  #$(DEPS) 
	$(CC) -c -o $@ $(SRC_DIR)/main.cpp $(CXXFLAGS) $(CFLAGS) 

# Make File for other objects
 $(ODIR)/%.o: $(SRC_DIR)/%.cpp $(IDIR)/%.hpp #$(DEPS)
	$(CC) -c -o $@ $< $(CXXFLAGS) $(CFLAGS)

.PHONY: clean

clean:
	rm -f $(ODIR)/*.o ./*~ $(INCDIR)/*~ $(ODIR)/GaugeView $(ODIR)/a.out $(ODIR)/gmon.out
