include ./Makefile.defs

CPPFLAGS = $(OPTIM) -D_LARGEFILE_SOURCE -D_LARGEFILE64_SOURCE -D_THREAD_SAFE -D_REENTRANT   
# CPPFLAGS_DEBUG = $(OPTIM) -D_LARGEFILE_SOURCE -D_LARGEFILE64_SOURCE -D_THREAD_SAFE -D_REENTRANT   -arch i386 -g
CPPFLAGS_DEBUG = $(OPTIM) -D_LARGEFILE_SOURCE -D_LARGEFILE64_SOURCE -D_THREAD_SAFE -D_REENTRANT -g

OPENGL          = /System/Library/Frameworks/OpenGL.framework/Versions/A/Headers
GLDLIBS         =  -framework AGL -framework OpenGL -lpthread -framework GLUT -framework Carbon -framework Cocoa -framework ApplicationServices

# LDFLAGS		= $(OPTIM)  -arch i386
LDFLAGS		= $(OPTIM) 

POSTBUILD       = /Developer/Tools/Rez -t APPL -o

DEBUG_DIR = ./debug
RELEASE_DIR = ./release

BIN = rtik
BIN_DEBUG = rtik_debug

MAIN_OBJS = \
	C3dFileInfo.o \
	Dof.o \
	Main.o \
	Marker.o \
	TransformNode.o \
	$(NULL)

BODY_OBJS = \
	ArticulatedBody.o \
	$(NULL)

FLTK_OBJS = \
	GLPrimitives.o \
	PhylterGLWindow.o \
	RealTimeIKui.o \
	$(NULL)

PARSER_OBJS = \
	ModelParser.o \
	skel_scanner.o \
	$(NULL)

TRANSFORM_OBJS = \
	RotateEuler.o \
	Scale.o \
	Translate.o \
	$(NULL)

UTIL_OBJS = \
	Command.o \
	Trackball.o \
	UICallback.o \
        My_Math_Lib.o \
	$(NULL)

FUNCTION_OBJS = \
	$(NULL)

OBJS = $(MAIN_OBJS) $(CONTROLLER_OBJS) $(FLTK_OBJS) $(PARSER_OBJS) $(TRANSFORM_OBJS) $(ABST_DOF_OBS) $(UTIL_OBJS) $(FUNCTION_OBJS) $(BODY_OBJS)

IDIRS = -I../../ticpp/ -I../../vl/include -I/usr/local/include -I/usr/include/malloc -I$(OPENGL) -I./fltk-1.3.0 -I./vl-1.3.2/include
LDIRS = -L/usr/X11R6/lib  -L./fltk-1.3.0/lib -L./vl-1.3.2/lib

# LIBS = /usr/local/lib/libfltk.a /usr/local/lib/libfltk_gl.a /usr/local/lib/libfltk_images.a -lXmu ../../vl/vld.a
LIBS = -lfltk -lfltk_gl -lfltk_images -lXmu -lvl.dbg

.SUFFIXES: .o .cpp .cxx

.cpp.o: 
	$(CPP) $(CPPFLAGS) $(IDIRS) -c -o $*.o $<

.cxx.o: 
	$(CPP) $(CPPFLAGS) $(IDIRS) -c -o $*.o $<


DEBUG_OBJS := $(foreach f,$(OBJS),$(DEBUG_DIR)/$(patsubst %.cpp,%.o,$(f)))
RELEASE_OBJS := $(foreach f,$(OBJS),$(RELEASE_DIR)/$(patsubst %.cpp,%.o,$(f)))

all: $(BIN)

$(BIN) : $(RELEASE_OBJS)
	$(CPP) $(LDFLAGS) $(RELEASE_OBJS) -o $@ $(LDIRS) $(LIBS) $(GLDLIBS) 

debug: $(BIN_DEBUG)

$(BIN_DEBUG) : $(DEBUG_OBJS)
	$(CPP) $(LDFLAGS) $(DEBUG_OBJS) -o $@  $(LDIRS) $(LIBS) $(GLDLIBS) 


$(RELEASE_DIR)/%.o: %.cpp
	@echo $<
	@$(CPP) $(CPPFLAGS) $(IDIRS) -c $< -o $@

$(DEBUG_DIR)/%.o: %.cpp
	@echo $<
	@$(CPP) $(CPPFLAGS_DEBUG) $(IDIRS) -c $< -o $@



clean:
	rm -f $(BIN) $(RELEASE_OBJS) $(BIN_DEBUG) $(DEBUG_OBJS)

clean_debug:
	rm -f $(BIN_DEBUG) $(DEBUG_OBJS)

clean_release:
	rm -f $(BIN) $(RELEASE_OBJS)
