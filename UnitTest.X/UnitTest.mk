#
# File: MCHP_dsPIC.tmf
#
#--------------------------------------------------------------
#   Embedded Coder for Microchip dsPIC30/dsPIC33/PIC24 family. |
#   Generate .c and .h files from your Matlab/simulink model   |
#   and compile the diagram to .hex / .elf or .coff file that  |
#   can be downloaded directly into the microcontroller        |
#                                                              |
#      The Microchip name PIC, dsPIC, and MPLAB are            |
#      registered trademarks of Microchip Technology Inc.      |
#      MATLAB, Simulink, and Real-Time Workshop are            |
#      registered trademarks of The MathWorks, Inc.            |
#                                                              |
#--------------------------------------------------------------
#------------------------ Macros read by make_rtw ------------------------------
MAKECMD 			 = "C:/Program Files/MATLAB/R2011b/bin/win64/gmake"
HOST             = ANY
BUILD            = yes
BUILD_SUCCESS    = *** Created
SYS_TARGET_FILE  = MCHP_dsPIC_stf.tlc
MAKEFILE_FILESEP = /
#---------------------- Tokens expanded by make_rtw ----------------------------
#
# The following tokens, when wrapped with "|>" and "<|" are expanded by the 
# Real-Time Workshop build procedure.
#   
MODEL                   = UnitTest
MODULES                 = MCHP_I2C2_Interrupt.c MCHP_I2C2_Interrupt_data.c MCHP_SPI1_Interrupt.c MCHP_SPI1_Interrupt_data.c MCHP_UART1_Interrupt.c MCHP_UART4_Interrupt.c MavlinkComm.c UnitTest_main.c adisCube16405.c circBuffer.c gpsPort.c gpsUblox.c hil.c mavlinkSensorMcu.c novatel.c updateSensorMcuState.c 
MAKEFILE                = UnitTest.mk
MATLAB_ROOT             = C:/Program Files/MATLAB/R2011b
ALT_MATLAB_ROOT         = C:/PROGRA~1/MATLAB/R2011b
MASTER_ANCHOR_DIR       = 
START_DIR               = C:/Users/sharon/Documents/GitHub/AUAV_V3_TestMavLink
S_FUNCTIONS             = 
S_FUNCTIONS_LIB         = 
NUMST                   = 2
NCSTATES                = 0
COMPUTER                = PCWIN64
BUILDARGS               =  GENERATE_REPORT=1 GENERATE_ASAP2=0 HOST_PLATFORM="PC" MCHP_AUTO_FLASH=0 MCHP_MULTITHREAD_COMPILATION=1
MULTITASKING            = 1
INTEGER_CODE            = 0
MAT_FILE                = 0
ONESTEPFCN              = 1
TERMFCN                 = 0
B_ERTSFCN               = 0
MEXEXT                  = mexw64
EXT_MODE                = 0
TMW_EXTMODE_TESTING     = 0
EXTMODE_TRANSPORT       = 0
EXTMODE_STATIC          = 0
EXTMODE_STATIC_SIZE     = 1000000
MULTI_INSTANCE_CODE     = 0
TGT_FCN_LIB             = C89/C90 (ANSI)
MODELREFS               = 
SHARED_SRC              = 
SHARED_SRC_DIR          = 
SHARED_BIN_DIR          = 
SHARED_LIB              = 
GEN_SAMPLE_MAIN         = 0
TARGET_LANG_EXT         = c
PORTABLE_WORDSIZES      = 0
SHRLIBTARGET            = 0
OPTIMIZATION_FLAGS      = 
ADDITIONAL_LDFLAGS      = 
MAKEFILEBUILDER_TGT     = 0
STANDALONE_SUPPRESS_EXE = 0

#--------------------------- Model and reference models -----------------------
MODELLIB                  = UnitTestlib_lib.a
MODELREF_LINK_LIBS        = 
MODELREF_INC_PATH         = 
# warning: .. is different from MAtlab 2013a version (pointing to a folder beyong)
RELATIVE_PATH_TO_ANCHOR   = ..
MODELREF_TARGET_TYPE       = NONE



#-- In the case when directory name contains space ---
ifneq ($(MATLAB_ROOT),$(ALT_MATLAB_ROOT))
MATLAB_ROOT := $(ALT_MATLAB_ROOT)
endif


#-----------------------------  tool specification settings -----------------

AssemblyListing = 0
ProduceHexOutput = 0

GCC1    = xc16-gcc.exe
AR1     = xc16-ar
BIN2HEX = xc16-bin2hex 	
OBJDUMP = xc16-objdump

GCCPATH	= C:\PROGRA~2\MICROC~1\xc16\v1.26\bin
ifeq ($(strip $(GCCPATH)),)
	CC   = $(GCC1)  
	CPP  = $(GCC1) 
	LD   = $(GCC1) 
	AR   = $(AR1) -r -omf=$(OMF)
else
	AR   = $(GCCPATH)/(AR1) 
	CC   = "$(GCCPATH)/$(GCC1)"
	CPP  = "$(GCCPATH)/$(GCC1)"
	LD   = "$(GCCPATH)/$(GCC1)"
	AR   = "$(GCCPATH)/$(AR1)" -r -omf=$(OMF)
endif

ifeq ($(HOST),PC)
DEL = del /Q
else
DEL = rm -rf
endif

#------------------------------ Others dsPIC blockset parameters --------------
EXTMODE_DATA_UPLOAD = 0
OMF   	        = elf

OPTIM_GCC 		=  -O3 -mlarge-data -mlarge-scalar -fschedule-insns -fschedule-insns2
LDFLAGS 		= -t,--report-mem,-Map=info.map,-cref
LINKER_SCRIPT 	= "C:\PROGRA~2\MICROC~1\xc16\v1.26\support\dsPIC33E\gld\P35C90~1.GLD"
PIC_LIB 		= C:\PROGRA~2\MICROC~1\xc16\v1.26\lib\dsPIC33E\libp33EP512MU810-elf.a C:\PROGRA~2\MICROC~1\xc16\v1.26\lib\libpic30-elf.a C:\PROGRA~2\MICROC~1\xc16\v1.26\lib\libm-elf.a C:\PROGRA~2\MICROC~1\xc16\v1.26\lib\libc-elf.a C:\PROGRA~2\MICROC~1\xc16\v1.26\lib\libq-elf.a C:\PROGRA~2\MICROC~1\xc16\v1.26\lib\libq-dsp-elf.a
PIC_INCLUDES 	= -IC:\PROGRA~2\MICROC~1\xc16\v1.26\include -IC:\PROGRA~2\MICROC~1\xc16\v1.26\support\dsPIC33E\h -IC:\PROGRA~2\MICROC~1\xc16\v1.26\support\generic\h -IC:\PROGRA~2\MICROC~1\xc16\v1.26\support\PERIPH~2
PIC_REF 		= 33EP512MU810

#-- always add one space just before -Xlinker solve possible ambiguity while using -Wl flag. Solve problem when LDFLAGS starts with -Xlinker (old style)
LDFLAGS := $(subst -Xlinker, -Xlinker,$(LDFLAGS))


MCPU  			   = -mcpu=$(PIC_REF)
PROGRAM_FILE_EXT   = .$(subst coff,cof,$(OMF))

T_LINKER_SCRIPT = -omf=$(OMF) -Wl,--script=$(LINKER_SCRIPT)
												
LDLIBPIC = $(PIC_LIB)


CC_OPTS 		= $(MCPU) -omf=$(OMF) $(OPTIM_GCC) $(OPTS)

ifeq ($(INTEGER_CODE),1)
	CC_OPTS 		+= -DINTEGER_CODE=1
endif

OBJ_EXT =.o
CCOUTPUTFLAG = -o

#----------------------------- External mode -----------------------------------
# added OPTS: from make file call, define MemUnit_T=uint8_T and MEM_UNIT_BYTES=, usefull for PIL or external mode
# EXTMODE_DISABLESIGNALMONITORING EXTMODE_DISABLE_ARGS_PROCESSING  MULTITASKING
# -DEXTMODE_DISABLESIGNALMONITORING=1

ifeq ($(EXT_MODE),1)
	#---Compiler and Linker settings---
	ifeq ($(MULTITASKING),1)
		LDFLAGS += -Wl,--wrap,_SetParam
	endif
	
	CC_OPTS += -DEXT_MODE=1 -DMODEL=$(MODEL) -DNUMST=$(NUMST)  -DEXTMODE_DISABLEPRINTF=1 -DEXTMODE_DISABLE_ARGS_PROCESSING=1
	ifeq ($(EXTMODE_DATA_UPLOAD),0)
		CC_OPTS 		+= -DEXTMODE_DISABLESIGNALMONITORING=1
	endif

	#---required files---

	#external mode sources
	EXT_SRC  = ext_svr.c updown.c ext_work.c rtiostream_interface.c
  
	# static memory management (optional)
	ifeq ($(EXTMODE_STATIC),1) 
	  EXT_SRC += mem_mgr.c
	  CC_OPTS +=  -DEXTMODE_STATIC=1 -DEXTMODE_STATIC_SIZE=$(EXTMODE_STATIC_SIZE)
	endif

endif

#------------------------------ parameter controlled from simulink ----------------



# Keep following line, help the Matlab programming tool to find the programmer. 												
# Additional info : PICREF{33EP512MU810} PROGRAMMER{PICkit3 S.No : BUR102554963}
												
#------------------------------ Include Path -----------------------------------
#
MATLAB_INCLUDES = \
	-I$(MATLAB_ROOT)/rtw/c/ert \
	-I$(MATLAB_ROOT)/extern/include \
	-I$(MATLAB_ROOT)/simulink/include \
	-I$(MATLAB_ROOT)/rtw/c/src \
	-I$(MATLAB_ROOT)/rtw/c/src/ext_mode/common

# Additional includes 
#
ADD_INCLUDES = \
-I$(START_DIR)/UnitTest.X \
-I$(START_DIR) \
-I$(START_DIR)/mavlink/include/slugs \
-I$(START_DIR)/clib \



SHARED_INCLUDES =
ifneq ($(SHARED_SRC_DIR),)
SHARED_INCLUDES = $(SHARED_SRC_DIR) 
endif

INCLUDES = $(RELATIVE_PATH_TO_ANCHOR) $(MATLAB_INCLUDES) $(ADD_INCLUDES) \
           $(PIC_INCLUDES) $(MODELREF_INC_PATH) \
           $(SHARED_INCLUDES)

#-------------------------------- C Flags --------------------------------------
# General User Options 

# New : to support Continuous States ? find in "Upgrading from V3.2.1 or V3.2"
CPP_REQ_DEFINES = -DMODEL=$(MODEL) -DNUMST=$(NUMST) -DNCSTATES=$(NCSTATES) \
		  -DMAT_FILE=$(MAT_FILE) -DINTEGER_CODE=$(INTEGER_CODE) \
		  -DONESTEPFCN=$(ONESTEPFCN) -DTERMFCN=$(TERMFCN) \
		  -DHAVESTDIO -DMULTI_INSTANCE_CODE=$(MULTI_INSTANCE_CODE) \
		  -DCLASSIC_INTERFACE=$(CLASSIC_INTERFACE)\
		  -DADD_MDL_NAME_TO_GLOBALS=$(ADD_MDL_NAME_TO_GLOBALS)

CFLAGS    =$(CC_OPTS)  $(INCLUDES) 
CPPFLAGS = $(CC_OPTS)  $(INCLUDES)

LIBS =










#-------------------------------- Target application --------------------------------------
# Define the target file

ifeq ($(MODELREF_TARGET_TYPE), NONE)
ifeq ($(MAKEFILEBUILDER_TGT), 0)
ifeq ($(STANDALONE_SUPPRESS_EXE), 1)
# Build object code only for top level model (preliminary step for building
# PIL application)
BUILD_MODE=pil_prebuild
PRODUCT=ObjectModules
BUILD_PRODUCT_TYPE = object modules
else
	# Building executable for real-time deployment
	BUILD_MODE=real_time
	# remove MODELLIB from linker
	# MODELLIB =	
	PRODUCT = $(MODEL)$(PROGRAM_FILE_EXT)
	BUILD_PRODUCT_TYPE = executable
endif
else
# Building executable for deployment as PIL application (non-real-time 
# simulation)
BUILD_MODE=pil_application
PRODUCT = $(MODEL)$(PROGRAM_FILE_EXT)
BUILD_PRODUCT_TYPE = executable
endif
else
# Building a library for a referenced model
BUILD_MODE = model_reference
PRODUCT = $(MODELLIB)
BUILD_PRODUCT_TYPE = library
endif

#----------------------------- Source Files ------------------------------------


SRCS = $(S_FUNCTIONS)

ifeq ($(BUILD_MODE), pil_prebuild)
  SRCS += $(MODULES) $(MODEL).$(TARGET_LANG_EXT) $(EXT_SRC)
endif

ifeq ($(BUILD_MODE), pil_application)
  SRCS += $(TARGET_SRCS)
  PREBUILT_SRCS      = $(MODULES)
  PREBUILT_OBJS      = $(addsuffix $(OBJ_EXT), $(basename $(PREBUILT_SRCS)))
endif

ifeq ($(BUILD_MODE), real_time)
  SRCS += $(MODULES) $(MODEL).$(TARGET_LANG_EXT) $(EXT_SRC)
  SRCS += $(TARGET_SRCS)
endif

ifeq ($(BUILD_MODE), model_reference)
  SRCS += $(MODULES)
endif

USER_SRCS =
USER_OBJS       = $(addsuffix $(OBJ_EXT), $(basename $(USER_SRCS)))
LOCAL_USER_OBJS = $(notdir $(USER_OBJS))

OBJS      = $(addsuffix $(OBJ_EXT), $(basename $(SRCS))) $(USER_OBJS)
LINK_OBJS = $(addsuffix $(OBJ_EXT), $(basename $(SRCS))) $(LOCAL_USER_OBJS) $(PREBUILT_OBJS)

LIBS += $(S_FUNCTIONS_LIB)

#wildcard does not support slash, must be replaced with backslash at least for SHARED_SRC
SHARED_SRC := $(subst \,/,$(SHARED_SRC))
#SHARED_SRC_DIR := $(subst \,/,$(SHARED_SRC_DIR))
#SHARED_BIN_DIR := $(subst \,/,$(SHARED_BIN_DIR))
#SHARED_LIB := $(subst \,/,$(SHARED_LIB))

SHARED_SRC := $(wildcard $(SHARED_SRC))
SHARED_OBJS = $(addsuffix $(OBJ_EXT), $(basename $(SHARED_SRC)))

#-------------- Default target -----------
TARGETS = $(PRODUCT)

all: $(TARGETS)

# Need a rule to generate the build success string if product was already up to date
.PHONY : all
all : $(PRODUCT)
	@echo ### Created $(PRODUCT) successfully (or it was already up to date)

#----------------------------- Dependencies ------------------------------------

$(OBJS) : $(MAKEFILE) rtw_proj.tmw

$(SHARED_LIB) : $(SHARED_OBJS)
	@echo ### Creating $@
	$(AR) $@ $(SHARED_OBJS)
	@echo ### $@ Created



##-------------------------- Support for building modules ----------------------
ifeq ($(BUILD_MODE), pil_prebuild)
$(PRODUCT) : $(OBJS) $(SHARED_LIB)	
	@echo ### BUILD_MODE $(BUILD_MODE)
	@echo ### Created $(PRODUCT)
endif


ifeq ($(BUILD_MODE), model_reference)
$(PRODUCT) : $(OBJS) $(SHARED_LIB) $(LIBS)
	@echo ### BUILD_MODE $(BUILD_MODE)
	@echo ### Creating library ...
	$(AR) $(PRODUCT) $(OBJS) 
	@echo "$(BUILD_SUCCESS) $(BUILD_PRODUCT_TYPE): $@"	
endif		
		

ifeq ($(BUILD_MODE), pil_application)
$(PRODUCT) : $(PREBUILT_OBJS) $(OBJS) $(MODELLIB) $(MODELREF_LINK_LIBS) $(SHARED_LIB) $(LIBS)
	@echo ### BUILD_MODE $(BUILD_MODE)
	$(LD) $(MCPU) $^ $(LDLIBPIC) -o $(MODEL)$(PROGRAM_FILE_EXT) $(T_LINKER_SCRIPT),$(LDFLAGS)	 		
	@echo "$(BUILD_SUCCESS) $(BUILD_PRODUCT_TYPE): $@  [$(PIC_REF)]"
ifeq ($(ProduceHexOutput),1)
	$(GCCPATH)/$(BIN2HEX) -omf=$(OMF) $(PRODUCT)
	@echo "*** Converted $(MODEL)$(PROGRAM_FILE_EXT) to $(MODEL).hex"	
endif
ifeq ($(AssemblyListing),1)
	$(GCCPATH)/$(OBJDUMP) -omf=$(OMF) -S -f -Msymbolic $(PRODUCT) > $(MODEL).lst
		@echo "*** Created Assembly list file $(MODEL).lst"	
endif
endif
	
ifeq ($(BUILD_MODE), real_time)
#$(MODEL)$(PROGRAM_FILE_EXT) : $(OBJS) $(MODELREF_LINK_LIBS) $(SHARED_LIB) $(LIBS)
$(PRODUCT) : $(OBJS) $(MODELREF_LINK_LIBS) $(SHARED_LIB) $(LIBS)
	@echo ### BUILD_MODE $(BUILD_MODE)
	@echo ### BUILD_MODE $(BUILD_MODE)
	$(LD) $(MCPU) $(LINK_OBJS) $(MODELREF_LINK_LIBS) $(SHARED_LIB) $(LIBS) $(SYSLIBS) $(LDLIBPIC) -o $(MODEL)$(PROGRAM_FILE_EXT) $(T_LINKER_SCRIPT),$(LDFLAGS)	 		
	@echo $(BUILD_SUCCESS) $(BUILD_PRODUCT_TYPE): $@
ifeq ($(ProduceHexOutput),1)
	$(GCCPATH)/$(BIN2HEX) -omf=$(OMF) $(PRODUCT)
	@echo "*** Converted $(PRODUCT) to $(MODEL).hex"	
endif
ifeq ($(AssemblyListing),1)
	$(GCCPATH)/$(OBJDUMP) -omf=$(OMF) -S -f -Msymbolic $(PRODUCT) > $(MODEL).lst
		@echo "*** Created Assembly list file $(MODEL).lst"	
endif
endif



ifeq ($(OMF),elf)
ifeq (,$(wildcard $(MODEL).cof))	# if file exist
else
	$(DEL) $(MODEL).cof
endif
endif

ifeq ($(OMF),coff)
ifeq (,$(wildcard $(MODEL).elf))		# if file exist
else
	$(DEL) $(MODEL).elf
endif
endif		

ifeq ($(ProduceHexOutput),0)
ifeq (,$(wilcard $(MODEL).hex))	# if file exist
else
	$(DEL) $(MODEL).hex
endif
endif

ifeq ($(AssemblyListing),0)
ifeq (,$(wildcard $(MODEL).lst))		# if file exist
else
	$(DEL) $(RELATIVE_PATH_TO_ANCHOR)\$(MODEL).lst
endif
endif



mem_mgr.o : $(MATLAB_ROOT)/rtw/c/src/ext_mode/common/mem_mgr.c
	$(CC) -c -fno-strict-aliasing $(CFLAGS) $< $(CCOUTPUTFLAG) $@

%$(OBJ_EXT) : $(RELATIVE_PATH_TO_ANCHOR)/%.c
	@echo Compiling: $<
	$(CC) -c $(CFLAGS) $< $(CCOUTPUTFLAG) $@

%$(OBJ_EXT) : $(RELATIVE_PATH_TO_ANCHOR)/%.cpp
	@echo Compiling: $<
	$(CC) -c $(CPPFLAGS) $< $(CCOUTPUTFLAG) $@
	
%$(OBJ_EXT) : $(RELATIVE_PATH_TO_ANCHOR)/%.s
	@echo Compiling: $<
	$(CC) -c -mcpu=$(PIC_REF) $< -Wa,-g $(CCOUTPUTFLAG) $@

%$(OBJ_EXT) : %.c
	@echo Compiling: $<
	$(CC) -c $(CFLAGS) $< $(CCOUTPUTFLAG) $@

%$(OBJ_EXT) : %.cpp
	@echo Compiling: $<
	$(CC) -c $(CPPFLAGS) $< $(CCOUTPUTFLAG) $@

%$(OBJ_EXT) : %.s
	@echo Compiling: $<
	$(CC) -c -mcpu=$(PIC_REF) $< -Wa,-g $(CCOUTPUTFLAG) $@
	
%$(OBJ_EXT) : $(START_DIR)/clib/%.c
	@echo Compiling: $<
	$(CC) -c $(CFLAGS) $< $(CCOUTPUTFLAG) $@

%$(OBJ_EXT) : $(MATLAB_ROOT)/rtw/c/src/%.c
	@echo Compiling: $<
	$(CC) -c $(CFLAGS) $< $(CCOUTPUTFLAG) $@



%$(OBJ_EXT) : $(START_DIR)/clib/%.cpp
	@echo Compiling: $<
	$(CC) -c $(CPPFLAGS) $< $(CCOUTPUTFLAG) $@

%$(OBJ_EXT) : $(MATLAB_ROOT)/rtw/c/src/%.cpp
	@echo Compiling: $<
	$(CC) -c $(CPPFLAGS) $< $(CCOUTPUTFLAG) $@



%$(OBJ_EXT) : $(START_DIR)/clib/%.s
	@echo Compiling: $<
	$(CC) -c -mcpu=$(PIC_REF) $< -Wa,-g $(CCOUTPUTFLAG) $@

%$(OBJ_EXT) : $(MATLAB_ROOT)/rtw/c/src/%.s
	@echo Compiling: $<
	$(CC) -c -mcpu=$(PIC_REF) $< -Wa,-g $(CCOUTPUTFLAG) $@


