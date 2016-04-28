#
# Generated Makefile - do not edit!
#
# Edit the Makefile in the project folder instead (../Makefile). Each target
# has a -pre and a -post target defined where you can add customized code.
#
# This makefile implements configuration specific macros and targets.


# Include project Makefile
ifeq "${IGNORE_LOCAL}" "TRUE"
# do not include local makefile. User is passing all local related variables already
else
include Makefile
# Include makefile containing local settings
ifeq "$(wildcard nbproject/Makefile-local-default.mk)" "nbproject/Makefile-local-default.mk"
include nbproject/Makefile-local-default.mk
endif
endif

# Environment
MKDIR=gnumkdir -p
RM=rm -f 
MV=mv 
CP=cp 

# Macros
CND_CONF=default
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
IMAGE_TYPE=debug
OUTPUT_SUFFIX=elf
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/UnitTest.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
else
IMAGE_TYPE=production
OUTPUT_SUFFIX=hex
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/UnitTest.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
endif

# Object Directory
OBJECTDIR=build/${CND_CONF}/${IMAGE_TYPE}

# Distribution Directory
DISTDIR=dist/${CND_CONF}/${IMAGE_TYPE}

# Source Files Quoted if spaced
SOURCEFILES_QUOTED_IF_SPACED=UnitTest.c MCHP_I2C2_Interrupt.c MCHP_I2C2_Interrupt_data.c MCHP_SPI1_Interrupt.c MCHP_SPI1_Interrupt_data.c MCHP_UART1_Interrupt.c MCHP_UART4_Interrupt.c UnitTest_main.c

# Object Files Quoted if spaced
OBJECTFILES_QUOTED_IF_SPACED=${OBJECTDIR}/UnitTest.o ${OBJECTDIR}/MCHP_I2C2_Interrupt.o ${OBJECTDIR}/MCHP_I2C2_Interrupt_data.o ${OBJECTDIR}/MCHP_SPI1_Interrupt.o ${OBJECTDIR}/MCHP_SPI1_Interrupt_data.o ${OBJECTDIR}/MCHP_UART1_Interrupt.o ${OBJECTDIR}/MCHP_UART4_Interrupt.o ${OBJECTDIR}/UnitTest_main.o
POSSIBLE_DEPFILES=${OBJECTDIR}/UnitTest.o.d ${OBJECTDIR}/MCHP_I2C2_Interrupt.o.d ${OBJECTDIR}/MCHP_I2C2_Interrupt_data.o.d ${OBJECTDIR}/MCHP_SPI1_Interrupt.o.d ${OBJECTDIR}/MCHP_SPI1_Interrupt_data.o.d ${OBJECTDIR}/MCHP_UART1_Interrupt.o.d ${OBJECTDIR}/MCHP_UART4_Interrupt.o.d ${OBJECTDIR}/UnitTest_main.o.d

# Object Files
OBJECTFILES=${OBJECTDIR}/UnitTest.o ${OBJECTDIR}/MCHP_I2C2_Interrupt.o ${OBJECTDIR}/MCHP_I2C2_Interrupt_data.o ${OBJECTDIR}/MCHP_SPI1_Interrupt.o ${OBJECTDIR}/MCHP_SPI1_Interrupt_data.o ${OBJECTDIR}/MCHP_UART1_Interrupt.o ${OBJECTDIR}/MCHP_UART4_Interrupt.o ${OBJECTDIR}/UnitTest_main.o

# Source Files
SOURCEFILES=UnitTest.c MCHP_I2C2_Interrupt.c MCHP_I2C2_Interrupt_data.c MCHP_SPI1_Interrupt.c MCHP_SPI1_Interrupt_data.c MCHP_UART1_Interrupt.c MCHP_UART4_Interrupt.c UnitTest_main.c


CFLAGS=
ASFLAGS=
LDLIBSOPTIONS=

############# Tool locations ##########################################
# If you copy a project from one host to another, the path where the  #
# compiler is installed may be different.                             #
# If you open this project with MPLAB X in the new host, this         #
# makefile will be regenerated and the paths will be corrected.       #
#######################################################################
# fixDeps replaces a bunch of sed/cat/printf statements that slow down the build
FIXDEPS=fixDeps

.build-conf:  ${BUILD_SUBPROJECTS}
ifneq ($(INFORMATION_MESSAGE), )
	@echo $(INFORMATION_MESSAGE)
endif
	${MAKE}  -f nbproject/Makefile-default.mk dist/${CND_CONF}/${IMAGE_TYPE}/UnitTest.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}

MP_PROCESSOR_OPTION=33EP512MU810
MP_LINKER_FILE_OPTION=,--script=p33EP512MU810.gld
# ------------------------------------------------------------------------------------
# Rules for buildStep: compile
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/UnitTest.o: UnitTest.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/UnitTest.o.d 
	@${RM} ${OBJECTDIR}/UnitTest.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  UnitTest.c  -o ${OBJECTDIR}/UnitTest.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/UnitTest.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -mno-eds-warn  -omf=elf -mlarge-code -mlarge-data -O0 -I"UnitTest.X" -I"." -I"/UnitTest.X" -I"../mavlink/include/slugs" -I"../clib" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/include" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/dsPIC33E/h" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/generic/h" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/PERIPH~2" -I"C:/Program Files/MATLAB/R2011b/rtw/c/ert" -I"C:/Program Files/MATLAB/R2011b/extern/include" -I"C:/Program Files/MATLAB/R2011b/simulink/include" -I"C:/Program Files/MATLAB/R2011b/rtw/c/src" -I"C:/Program Files/MATLAB/R2011b/rtw/c/src/ext_mode/common" -msmart-io=1 -Wall -msfr-warn=off -O3 -mlarge-data -mlarge-scalar -fschedule-insns -fschedule-insns2
	@${FIXDEPS} "${OBJECTDIR}/UnitTest.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/MCHP_I2C2_Interrupt.o: MCHP_I2C2_Interrupt.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/MCHP_I2C2_Interrupt.o.d 
	@${RM} ${OBJECTDIR}/MCHP_I2C2_Interrupt.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  MCHP_I2C2_Interrupt.c  -o ${OBJECTDIR}/MCHP_I2C2_Interrupt.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/MCHP_I2C2_Interrupt.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -mno-eds-warn  -omf=elf -mlarge-code -mlarge-data -O0 -I"UnitTest.X" -I"." -I"/UnitTest.X" -I"../mavlink/include/slugs" -I"../clib" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/include" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/dsPIC33E/h" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/generic/h" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/PERIPH~2" -I"C:/Program Files/MATLAB/R2011b/rtw/c/ert" -I"C:/Program Files/MATLAB/R2011b/extern/include" -I"C:/Program Files/MATLAB/R2011b/simulink/include" -I"C:/Program Files/MATLAB/R2011b/rtw/c/src" -I"C:/Program Files/MATLAB/R2011b/rtw/c/src/ext_mode/common" -msmart-io=1 -Wall -msfr-warn=off -O3 -mlarge-data -mlarge-scalar -fschedule-insns -fschedule-insns2
	@${FIXDEPS} "${OBJECTDIR}/MCHP_I2C2_Interrupt.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/MCHP_I2C2_Interrupt_data.o: MCHP_I2C2_Interrupt_data.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/MCHP_I2C2_Interrupt_data.o.d 
	@${RM} ${OBJECTDIR}/MCHP_I2C2_Interrupt_data.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  MCHP_I2C2_Interrupt_data.c  -o ${OBJECTDIR}/MCHP_I2C2_Interrupt_data.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/MCHP_I2C2_Interrupt_data.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -mno-eds-warn  -omf=elf -mlarge-code -mlarge-data -O0 -I"UnitTest.X" -I"." -I"/UnitTest.X" -I"../mavlink/include/slugs" -I"../clib" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/include" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/dsPIC33E/h" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/generic/h" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/PERIPH~2" -I"C:/Program Files/MATLAB/R2011b/rtw/c/ert" -I"C:/Program Files/MATLAB/R2011b/extern/include" -I"C:/Program Files/MATLAB/R2011b/simulink/include" -I"C:/Program Files/MATLAB/R2011b/rtw/c/src" -I"C:/Program Files/MATLAB/R2011b/rtw/c/src/ext_mode/common" -msmart-io=1 -Wall -msfr-warn=off -O3 -mlarge-data -mlarge-scalar -fschedule-insns -fschedule-insns2
	@${FIXDEPS} "${OBJECTDIR}/MCHP_I2C2_Interrupt_data.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/MCHP_SPI1_Interrupt.o: MCHP_SPI1_Interrupt.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/MCHP_SPI1_Interrupt.o.d 
	@${RM} ${OBJECTDIR}/MCHP_SPI1_Interrupt.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  MCHP_SPI1_Interrupt.c  -o ${OBJECTDIR}/MCHP_SPI1_Interrupt.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/MCHP_SPI1_Interrupt.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -mno-eds-warn  -omf=elf -mlarge-code -mlarge-data -O0 -I"UnitTest.X" -I"." -I"/UnitTest.X" -I"../mavlink/include/slugs" -I"../clib" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/include" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/dsPIC33E/h" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/generic/h" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/PERIPH~2" -I"C:/Program Files/MATLAB/R2011b/rtw/c/ert" -I"C:/Program Files/MATLAB/R2011b/extern/include" -I"C:/Program Files/MATLAB/R2011b/simulink/include" -I"C:/Program Files/MATLAB/R2011b/rtw/c/src" -I"C:/Program Files/MATLAB/R2011b/rtw/c/src/ext_mode/common" -msmart-io=1 -Wall -msfr-warn=off -O3 -mlarge-data -mlarge-scalar -fschedule-insns -fschedule-insns2
	@${FIXDEPS} "${OBJECTDIR}/MCHP_SPI1_Interrupt.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/MCHP_SPI1_Interrupt_data.o: MCHP_SPI1_Interrupt_data.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/MCHP_SPI1_Interrupt_data.o.d 
	@${RM} ${OBJECTDIR}/MCHP_SPI1_Interrupt_data.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  MCHP_SPI1_Interrupt_data.c  -o ${OBJECTDIR}/MCHP_SPI1_Interrupt_data.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/MCHP_SPI1_Interrupt_data.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -mno-eds-warn  -omf=elf -mlarge-code -mlarge-data -O0 -I"UnitTest.X" -I"." -I"/UnitTest.X" -I"../mavlink/include/slugs" -I"../clib" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/include" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/dsPIC33E/h" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/generic/h" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/PERIPH~2" -I"C:/Program Files/MATLAB/R2011b/rtw/c/ert" -I"C:/Program Files/MATLAB/R2011b/extern/include" -I"C:/Program Files/MATLAB/R2011b/simulink/include" -I"C:/Program Files/MATLAB/R2011b/rtw/c/src" -I"C:/Program Files/MATLAB/R2011b/rtw/c/src/ext_mode/common" -msmart-io=1 -Wall -msfr-warn=off -O3 -mlarge-data -mlarge-scalar -fschedule-insns -fschedule-insns2
	@${FIXDEPS} "${OBJECTDIR}/MCHP_SPI1_Interrupt_data.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/MCHP_UART1_Interrupt.o: MCHP_UART1_Interrupt.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/MCHP_UART1_Interrupt.o.d 
	@${RM} ${OBJECTDIR}/MCHP_UART1_Interrupt.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  MCHP_UART1_Interrupt.c  -o ${OBJECTDIR}/MCHP_UART1_Interrupt.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/MCHP_UART1_Interrupt.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -mno-eds-warn  -omf=elf -mlarge-code -mlarge-data -O0 -I"UnitTest.X" -I"." -I"/UnitTest.X" -I"../mavlink/include/slugs" -I"../clib" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/include" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/dsPIC33E/h" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/generic/h" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/PERIPH~2" -I"C:/Program Files/MATLAB/R2011b/rtw/c/ert" -I"C:/Program Files/MATLAB/R2011b/extern/include" -I"C:/Program Files/MATLAB/R2011b/simulink/include" -I"C:/Program Files/MATLAB/R2011b/rtw/c/src" -I"C:/Program Files/MATLAB/R2011b/rtw/c/src/ext_mode/common" -msmart-io=1 -Wall -msfr-warn=off -O3 -mlarge-data -mlarge-scalar -fschedule-insns -fschedule-insns2
	@${FIXDEPS} "${OBJECTDIR}/MCHP_UART1_Interrupt.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/MCHP_UART4_Interrupt.o: MCHP_UART4_Interrupt.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/MCHP_UART4_Interrupt.o.d 
	@${RM} ${OBJECTDIR}/MCHP_UART4_Interrupt.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  MCHP_UART4_Interrupt.c  -o ${OBJECTDIR}/MCHP_UART4_Interrupt.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/MCHP_UART4_Interrupt.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -mno-eds-warn  -omf=elf -mlarge-code -mlarge-data -O0 -I"UnitTest.X" -I"." -I"/UnitTest.X" -I"../mavlink/include/slugs" -I"../clib" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/include" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/dsPIC33E/h" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/generic/h" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/PERIPH~2" -I"C:/Program Files/MATLAB/R2011b/rtw/c/ert" -I"C:/Program Files/MATLAB/R2011b/extern/include" -I"C:/Program Files/MATLAB/R2011b/simulink/include" -I"C:/Program Files/MATLAB/R2011b/rtw/c/src" -I"C:/Program Files/MATLAB/R2011b/rtw/c/src/ext_mode/common" -msmart-io=1 -Wall -msfr-warn=off -O3 -mlarge-data -mlarge-scalar -fschedule-insns -fschedule-insns2
	@${FIXDEPS} "${OBJECTDIR}/MCHP_UART4_Interrupt.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/UnitTest_main.o: UnitTest_main.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/UnitTest_main.o.d 
	@${RM} ${OBJECTDIR}/UnitTest_main.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  UnitTest_main.c  -o ${OBJECTDIR}/UnitTest_main.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/UnitTest_main.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -mno-eds-warn  -omf=elf -mlarge-code -mlarge-data -O0 -I"UnitTest.X" -I"." -I"/UnitTest.X" -I"../mavlink/include/slugs" -I"../clib" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/include" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/dsPIC33E/h" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/generic/h" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/PERIPH~2" -I"C:/Program Files/MATLAB/R2011b/rtw/c/ert" -I"C:/Program Files/MATLAB/R2011b/extern/include" -I"C:/Program Files/MATLAB/R2011b/simulink/include" -I"C:/Program Files/MATLAB/R2011b/rtw/c/src" -I"C:/Program Files/MATLAB/R2011b/rtw/c/src/ext_mode/common" -msmart-io=1 -Wall -msfr-warn=off -O3 -mlarge-data -mlarge-scalar -fschedule-insns -fschedule-insns2
	@${FIXDEPS} "${OBJECTDIR}/UnitTest_main.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
else
${OBJECTDIR}/UnitTest.o: UnitTest.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/UnitTest.o.d 
	@${RM} ${OBJECTDIR}/UnitTest.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  UnitTest.c  -o ${OBJECTDIR}/UnitTest.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/UnitTest.o.d"      -mno-eds-warn  -g -omf=elf -mlarge-code -mlarge-data -O0 -I"UnitTest.X" -I"." -I"/UnitTest.X" -I"../mavlink/include/slugs" -I"../clib" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/include" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/dsPIC33E/h" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/generic/h" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/PERIPH~2" -I"C:/Program Files/MATLAB/R2011b/rtw/c/ert" -I"C:/Program Files/MATLAB/R2011b/extern/include" -I"C:/Program Files/MATLAB/R2011b/simulink/include" -I"C:/Program Files/MATLAB/R2011b/rtw/c/src" -I"C:/Program Files/MATLAB/R2011b/rtw/c/src/ext_mode/common" -msmart-io=1 -Wall -msfr-warn=off -O3 -mlarge-data -mlarge-scalar -fschedule-insns -fschedule-insns2
	@${FIXDEPS} "${OBJECTDIR}/UnitTest.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/MCHP_I2C2_Interrupt.o: MCHP_I2C2_Interrupt.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/MCHP_I2C2_Interrupt.o.d 
	@${RM} ${OBJECTDIR}/MCHP_I2C2_Interrupt.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  MCHP_I2C2_Interrupt.c  -o ${OBJECTDIR}/MCHP_I2C2_Interrupt.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/MCHP_I2C2_Interrupt.o.d"      -mno-eds-warn  -g -omf=elf -mlarge-code -mlarge-data -O0 -I"UnitTest.X" -I"." -I"/UnitTest.X" -I"../mavlink/include/slugs" -I"../clib" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/include" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/dsPIC33E/h" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/generic/h" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/PERIPH~2" -I"C:/Program Files/MATLAB/R2011b/rtw/c/ert" -I"C:/Program Files/MATLAB/R2011b/extern/include" -I"C:/Program Files/MATLAB/R2011b/simulink/include" -I"C:/Program Files/MATLAB/R2011b/rtw/c/src" -I"C:/Program Files/MATLAB/R2011b/rtw/c/src/ext_mode/common" -msmart-io=1 -Wall -msfr-warn=off -O3 -mlarge-data -mlarge-scalar -fschedule-insns -fschedule-insns2
	@${FIXDEPS} "${OBJECTDIR}/MCHP_I2C2_Interrupt.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/MCHP_I2C2_Interrupt_data.o: MCHP_I2C2_Interrupt_data.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/MCHP_I2C2_Interrupt_data.o.d 
	@${RM} ${OBJECTDIR}/MCHP_I2C2_Interrupt_data.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  MCHP_I2C2_Interrupt_data.c  -o ${OBJECTDIR}/MCHP_I2C2_Interrupt_data.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/MCHP_I2C2_Interrupt_data.o.d"      -mno-eds-warn  -g -omf=elf -mlarge-code -mlarge-data -O0 -I"UnitTest.X" -I"." -I"/UnitTest.X" -I"../mavlink/include/slugs" -I"../clib" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/include" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/dsPIC33E/h" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/generic/h" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/PERIPH~2" -I"C:/Program Files/MATLAB/R2011b/rtw/c/ert" -I"C:/Program Files/MATLAB/R2011b/extern/include" -I"C:/Program Files/MATLAB/R2011b/simulink/include" -I"C:/Program Files/MATLAB/R2011b/rtw/c/src" -I"C:/Program Files/MATLAB/R2011b/rtw/c/src/ext_mode/common" -msmart-io=1 -Wall -msfr-warn=off -O3 -mlarge-data -mlarge-scalar -fschedule-insns -fschedule-insns2
	@${FIXDEPS} "${OBJECTDIR}/MCHP_I2C2_Interrupt_data.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/MCHP_SPI1_Interrupt.o: MCHP_SPI1_Interrupt.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/MCHP_SPI1_Interrupt.o.d 
	@${RM} ${OBJECTDIR}/MCHP_SPI1_Interrupt.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  MCHP_SPI1_Interrupt.c  -o ${OBJECTDIR}/MCHP_SPI1_Interrupt.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/MCHP_SPI1_Interrupt.o.d"      -mno-eds-warn  -g -omf=elf -mlarge-code -mlarge-data -O0 -I"UnitTest.X" -I"." -I"/UnitTest.X" -I"../mavlink/include/slugs" -I"../clib" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/include" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/dsPIC33E/h" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/generic/h" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/PERIPH~2" -I"C:/Program Files/MATLAB/R2011b/rtw/c/ert" -I"C:/Program Files/MATLAB/R2011b/extern/include" -I"C:/Program Files/MATLAB/R2011b/simulink/include" -I"C:/Program Files/MATLAB/R2011b/rtw/c/src" -I"C:/Program Files/MATLAB/R2011b/rtw/c/src/ext_mode/common" -msmart-io=1 -Wall -msfr-warn=off -O3 -mlarge-data -mlarge-scalar -fschedule-insns -fschedule-insns2
	@${FIXDEPS} "${OBJECTDIR}/MCHP_SPI1_Interrupt.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/MCHP_SPI1_Interrupt_data.o: MCHP_SPI1_Interrupt_data.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/MCHP_SPI1_Interrupt_data.o.d 
	@${RM} ${OBJECTDIR}/MCHP_SPI1_Interrupt_data.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  MCHP_SPI1_Interrupt_data.c  -o ${OBJECTDIR}/MCHP_SPI1_Interrupt_data.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/MCHP_SPI1_Interrupt_data.o.d"      -mno-eds-warn  -g -omf=elf -mlarge-code -mlarge-data -O0 -I"UnitTest.X" -I"." -I"/UnitTest.X" -I"../mavlink/include/slugs" -I"../clib" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/include" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/dsPIC33E/h" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/generic/h" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/PERIPH~2" -I"C:/Program Files/MATLAB/R2011b/rtw/c/ert" -I"C:/Program Files/MATLAB/R2011b/extern/include" -I"C:/Program Files/MATLAB/R2011b/simulink/include" -I"C:/Program Files/MATLAB/R2011b/rtw/c/src" -I"C:/Program Files/MATLAB/R2011b/rtw/c/src/ext_mode/common" -msmart-io=1 -Wall -msfr-warn=off -O3 -mlarge-data -mlarge-scalar -fschedule-insns -fschedule-insns2
	@${FIXDEPS} "${OBJECTDIR}/MCHP_SPI1_Interrupt_data.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/MCHP_UART1_Interrupt.o: MCHP_UART1_Interrupt.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/MCHP_UART1_Interrupt.o.d 
	@${RM} ${OBJECTDIR}/MCHP_UART1_Interrupt.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  MCHP_UART1_Interrupt.c  -o ${OBJECTDIR}/MCHP_UART1_Interrupt.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/MCHP_UART1_Interrupt.o.d"      -mno-eds-warn  -g -omf=elf -mlarge-code -mlarge-data -O0 -I"UnitTest.X" -I"." -I"/UnitTest.X" -I"../mavlink/include/slugs" -I"../clib" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/include" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/dsPIC33E/h" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/generic/h" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/PERIPH~2" -I"C:/Program Files/MATLAB/R2011b/rtw/c/ert" -I"C:/Program Files/MATLAB/R2011b/extern/include" -I"C:/Program Files/MATLAB/R2011b/simulink/include" -I"C:/Program Files/MATLAB/R2011b/rtw/c/src" -I"C:/Program Files/MATLAB/R2011b/rtw/c/src/ext_mode/common" -msmart-io=1 -Wall -msfr-warn=off -O3 -mlarge-data -mlarge-scalar -fschedule-insns -fschedule-insns2
	@${FIXDEPS} "${OBJECTDIR}/MCHP_UART1_Interrupt.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/MCHP_UART4_Interrupt.o: MCHP_UART4_Interrupt.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/MCHP_UART4_Interrupt.o.d 
	@${RM} ${OBJECTDIR}/MCHP_UART4_Interrupt.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  MCHP_UART4_Interrupt.c  -o ${OBJECTDIR}/MCHP_UART4_Interrupt.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/MCHP_UART4_Interrupt.o.d"      -mno-eds-warn  -g -omf=elf -mlarge-code -mlarge-data -O0 -I"UnitTest.X" -I"." -I"/UnitTest.X" -I"../mavlink/include/slugs" -I"../clib" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/include" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/dsPIC33E/h" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/generic/h" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/PERIPH~2" -I"C:/Program Files/MATLAB/R2011b/rtw/c/ert" -I"C:/Program Files/MATLAB/R2011b/extern/include" -I"C:/Program Files/MATLAB/R2011b/simulink/include" -I"C:/Program Files/MATLAB/R2011b/rtw/c/src" -I"C:/Program Files/MATLAB/R2011b/rtw/c/src/ext_mode/common" -msmart-io=1 -Wall -msfr-warn=off -O3 -mlarge-data -mlarge-scalar -fschedule-insns -fschedule-insns2
	@${FIXDEPS} "${OBJECTDIR}/MCHP_UART4_Interrupt.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/UnitTest_main.o: UnitTest_main.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/UnitTest_main.o.d 
	@${RM} ${OBJECTDIR}/UnitTest_main.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  UnitTest_main.c  -o ${OBJECTDIR}/UnitTest_main.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/UnitTest_main.o.d"      -mno-eds-warn  -g -omf=elf -mlarge-code -mlarge-data -O0 -I"UnitTest.X" -I"." -I"/UnitTest.X" -I"../mavlink/include/slugs" -I"../clib" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/include" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/dsPIC33E/h" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/generic/h" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/PERIPH~2" -I"C:/Program Files/MATLAB/R2011b/rtw/c/ert" -I"C:/Program Files/MATLAB/R2011b/extern/include" -I"C:/Program Files/MATLAB/R2011b/simulink/include" -I"C:/Program Files/MATLAB/R2011b/rtw/c/src" -I"C:/Program Files/MATLAB/R2011b/rtw/c/src/ext_mode/common" -msmart-io=1 -Wall -msfr-warn=off -O3 -mlarge-data -mlarge-scalar -fschedule-insns -fschedule-insns2
	@${FIXDEPS} "${OBJECTDIR}/UnitTest_main.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: assemble
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: assemblePreproc
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: link
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
dist/${CND_CONF}/${IMAGE_TYPE}/UnitTest.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk MavlinkComm.o adisCube16405.o circBuffer.o gpsPort.o gpsUblox.o hil.o mavlinkSensorMcu.o novatel.o updateSensorMcuState.o C:/PROGRA~2/MICROC~1/xc16/v1.26/lib/dsPIC33E/libp33EP512MU810-elf.a C:/PROGRA~2/MICROC~1/xc16/v1.26/lib/libpic30-elf.a C:/PROGRA~2/MICROC~1/xc16/v1.26/lib/libm-elf.a C:/PROGRA~2/MICROC~1/xc16/v1.26/lib/libc-elf.a C:/PROGRA~2/MICROC~1/xc16/v1.26/lib/libq-elf.a C:/PROGRA~2/MICROC~1/xc16/v1.26/lib/libq-dsp-elf.a  
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -o dist/${CND_CONF}/${IMAGE_TYPE}/UnitTest.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}  ${OBJECTFILES_QUOTED_IF_SPACED}  MavlinkComm.o adisCube16405.o circBuffer.o gpsPort.o gpsUblox.o hil.o mavlinkSensorMcu.o novatel.o updateSensorMcuState.o  C:\PROGRA~2\MICROC~1\xc16\v1.26\lib\dsPIC33E\libp33EP512MU810-elf.a C:\PROGRA~2\MICROC~1\xc16\v1.26\lib\libpic30-elf.a C:\PROGRA~2\MICROC~1\xc16\v1.26\lib\libm-elf.a C:\PROGRA~2\MICROC~1\xc16\v1.26\lib\libc-elf.a C:\PROGRA~2\MICROC~1\xc16\v1.26\lib\libq-elf.a C:\PROGRA~2\MICROC~1\xc16\v1.26\lib\libq-dsp-elf.a  -mcpu=$(MP_PROCESSOR_OPTION)        -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -omf=elf  -mreserve=data@0x1000:0x101B -mreserve=data@0x101C:0x101D -mreserve=data@0x101E:0x101F -mreserve=data@0x1020:0x1021 -mreserve=data@0x1022:0x1023 -mreserve=data@0x1024:0x1027 -mreserve=data@0x1028:0x104F   -Wl,--local-stack,--defsym=__MPLAB_BUILD=1,--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1,--defsym=__MPLAB_DEBUGGER_PK3=1,$(MP_LINKER_FILE_OPTION),--stack=16,--check-sections,--data-init,--pack-data,--handles,--isr,--no-gc-sections,--fill-upper=0,--stackguard=16,--library-path="UnitTest.X",--library-path=".",--library-path="/UnitTest.X",--library-path="-I/mavlink/include/slugs",--library-path="/clib",--library-path="C:/PROGRA~2/MICROC~1/xc16/v1.26/include",--library-path="C:/PROGRA~2/MICROC~1/xc16/v1.26/support/dsPIC33E/h",--library-path="C:/PROGRA~2/MICROC~1/xc16/v1.26/support/generic/h",--library-path="C:/PROGRA~2/MICROC~1/xc16/v1.26/support/PERIPH~2",--library-path="C:/Program Files/MATLAB/R2011b/rtw/c/ert",--library-path="C:/Program Files/MATLAB/R2011b/extern/include",--library-path="C:/Program Files/MATLAB/R2011b/simulink/include",--library-path="C:/Program Files/MATLAB/R2011b/rtw/c/src",--library-path="C:/Program Files/MATLAB/R2011b/rtw/c/src/ext_mode/common",--no-force-link,--smart-io,-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map",--report-mem$(MP_EXTRA_LD_POST) 
	
else
dist/${CND_CONF}/${IMAGE_TYPE}/UnitTest.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk MavlinkComm.o adisCube16405.o circBuffer.o gpsPort.o gpsUblox.o hil.o mavlinkSensorMcu.o novatel.o updateSensorMcuState.o C:/PROGRA~2/MICROC~1/xc16/v1.26/lib/dsPIC33E/libp33EP512MU810-elf.a C:/PROGRA~2/MICROC~1/xc16/v1.26/lib/libpic30-elf.a C:/PROGRA~2/MICROC~1/xc16/v1.26/lib/libm-elf.a C:/PROGRA~2/MICROC~1/xc16/v1.26/lib/libc-elf.a C:/PROGRA~2/MICROC~1/xc16/v1.26/lib/libq-elf.a C:/PROGRA~2/MICROC~1/xc16/v1.26/lib/libq-dsp-elf.a 
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -o dist/${CND_CONF}/${IMAGE_TYPE}/UnitTest.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX}  ${OBJECTFILES_QUOTED_IF_SPACED}  MavlinkComm.o adisCube16405.o circBuffer.o gpsPort.o gpsUblox.o hil.o mavlinkSensorMcu.o novatel.o updateSensorMcuState.o  C:\PROGRA~2\MICROC~1\xc16\v1.26\lib\dsPIC33E\libp33EP512MU810-elf.a C:\PROGRA~2\MICROC~1\xc16\v1.26\lib\libpic30-elf.a C:\PROGRA~2\MICROC~1\xc16\v1.26\lib\libm-elf.a C:\PROGRA~2\MICROC~1\xc16\v1.26\lib\libc-elf.a C:\PROGRA~2\MICROC~1\xc16\v1.26\lib\libq-elf.a C:\PROGRA~2\MICROC~1\xc16\v1.26\lib\libq-dsp-elf.a  -mcpu=$(MP_PROCESSOR_OPTION)        -omf=elf -Wl,--local-stack,--defsym=__MPLAB_BUILD=1,$(MP_LINKER_FILE_OPTION),--stack=16,--check-sections,--data-init,--pack-data,--handles,--isr,--no-gc-sections,--fill-upper=0,--stackguard=16,--library-path="UnitTest.X",--library-path=".",--library-path="/UnitTest.X",--library-path="-I/mavlink/include/slugs",--library-path="/clib",--library-path="C:/PROGRA~2/MICROC~1/xc16/v1.26/include",--library-path="C:/PROGRA~2/MICROC~1/xc16/v1.26/support/dsPIC33E/h",--library-path="C:/PROGRA~2/MICROC~1/xc16/v1.26/support/generic/h",--library-path="C:/PROGRA~2/MICROC~1/xc16/v1.26/support/PERIPH~2",--library-path="C:/Program Files/MATLAB/R2011b/rtw/c/ert",--library-path="C:/Program Files/MATLAB/R2011b/extern/include",--library-path="C:/Program Files/MATLAB/R2011b/simulink/include",--library-path="C:/Program Files/MATLAB/R2011b/rtw/c/src",--library-path="C:/Program Files/MATLAB/R2011b/rtw/c/src/ext_mode/common",--no-force-link,--smart-io,-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map",--report-mem$(MP_EXTRA_LD_POST) 
	${MP_CC_DIR}\\xc16-bin2hex dist/${CND_CONF}/${IMAGE_TYPE}/UnitTest.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} -a  -omf=elf  
	
endif


# Subprojects
.build-subprojects:


# Subprojects
.clean-subprojects:

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r build/default
	${RM} -r dist/default

# Enable dependency checking
.dep.inc: .depcheck-impl

DEPFILES=$(shell mplabwildcard ${POSSIBLE_DEPFILES})
ifneq (${DEPFILES},)
include ${DEPFILES}
endif
