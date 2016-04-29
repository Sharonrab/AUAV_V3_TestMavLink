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
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/AUAV3_AND_SLUGS_SENSOR.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
else
IMAGE_TYPE=production
OUTPUT_SUFFIX=hex
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/AUAV3_AND_SLUGS_SENSOR.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
endif

# Object Directory
OBJECTDIR=build/${CND_CONF}/${IMAGE_TYPE}

# Distribution Directory
DISTDIR=dist/${CND_CONF}/${IMAGE_TYPE}

# Source Files Quoted if spaced
SOURCEFILES_QUOTED_IF_SPACED=AUAV3_AND_SLUGS_SENSOR.c AUAV3_AND_SLUGS_SENSOR_data.c AUAV3_AND_SLUGS_SENSOR_main.c MCHP_I2C2_Interrupt.c MCHP_I2C2_Interrupt_data.c MCHP_IC_Interrupt.c MCHP_SPI1_Interrupt.c MCHP_SPI1_Interrupt_data.c MCHP_UART1_Interrupt.c MCHP_UART4_Interrupt.c ../clib/MavlinkComm.c ../clib/adisCube16405.c ../clib/apUtils.c ../clib/circBuffer.c ../clib/gpsPort.c ../clib/gpsUblox.c ../clib/hil.c ../clib/mavlinkSensorMcu.c ../clib/novatel.c rtGetInf.c rtGetNaN.c rt_nonfinite.c ../clib/updateSensorMcuState.c

# Object Files Quoted if spaced
OBJECTFILES_QUOTED_IF_SPACED=${OBJECTDIR}/AUAV3_AND_SLUGS_SENSOR.o ${OBJECTDIR}/AUAV3_AND_SLUGS_SENSOR_data.o ${OBJECTDIR}/AUAV3_AND_SLUGS_SENSOR_main.o ${OBJECTDIR}/MCHP_I2C2_Interrupt.o ${OBJECTDIR}/MCHP_I2C2_Interrupt_data.o ${OBJECTDIR}/MCHP_IC_Interrupt.o ${OBJECTDIR}/MCHP_SPI1_Interrupt.o ${OBJECTDIR}/MCHP_SPI1_Interrupt_data.o ${OBJECTDIR}/MCHP_UART1_Interrupt.o ${OBJECTDIR}/MCHP_UART4_Interrupt.o ${OBJECTDIR}/_ext/761100751/MavlinkComm.o ${OBJECTDIR}/_ext/761100751/adisCube16405.o ${OBJECTDIR}/_ext/761100751/apUtils.o ${OBJECTDIR}/_ext/761100751/circBuffer.o ${OBJECTDIR}/_ext/761100751/gpsPort.o ${OBJECTDIR}/_ext/761100751/gpsUblox.o ${OBJECTDIR}/_ext/761100751/hil.o ${OBJECTDIR}/_ext/761100751/mavlinkSensorMcu.o ${OBJECTDIR}/_ext/761100751/novatel.o ${OBJECTDIR}/rtGetInf.o ${OBJECTDIR}/rtGetNaN.o ${OBJECTDIR}/rt_nonfinite.o ${OBJECTDIR}/_ext/761100751/updateSensorMcuState.o
POSSIBLE_DEPFILES=${OBJECTDIR}/AUAV3_AND_SLUGS_SENSOR.o.d ${OBJECTDIR}/AUAV3_AND_SLUGS_SENSOR_data.o.d ${OBJECTDIR}/AUAV3_AND_SLUGS_SENSOR_main.o.d ${OBJECTDIR}/MCHP_I2C2_Interrupt.o.d ${OBJECTDIR}/MCHP_I2C2_Interrupt_data.o.d ${OBJECTDIR}/MCHP_IC_Interrupt.o.d ${OBJECTDIR}/MCHP_SPI1_Interrupt.o.d ${OBJECTDIR}/MCHP_SPI1_Interrupt_data.o.d ${OBJECTDIR}/MCHP_UART1_Interrupt.o.d ${OBJECTDIR}/MCHP_UART4_Interrupt.o.d ${OBJECTDIR}/_ext/761100751/MavlinkComm.o.d ${OBJECTDIR}/_ext/761100751/adisCube16405.o.d ${OBJECTDIR}/_ext/761100751/apUtils.o.d ${OBJECTDIR}/_ext/761100751/circBuffer.o.d ${OBJECTDIR}/_ext/761100751/gpsPort.o.d ${OBJECTDIR}/_ext/761100751/gpsUblox.o.d ${OBJECTDIR}/_ext/761100751/hil.o.d ${OBJECTDIR}/_ext/761100751/mavlinkSensorMcu.o.d ${OBJECTDIR}/_ext/761100751/novatel.o.d ${OBJECTDIR}/rtGetInf.o.d ${OBJECTDIR}/rtGetNaN.o.d ${OBJECTDIR}/rt_nonfinite.o.d ${OBJECTDIR}/_ext/761100751/updateSensorMcuState.o.d

# Object Files
OBJECTFILES=${OBJECTDIR}/AUAV3_AND_SLUGS_SENSOR.o ${OBJECTDIR}/AUAV3_AND_SLUGS_SENSOR_data.o ${OBJECTDIR}/AUAV3_AND_SLUGS_SENSOR_main.o ${OBJECTDIR}/MCHP_I2C2_Interrupt.o ${OBJECTDIR}/MCHP_I2C2_Interrupt_data.o ${OBJECTDIR}/MCHP_IC_Interrupt.o ${OBJECTDIR}/MCHP_SPI1_Interrupt.o ${OBJECTDIR}/MCHP_SPI1_Interrupt_data.o ${OBJECTDIR}/MCHP_UART1_Interrupt.o ${OBJECTDIR}/MCHP_UART4_Interrupt.o ${OBJECTDIR}/_ext/761100751/MavlinkComm.o ${OBJECTDIR}/_ext/761100751/adisCube16405.o ${OBJECTDIR}/_ext/761100751/apUtils.o ${OBJECTDIR}/_ext/761100751/circBuffer.o ${OBJECTDIR}/_ext/761100751/gpsPort.o ${OBJECTDIR}/_ext/761100751/gpsUblox.o ${OBJECTDIR}/_ext/761100751/hil.o ${OBJECTDIR}/_ext/761100751/mavlinkSensorMcu.o ${OBJECTDIR}/_ext/761100751/novatel.o ${OBJECTDIR}/rtGetInf.o ${OBJECTDIR}/rtGetNaN.o ${OBJECTDIR}/rt_nonfinite.o ${OBJECTDIR}/_ext/761100751/updateSensorMcuState.o

# Source Files
SOURCEFILES=AUAV3_AND_SLUGS_SENSOR.c AUAV3_AND_SLUGS_SENSOR_data.c AUAV3_AND_SLUGS_SENSOR_main.c MCHP_I2C2_Interrupt.c MCHP_I2C2_Interrupt_data.c MCHP_IC_Interrupt.c MCHP_SPI1_Interrupt.c MCHP_SPI1_Interrupt_data.c MCHP_UART1_Interrupt.c MCHP_UART4_Interrupt.c ../clib/MavlinkComm.c ../clib/adisCube16405.c ../clib/apUtils.c ../clib/circBuffer.c ../clib/gpsPort.c ../clib/gpsUblox.c ../clib/hil.c ../clib/mavlinkSensorMcu.c ../clib/novatel.c rtGetInf.c rtGetNaN.c rt_nonfinite.c ../clib/updateSensorMcuState.c


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
	${MAKE}  -f nbproject/Makefile-default.mk dist/${CND_CONF}/${IMAGE_TYPE}/AUAV3_AND_SLUGS_SENSOR.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}

MP_PROCESSOR_OPTION=33EP512MU810
MP_LINKER_FILE_OPTION=,--script=p33EP512MU810.gld
# ------------------------------------------------------------------------------------
# Rules for buildStep: compile
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/AUAV3_AND_SLUGS_SENSOR.o: AUAV3_AND_SLUGS_SENSOR.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/AUAV3_AND_SLUGS_SENSOR.o.d 
	@${RM} ${OBJECTDIR}/AUAV3_AND_SLUGS_SENSOR.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  AUAV3_AND_SLUGS_SENSOR.c  -o ${OBJECTDIR}/AUAV3_AND_SLUGS_SENSOR.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/AUAV3_AND_SLUGS_SENSOR.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -mno-eds-warn  -omf=elf -legacy-libc  -mlarge-code -mlarge-data -O0 -I"AUAV3_AND_SLUGS_SENSOR.X" -I"." -I".." -I"../mavlink/include/slugs" -I"C:/Users/sharon/Documents/GitHub/AUAV_V3_TestMavLink/../clib" -I"../clib" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/include" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/dsPIC33E/h" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/generic/h" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/PERIPH~2" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/ert" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/extern/include" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/simulink/include" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src/ext_mode/common" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src/ext_mode/common" -msmart-io=1 -Wall -msfr-warn=off   -O3 -mlarge-data -mlarge-scalar -fschedule-insns -fschedule-insns2
	@${FIXDEPS} "${OBJECTDIR}/AUAV3_AND_SLUGS_SENSOR.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/AUAV3_AND_SLUGS_SENSOR_data.o: AUAV3_AND_SLUGS_SENSOR_data.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/AUAV3_AND_SLUGS_SENSOR_data.o.d 
	@${RM} ${OBJECTDIR}/AUAV3_AND_SLUGS_SENSOR_data.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  AUAV3_AND_SLUGS_SENSOR_data.c  -o ${OBJECTDIR}/AUAV3_AND_SLUGS_SENSOR_data.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/AUAV3_AND_SLUGS_SENSOR_data.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -mno-eds-warn  -omf=elf -legacy-libc  -mlarge-code -mlarge-data -O0 -I"AUAV3_AND_SLUGS_SENSOR.X" -I"." -I".." -I"../mavlink/include/slugs" -I"C:/Users/sharon/Documents/GitHub/AUAV_V3_TestMavLink/../clib" -I"../clib" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/include" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/dsPIC33E/h" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/generic/h" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/PERIPH~2" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/ert" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/extern/include" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/simulink/include" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src/ext_mode/common" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src/ext_mode/common" -msmart-io=1 -Wall -msfr-warn=off   -O3 -mlarge-data -mlarge-scalar -fschedule-insns -fschedule-insns2
	@${FIXDEPS} "${OBJECTDIR}/AUAV3_AND_SLUGS_SENSOR_data.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/AUAV3_AND_SLUGS_SENSOR_main.o: AUAV3_AND_SLUGS_SENSOR_main.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/AUAV3_AND_SLUGS_SENSOR_main.o.d 
	@${RM} ${OBJECTDIR}/AUAV3_AND_SLUGS_SENSOR_main.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  AUAV3_AND_SLUGS_SENSOR_main.c  -o ${OBJECTDIR}/AUAV3_AND_SLUGS_SENSOR_main.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/AUAV3_AND_SLUGS_SENSOR_main.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -mno-eds-warn  -omf=elf -legacy-libc  -mlarge-code -mlarge-data -O0 -I"AUAV3_AND_SLUGS_SENSOR.X" -I"." -I".." -I"../mavlink/include/slugs" -I"C:/Users/sharon/Documents/GitHub/AUAV_V3_TestMavLink/../clib" -I"../clib" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/include" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/dsPIC33E/h" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/generic/h" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/PERIPH~2" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/ert" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/extern/include" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/simulink/include" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src/ext_mode/common" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src/ext_mode/common" -msmart-io=1 -Wall -msfr-warn=off   -O3 -mlarge-data -mlarge-scalar -fschedule-insns -fschedule-insns2
	@${FIXDEPS} "${OBJECTDIR}/AUAV3_AND_SLUGS_SENSOR_main.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/MCHP_I2C2_Interrupt.o: MCHP_I2C2_Interrupt.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/MCHP_I2C2_Interrupt.o.d 
	@${RM} ${OBJECTDIR}/MCHP_I2C2_Interrupt.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  MCHP_I2C2_Interrupt.c  -o ${OBJECTDIR}/MCHP_I2C2_Interrupt.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/MCHP_I2C2_Interrupt.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -mno-eds-warn  -omf=elf -legacy-libc  -mlarge-code -mlarge-data -O0 -I"AUAV3_AND_SLUGS_SENSOR.X" -I"." -I".." -I"../mavlink/include/slugs" -I"C:/Users/sharon/Documents/GitHub/AUAV_V3_TestMavLink/../clib" -I"../clib" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/include" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/dsPIC33E/h" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/generic/h" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/PERIPH~2" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/ert" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/extern/include" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/simulink/include" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src/ext_mode/common" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src/ext_mode/common" -msmart-io=1 -Wall -msfr-warn=off   -O3 -mlarge-data -mlarge-scalar -fschedule-insns -fschedule-insns2
	@${FIXDEPS} "${OBJECTDIR}/MCHP_I2C2_Interrupt.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/MCHP_I2C2_Interrupt_data.o: MCHP_I2C2_Interrupt_data.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/MCHP_I2C2_Interrupt_data.o.d 
	@${RM} ${OBJECTDIR}/MCHP_I2C2_Interrupt_data.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  MCHP_I2C2_Interrupt_data.c  -o ${OBJECTDIR}/MCHP_I2C2_Interrupt_data.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/MCHP_I2C2_Interrupt_data.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -mno-eds-warn  -omf=elf -legacy-libc  -mlarge-code -mlarge-data -O0 -I"AUAV3_AND_SLUGS_SENSOR.X" -I"." -I".." -I"../mavlink/include/slugs" -I"C:/Users/sharon/Documents/GitHub/AUAV_V3_TestMavLink/../clib" -I"../clib" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/include" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/dsPIC33E/h" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/generic/h" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/PERIPH~2" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/ert" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/extern/include" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/simulink/include" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src/ext_mode/common" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src/ext_mode/common" -msmart-io=1 -Wall -msfr-warn=off   -O3 -mlarge-data -mlarge-scalar -fschedule-insns -fschedule-insns2
	@${FIXDEPS} "${OBJECTDIR}/MCHP_I2C2_Interrupt_data.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/MCHP_IC_Interrupt.o: MCHP_IC_Interrupt.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/MCHP_IC_Interrupt.o.d 
	@${RM} ${OBJECTDIR}/MCHP_IC_Interrupt.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  MCHP_IC_Interrupt.c  -o ${OBJECTDIR}/MCHP_IC_Interrupt.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/MCHP_IC_Interrupt.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -mno-eds-warn  -omf=elf -legacy-libc  -mlarge-code -mlarge-data -O0 -I"AUAV3_AND_SLUGS_SENSOR.X" -I"." -I".." -I"../mavlink/include/slugs" -I"C:/Users/sharon/Documents/GitHub/AUAV_V3_TestMavLink/../clib" -I"../clib" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/include" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/dsPIC33E/h" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/generic/h" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/PERIPH~2" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/ert" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/extern/include" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/simulink/include" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src/ext_mode/common" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src/ext_mode/common" -msmart-io=1 -Wall -msfr-warn=off   -O3 -mlarge-data -mlarge-scalar -fschedule-insns -fschedule-insns2
	@${FIXDEPS} "${OBJECTDIR}/MCHP_IC_Interrupt.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/MCHP_SPI1_Interrupt.o: MCHP_SPI1_Interrupt.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/MCHP_SPI1_Interrupt.o.d 
	@${RM} ${OBJECTDIR}/MCHP_SPI1_Interrupt.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  MCHP_SPI1_Interrupt.c  -o ${OBJECTDIR}/MCHP_SPI1_Interrupt.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/MCHP_SPI1_Interrupt.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -mno-eds-warn  -omf=elf -legacy-libc  -mlarge-code -mlarge-data -O0 -I"AUAV3_AND_SLUGS_SENSOR.X" -I"." -I".." -I"../mavlink/include/slugs" -I"C:/Users/sharon/Documents/GitHub/AUAV_V3_TestMavLink/../clib" -I"../clib" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/include" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/dsPIC33E/h" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/generic/h" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/PERIPH~2" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/ert" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/extern/include" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/simulink/include" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src/ext_mode/common" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src/ext_mode/common" -msmart-io=1 -Wall -msfr-warn=off   -O3 -mlarge-data -mlarge-scalar -fschedule-insns -fschedule-insns2
	@${FIXDEPS} "${OBJECTDIR}/MCHP_SPI1_Interrupt.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/MCHP_SPI1_Interrupt_data.o: MCHP_SPI1_Interrupt_data.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/MCHP_SPI1_Interrupt_data.o.d 
	@${RM} ${OBJECTDIR}/MCHP_SPI1_Interrupt_data.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  MCHP_SPI1_Interrupt_data.c  -o ${OBJECTDIR}/MCHP_SPI1_Interrupt_data.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/MCHP_SPI1_Interrupt_data.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -mno-eds-warn  -omf=elf -legacy-libc  -mlarge-code -mlarge-data -O0 -I"AUAV3_AND_SLUGS_SENSOR.X" -I"." -I".." -I"../mavlink/include/slugs" -I"C:/Users/sharon/Documents/GitHub/AUAV_V3_TestMavLink/../clib" -I"../clib" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/include" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/dsPIC33E/h" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/generic/h" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/PERIPH~2" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/ert" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/extern/include" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/simulink/include" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src/ext_mode/common" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src/ext_mode/common" -msmart-io=1 -Wall -msfr-warn=off   -O3 -mlarge-data -mlarge-scalar -fschedule-insns -fschedule-insns2
	@${FIXDEPS} "${OBJECTDIR}/MCHP_SPI1_Interrupt_data.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/MCHP_UART1_Interrupt.o: MCHP_UART1_Interrupt.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/MCHP_UART1_Interrupt.o.d 
	@${RM} ${OBJECTDIR}/MCHP_UART1_Interrupt.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  MCHP_UART1_Interrupt.c  -o ${OBJECTDIR}/MCHP_UART1_Interrupt.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/MCHP_UART1_Interrupt.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -mno-eds-warn  -omf=elf -legacy-libc  -mlarge-code -mlarge-data -O0 -I"AUAV3_AND_SLUGS_SENSOR.X" -I"." -I".." -I"../mavlink/include/slugs" -I"C:/Users/sharon/Documents/GitHub/AUAV_V3_TestMavLink/../clib" -I"../clib" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/include" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/dsPIC33E/h" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/generic/h" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/PERIPH~2" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/ert" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/extern/include" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/simulink/include" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src/ext_mode/common" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src/ext_mode/common" -msmart-io=1 -Wall -msfr-warn=off   -O3 -mlarge-data -mlarge-scalar -fschedule-insns -fschedule-insns2
	@${FIXDEPS} "${OBJECTDIR}/MCHP_UART1_Interrupt.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/MCHP_UART4_Interrupt.o: MCHP_UART4_Interrupt.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/MCHP_UART4_Interrupt.o.d 
	@${RM} ${OBJECTDIR}/MCHP_UART4_Interrupt.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  MCHP_UART4_Interrupt.c  -o ${OBJECTDIR}/MCHP_UART4_Interrupt.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/MCHP_UART4_Interrupt.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -mno-eds-warn  -omf=elf -legacy-libc  -mlarge-code -mlarge-data -O0 -I"AUAV3_AND_SLUGS_SENSOR.X" -I"." -I".." -I"../mavlink/include/slugs" -I"C:/Users/sharon/Documents/GitHub/AUAV_V3_TestMavLink/../clib" -I"../clib" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/include" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/dsPIC33E/h" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/generic/h" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/PERIPH~2" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/ert" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/extern/include" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/simulink/include" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src/ext_mode/common" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src/ext_mode/common" -msmart-io=1 -Wall -msfr-warn=off   -O3 -mlarge-data -mlarge-scalar -fschedule-insns -fschedule-insns2
	@${FIXDEPS} "${OBJECTDIR}/MCHP_UART4_Interrupt.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/761100751/MavlinkComm.o: ../clib/MavlinkComm.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/761100751" 
	@${RM} ${OBJECTDIR}/_ext/761100751/MavlinkComm.o.d 
	@${RM} ${OBJECTDIR}/_ext/761100751/MavlinkComm.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../clib/MavlinkComm.c  -o ${OBJECTDIR}/_ext/761100751/MavlinkComm.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/761100751/MavlinkComm.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -mno-eds-warn  -omf=elf -legacy-libc  -mlarge-code -mlarge-data -O0 -I"AUAV3_AND_SLUGS_SENSOR.X" -I"." -I".." -I"../mavlink/include/slugs" -I"C:/Users/sharon/Documents/GitHub/AUAV_V3_TestMavLink/../clib" -I"../clib" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/include" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/dsPIC33E/h" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/generic/h" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/PERIPH~2" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/ert" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/extern/include" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/simulink/include" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src/ext_mode/common" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src/ext_mode/common" -msmart-io=1 -Wall -msfr-warn=off   -O3 -mlarge-data -mlarge-scalar -fschedule-insns -fschedule-insns2
	@${FIXDEPS} "${OBJECTDIR}/_ext/761100751/MavlinkComm.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/761100751/adisCube16405.o: ../clib/adisCube16405.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/761100751" 
	@${RM} ${OBJECTDIR}/_ext/761100751/adisCube16405.o.d 
	@${RM} ${OBJECTDIR}/_ext/761100751/adisCube16405.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../clib/adisCube16405.c  -o ${OBJECTDIR}/_ext/761100751/adisCube16405.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/761100751/adisCube16405.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -mno-eds-warn  -omf=elf -legacy-libc  -mlarge-code -mlarge-data -O0 -I"AUAV3_AND_SLUGS_SENSOR.X" -I"." -I".." -I"../mavlink/include/slugs" -I"C:/Users/sharon/Documents/GitHub/AUAV_V3_TestMavLink/../clib" -I"../clib" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/include" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/dsPIC33E/h" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/generic/h" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/PERIPH~2" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/ert" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/extern/include" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/simulink/include" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src/ext_mode/common" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src/ext_mode/common" -msmart-io=1 -Wall -msfr-warn=off   -O3 -mlarge-data -mlarge-scalar -fschedule-insns -fschedule-insns2
	@${FIXDEPS} "${OBJECTDIR}/_ext/761100751/adisCube16405.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/761100751/apUtils.o: ../clib/apUtils.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/761100751" 
	@${RM} ${OBJECTDIR}/_ext/761100751/apUtils.o.d 
	@${RM} ${OBJECTDIR}/_ext/761100751/apUtils.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../clib/apUtils.c  -o ${OBJECTDIR}/_ext/761100751/apUtils.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/761100751/apUtils.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -mno-eds-warn  -omf=elf -legacy-libc  -mlarge-code -mlarge-data -O0 -I"AUAV3_AND_SLUGS_SENSOR.X" -I"." -I".." -I"../mavlink/include/slugs" -I"C:/Users/sharon/Documents/GitHub/AUAV_V3_TestMavLink/../clib" -I"../clib" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/include" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/dsPIC33E/h" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/generic/h" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/PERIPH~2" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/ert" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/extern/include" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/simulink/include" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src/ext_mode/common" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src/ext_mode/common" -msmart-io=1 -Wall -msfr-warn=off   -O3 -mlarge-data -mlarge-scalar -fschedule-insns -fschedule-insns2
	@${FIXDEPS} "${OBJECTDIR}/_ext/761100751/apUtils.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/761100751/circBuffer.o: ../clib/circBuffer.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/761100751" 
	@${RM} ${OBJECTDIR}/_ext/761100751/circBuffer.o.d 
	@${RM} ${OBJECTDIR}/_ext/761100751/circBuffer.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../clib/circBuffer.c  -o ${OBJECTDIR}/_ext/761100751/circBuffer.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/761100751/circBuffer.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -mno-eds-warn  -omf=elf -legacy-libc  -mlarge-code -mlarge-data -O0 -I"AUAV3_AND_SLUGS_SENSOR.X" -I"." -I".." -I"../mavlink/include/slugs" -I"C:/Users/sharon/Documents/GitHub/AUAV_V3_TestMavLink/../clib" -I"../clib" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/include" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/dsPIC33E/h" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/generic/h" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/PERIPH~2" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/ert" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/extern/include" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/simulink/include" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src/ext_mode/common" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src/ext_mode/common" -msmart-io=1 -Wall -msfr-warn=off   -O3 -mlarge-data -mlarge-scalar -fschedule-insns -fschedule-insns2
	@${FIXDEPS} "${OBJECTDIR}/_ext/761100751/circBuffer.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/761100751/gpsPort.o: ../clib/gpsPort.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/761100751" 
	@${RM} ${OBJECTDIR}/_ext/761100751/gpsPort.o.d 
	@${RM} ${OBJECTDIR}/_ext/761100751/gpsPort.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../clib/gpsPort.c  -o ${OBJECTDIR}/_ext/761100751/gpsPort.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/761100751/gpsPort.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -mno-eds-warn  -omf=elf -legacy-libc  -mlarge-code -mlarge-data -O0 -I"AUAV3_AND_SLUGS_SENSOR.X" -I"." -I".." -I"../mavlink/include/slugs" -I"C:/Users/sharon/Documents/GitHub/AUAV_V3_TestMavLink/../clib" -I"../clib" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/include" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/dsPIC33E/h" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/generic/h" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/PERIPH~2" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/ert" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/extern/include" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/simulink/include" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src/ext_mode/common" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src/ext_mode/common" -msmart-io=1 -Wall -msfr-warn=off   -O3 -mlarge-data -mlarge-scalar -fschedule-insns -fschedule-insns2
	@${FIXDEPS} "${OBJECTDIR}/_ext/761100751/gpsPort.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/761100751/gpsUblox.o: ../clib/gpsUblox.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/761100751" 
	@${RM} ${OBJECTDIR}/_ext/761100751/gpsUblox.o.d 
	@${RM} ${OBJECTDIR}/_ext/761100751/gpsUblox.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../clib/gpsUblox.c  -o ${OBJECTDIR}/_ext/761100751/gpsUblox.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/761100751/gpsUblox.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -mno-eds-warn  -omf=elf -legacy-libc  -mlarge-code -mlarge-data -O0 -I"AUAV3_AND_SLUGS_SENSOR.X" -I"." -I".." -I"../mavlink/include/slugs" -I"C:/Users/sharon/Documents/GitHub/AUAV_V3_TestMavLink/../clib" -I"../clib" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/include" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/dsPIC33E/h" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/generic/h" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/PERIPH~2" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/ert" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/extern/include" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/simulink/include" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src/ext_mode/common" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src/ext_mode/common" -msmart-io=1 -Wall -msfr-warn=off   -O3 -mlarge-data -mlarge-scalar -fschedule-insns -fschedule-insns2
	@${FIXDEPS} "${OBJECTDIR}/_ext/761100751/gpsUblox.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/761100751/hil.o: ../clib/hil.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/761100751" 
	@${RM} ${OBJECTDIR}/_ext/761100751/hil.o.d 
	@${RM} ${OBJECTDIR}/_ext/761100751/hil.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../clib/hil.c  -o ${OBJECTDIR}/_ext/761100751/hil.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/761100751/hil.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -mno-eds-warn  -omf=elf -legacy-libc  -mlarge-code -mlarge-data -O0 -I"AUAV3_AND_SLUGS_SENSOR.X" -I"." -I".." -I"../mavlink/include/slugs" -I"C:/Users/sharon/Documents/GitHub/AUAV_V3_TestMavLink/../clib" -I"../clib" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/include" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/dsPIC33E/h" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/generic/h" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/PERIPH~2" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/ert" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/extern/include" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/simulink/include" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src/ext_mode/common" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src/ext_mode/common" -msmart-io=1 -Wall -msfr-warn=off   -O3 -mlarge-data -mlarge-scalar -fschedule-insns -fschedule-insns2
	@${FIXDEPS} "${OBJECTDIR}/_ext/761100751/hil.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/761100751/mavlinkSensorMcu.o: ../clib/mavlinkSensorMcu.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/761100751" 
	@${RM} ${OBJECTDIR}/_ext/761100751/mavlinkSensorMcu.o.d 
	@${RM} ${OBJECTDIR}/_ext/761100751/mavlinkSensorMcu.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../clib/mavlinkSensorMcu.c  -o ${OBJECTDIR}/_ext/761100751/mavlinkSensorMcu.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/761100751/mavlinkSensorMcu.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -mno-eds-warn  -omf=elf -legacy-libc  -mlarge-code -mlarge-data -O0 -I"AUAV3_AND_SLUGS_SENSOR.X" -I"." -I".." -I"../mavlink/include/slugs" -I"C:/Users/sharon/Documents/GitHub/AUAV_V3_TestMavLink/../clib" -I"../clib" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/include" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/dsPIC33E/h" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/generic/h" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/PERIPH~2" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/ert" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/extern/include" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/simulink/include" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src/ext_mode/common" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src/ext_mode/common" -msmart-io=1 -Wall -msfr-warn=off   -O3 -mlarge-data -mlarge-scalar -fschedule-insns -fschedule-insns2
	@${FIXDEPS} "${OBJECTDIR}/_ext/761100751/mavlinkSensorMcu.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/761100751/novatel.o: ../clib/novatel.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/761100751" 
	@${RM} ${OBJECTDIR}/_ext/761100751/novatel.o.d 
	@${RM} ${OBJECTDIR}/_ext/761100751/novatel.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../clib/novatel.c  -o ${OBJECTDIR}/_ext/761100751/novatel.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/761100751/novatel.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -mno-eds-warn  -omf=elf -legacy-libc  -mlarge-code -mlarge-data -O0 -I"AUAV3_AND_SLUGS_SENSOR.X" -I"." -I".." -I"../mavlink/include/slugs" -I"C:/Users/sharon/Documents/GitHub/AUAV_V3_TestMavLink/../clib" -I"../clib" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/include" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/dsPIC33E/h" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/generic/h" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/PERIPH~2" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/ert" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/extern/include" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/simulink/include" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src/ext_mode/common" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src/ext_mode/common" -msmart-io=1 -Wall -msfr-warn=off   -O3 -mlarge-data -mlarge-scalar -fschedule-insns -fschedule-insns2
	@${FIXDEPS} "${OBJECTDIR}/_ext/761100751/novatel.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/rtGetInf.o: rtGetInf.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/rtGetInf.o.d 
	@${RM} ${OBJECTDIR}/rtGetInf.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  rtGetInf.c  -o ${OBJECTDIR}/rtGetInf.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/rtGetInf.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -mno-eds-warn  -omf=elf -legacy-libc  -mlarge-code -mlarge-data -O0 -I"AUAV3_AND_SLUGS_SENSOR.X" -I"." -I".." -I"../mavlink/include/slugs" -I"C:/Users/sharon/Documents/GitHub/AUAV_V3_TestMavLink/../clib" -I"../clib" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/include" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/dsPIC33E/h" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/generic/h" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/PERIPH~2" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/ert" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/extern/include" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/simulink/include" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src/ext_mode/common" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src/ext_mode/common" -msmart-io=1 -Wall -msfr-warn=off   -O3 -mlarge-data -mlarge-scalar -fschedule-insns -fschedule-insns2
	@${FIXDEPS} "${OBJECTDIR}/rtGetInf.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/rtGetNaN.o: rtGetNaN.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/rtGetNaN.o.d 
	@${RM} ${OBJECTDIR}/rtGetNaN.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  rtGetNaN.c  -o ${OBJECTDIR}/rtGetNaN.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/rtGetNaN.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -mno-eds-warn  -omf=elf -legacy-libc  -mlarge-code -mlarge-data -O0 -I"AUAV3_AND_SLUGS_SENSOR.X" -I"." -I".." -I"../mavlink/include/slugs" -I"C:/Users/sharon/Documents/GitHub/AUAV_V3_TestMavLink/../clib" -I"../clib" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/include" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/dsPIC33E/h" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/generic/h" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/PERIPH~2" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/ert" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/extern/include" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/simulink/include" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src/ext_mode/common" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src/ext_mode/common" -msmart-io=1 -Wall -msfr-warn=off   -O3 -mlarge-data -mlarge-scalar -fschedule-insns -fschedule-insns2
	@${FIXDEPS} "${OBJECTDIR}/rtGetNaN.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/rt_nonfinite.o: rt_nonfinite.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/rt_nonfinite.o.d 
	@${RM} ${OBJECTDIR}/rt_nonfinite.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  rt_nonfinite.c  -o ${OBJECTDIR}/rt_nonfinite.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/rt_nonfinite.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -mno-eds-warn  -omf=elf -legacy-libc  -mlarge-code -mlarge-data -O0 -I"AUAV3_AND_SLUGS_SENSOR.X" -I"." -I".." -I"../mavlink/include/slugs" -I"C:/Users/sharon/Documents/GitHub/AUAV_V3_TestMavLink/../clib" -I"../clib" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/include" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/dsPIC33E/h" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/generic/h" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/PERIPH~2" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/ert" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/extern/include" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/simulink/include" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src/ext_mode/common" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src/ext_mode/common" -msmart-io=1 -Wall -msfr-warn=off   -O3 -mlarge-data -mlarge-scalar -fschedule-insns -fschedule-insns2
	@${FIXDEPS} "${OBJECTDIR}/rt_nonfinite.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/761100751/updateSensorMcuState.o: ../clib/updateSensorMcuState.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/761100751" 
	@${RM} ${OBJECTDIR}/_ext/761100751/updateSensorMcuState.o.d 
	@${RM} ${OBJECTDIR}/_ext/761100751/updateSensorMcuState.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../clib/updateSensorMcuState.c  -o ${OBJECTDIR}/_ext/761100751/updateSensorMcuState.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/761100751/updateSensorMcuState.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -mno-eds-warn  -omf=elf -legacy-libc  -mlarge-code -mlarge-data -O0 -I"AUAV3_AND_SLUGS_SENSOR.X" -I"." -I".." -I"../mavlink/include/slugs" -I"C:/Users/sharon/Documents/GitHub/AUAV_V3_TestMavLink/../clib" -I"../clib" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/include" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/dsPIC33E/h" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/generic/h" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/PERIPH~2" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/ert" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/extern/include" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/simulink/include" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src/ext_mode/common" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src/ext_mode/common" -msmart-io=1 -Wall -msfr-warn=off   -O3 -mlarge-data -mlarge-scalar -fschedule-insns -fschedule-insns2
	@${FIXDEPS} "${OBJECTDIR}/_ext/761100751/updateSensorMcuState.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
else
${OBJECTDIR}/AUAV3_AND_SLUGS_SENSOR.o: AUAV3_AND_SLUGS_SENSOR.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/AUAV3_AND_SLUGS_SENSOR.o.d 
	@${RM} ${OBJECTDIR}/AUAV3_AND_SLUGS_SENSOR.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  AUAV3_AND_SLUGS_SENSOR.c  -o ${OBJECTDIR}/AUAV3_AND_SLUGS_SENSOR.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/AUAV3_AND_SLUGS_SENSOR.o.d"      -mno-eds-warn  -omf=elf -legacy-libc  -mlarge-code -mlarge-data -O0 -I"AUAV3_AND_SLUGS_SENSOR.X" -I"." -I".." -I"../mavlink/include/slugs" -I"C:/Users/sharon/Documents/GitHub/AUAV_V3_TestMavLink/../clib" -I"../clib" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/include" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/dsPIC33E/h" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/generic/h" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/PERIPH~2" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/ert" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/extern/include" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/simulink/include" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src/ext_mode/common" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src/ext_mode/common" -msmart-io=1 -Wall -msfr-warn=off   -O3 -mlarge-data -mlarge-scalar -fschedule-insns -fschedule-insns2
	@${FIXDEPS} "${OBJECTDIR}/AUAV3_AND_SLUGS_SENSOR.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/AUAV3_AND_SLUGS_SENSOR_data.o: AUAV3_AND_SLUGS_SENSOR_data.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/AUAV3_AND_SLUGS_SENSOR_data.o.d 
	@${RM} ${OBJECTDIR}/AUAV3_AND_SLUGS_SENSOR_data.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  AUAV3_AND_SLUGS_SENSOR_data.c  -o ${OBJECTDIR}/AUAV3_AND_SLUGS_SENSOR_data.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/AUAV3_AND_SLUGS_SENSOR_data.o.d"      -mno-eds-warn  -omf=elf -legacy-libc  -mlarge-code -mlarge-data -O0 -I"AUAV3_AND_SLUGS_SENSOR.X" -I"." -I".." -I"../mavlink/include/slugs" -I"C:/Users/sharon/Documents/GitHub/AUAV_V3_TestMavLink/../clib" -I"../clib" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/include" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/dsPIC33E/h" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/generic/h" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/PERIPH~2" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/ert" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/extern/include" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/simulink/include" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src/ext_mode/common" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src/ext_mode/common" -msmart-io=1 -Wall -msfr-warn=off   -O3 -mlarge-data -mlarge-scalar -fschedule-insns -fschedule-insns2
	@${FIXDEPS} "${OBJECTDIR}/AUAV3_AND_SLUGS_SENSOR_data.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/AUAV3_AND_SLUGS_SENSOR_main.o: AUAV3_AND_SLUGS_SENSOR_main.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/AUAV3_AND_SLUGS_SENSOR_main.o.d 
	@${RM} ${OBJECTDIR}/AUAV3_AND_SLUGS_SENSOR_main.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  AUAV3_AND_SLUGS_SENSOR_main.c  -o ${OBJECTDIR}/AUAV3_AND_SLUGS_SENSOR_main.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/AUAV3_AND_SLUGS_SENSOR_main.o.d"      -mno-eds-warn  -omf=elf -legacy-libc  -mlarge-code -mlarge-data -O0 -I"AUAV3_AND_SLUGS_SENSOR.X" -I"." -I".." -I"../mavlink/include/slugs" -I"C:/Users/sharon/Documents/GitHub/AUAV_V3_TestMavLink/../clib" -I"../clib" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/include" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/dsPIC33E/h" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/generic/h" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/PERIPH~2" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/ert" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/extern/include" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/simulink/include" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src/ext_mode/common" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src/ext_mode/common" -msmart-io=1 -Wall -msfr-warn=off   -O3 -mlarge-data -mlarge-scalar -fschedule-insns -fschedule-insns2
	@${FIXDEPS} "${OBJECTDIR}/AUAV3_AND_SLUGS_SENSOR_main.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/MCHP_I2C2_Interrupt.o: MCHP_I2C2_Interrupt.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/MCHP_I2C2_Interrupt.o.d 
	@${RM} ${OBJECTDIR}/MCHP_I2C2_Interrupt.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  MCHP_I2C2_Interrupt.c  -o ${OBJECTDIR}/MCHP_I2C2_Interrupt.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/MCHP_I2C2_Interrupt.o.d"      -mno-eds-warn  -omf=elf -legacy-libc  -mlarge-code -mlarge-data -O0 -I"AUAV3_AND_SLUGS_SENSOR.X" -I"." -I".." -I"../mavlink/include/slugs" -I"C:/Users/sharon/Documents/GitHub/AUAV_V3_TestMavLink/../clib" -I"../clib" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/include" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/dsPIC33E/h" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/generic/h" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/PERIPH~2" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/ert" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/extern/include" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/simulink/include" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src/ext_mode/common" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src/ext_mode/common" -msmart-io=1 -Wall -msfr-warn=off   -O3 -mlarge-data -mlarge-scalar -fschedule-insns -fschedule-insns2
	@${FIXDEPS} "${OBJECTDIR}/MCHP_I2C2_Interrupt.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/MCHP_I2C2_Interrupt_data.o: MCHP_I2C2_Interrupt_data.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/MCHP_I2C2_Interrupt_data.o.d 
	@${RM} ${OBJECTDIR}/MCHP_I2C2_Interrupt_data.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  MCHP_I2C2_Interrupt_data.c  -o ${OBJECTDIR}/MCHP_I2C2_Interrupt_data.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/MCHP_I2C2_Interrupt_data.o.d"      -mno-eds-warn  -omf=elf -legacy-libc  -mlarge-code -mlarge-data -O0 -I"AUAV3_AND_SLUGS_SENSOR.X" -I"." -I".." -I"../mavlink/include/slugs" -I"C:/Users/sharon/Documents/GitHub/AUAV_V3_TestMavLink/../clib" -I"../clib" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/include" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/dsPIC33E/h" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/generic/h" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/PERIPH~2" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/ert" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/extern/include" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/simulink/include" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src/ext_mode/common" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src/ext_mode/common" -msmart-io=1 -Wall -msfr-warn=off   -O3 -mlarge-data -mlarge-scalar -fschedule-insns -fschedule-insns2
	@${FIXDEPS} "${OBJECTDIR}/MCHP_I2C2_Interrupt_data.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/MCHP_IC_Interrupt.o: MCHP_IC_Interrupt.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/MCHP_IC_Interrupt.o.d 
	@${RM} ${OBJECTDIR}/MCHP_IC_Interrupt.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  MCHP_IC_Interrupt.c  -o ${OBJECTDIR}/MCHP_IC_Interrupt.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/MCHP_IC_Interrupt.o.d"      -mno-eds-warn  -omf=elf -legacy-libc  -mlarge-code -mlarge-data -O0 -I"AUAV3_AND_SLUGS_SENSOR.X" -I"." -I".." -I"../mavlink/include/slugs" -I"C:/Users/sharon/Documents/GitHub/AUAV_V3_TestMavLink/../clib" -I"../clib" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/include" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/dsPIC33E/h" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/generic/h" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/PERIPH~2" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/ert" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/extern/include" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/simulink/include" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src/ext_mode/common" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src/ext_mode/common" -msmart-io=1 -Wall -msfr-warn=off   -O3 -mlarge-data -mlarge-scalar -fschedule-insns -fschedule-insns2
	@${FIXDEPS} "${OBJECTDIR}/MCHP_IC_Interrupt.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/MCHP_SPI1_Interrupt.o: MCHP_SPI1_Interrupt.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/MCHP_SPI1_Interrupt.o.d 
	@${RM} ${OBJECTDIR}/MCHP_SPI1_Interrupt.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  MCHP_SPI1_Interrupt.c  -o ${OBJECTDIR}/MCHP_SPI1_Interrupt.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/MCHP_SPI1_Interrupt.o.d"      -mno-eds-warn  -omf=elf -legacy-libc  -mlarge-code -mlarge-data -O0 -I"AUAV3_AND_SLUGS_SENSOR.X" -I"." -I".." -I"../mavlink/include/slugs" -I"C:/Users/sharon/Documents/GitHub/AUAV_V3_TestMavLink/../clib" -I"../clib" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/include" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/dsPIC33E/h" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/generic/h" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/PERIPH~2" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/ert" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/extern/include" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/simulink/include" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src/ext_mode/common" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src/ext_mode/common" -msmart-io=1 -Wall -msfr-warn=off   -O3 -mlarge-data -mlarge-scalar -fschedule-insns -fschedule-insns2
	@${FIXDEPS} "${OBJECTDIR}/MCHP_SPI1_Interrupt.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/MCHP_SPI1_Interrupt_data.o: MCHP_SPI1_Interrupt_data.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/MCHP_SPI1_Interrupt_data.o.d 
	@${RM} ${OBJECTDIR}/MCHP_SPI1_Interrupt_data.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  MCHP_SPI1_Interrupt_data.c  -o ${OBJECTDIR}/MCHP_SPI1_Interrupt_data.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/MCHP_SPI1_Interrupt_data.o.d"      -mno-eds-warn  -omf=elf -legacy-libc  -mlarge-code -mlarge-data -O0 -I"AUAV3_AND_SLUGS_SENSOR.X" -I"." -I".." -I"../mavlink/include/slugs" -I"C:/Users/sharon/Documents/GitHub/AUAV_V3_TestMavLink/../clib" -I"../clib" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/include" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/dsPIC33E/h" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/generic/h" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/PERIPH~2" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/ert" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/extern/include" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/simulink/include" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src/ext_mode/common" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src/ext_mode/common" -msmart-io=1 -Wall -msfr-warn=off   -O3 -mlarge-data -mlarge-scalar -fschedule-insns -fschedule-insns2
	@${FIXDEPS} "${OBJECTDIR}/MCHP_SPI1_Interrupt_data.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/MCHP_UART1_Interrupt.o: MCHP_UART1_Interrupt.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/MCHP_UART1_Interrupt.o.d 
	@${RM} ${OBJECTDIR}/MCHP_UART1_Interrupt.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  MCHP_UART1_Interrupt.c  -o ${OBJECTDIR}/MCHP_UART1_Interrupt.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/MCHP_UART1_Interrupt.o.d"      -mno-eds-warn  -omf=elf -legacy-libc  -mlarge-code -mlarge-data -O0 -I"AUAV3_AND_SLUGS_SENSOR.X" -I"." -I".." -I"../mavlink/include/slugs" -I"C:/Users/sharon/Documents/GitHub/AUAV_V3_TestMavLink/../clib" -I"../clib" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/include" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/dsPIC33E/h" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/generic/h" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/PERIPH~2" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/ert" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/extern/include" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/simulink/include" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src/ext_mode/common" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src/ext_mode/common" -msmart-io=1 -Wall -msfr-warn=off   -O3 -mlarge-data -mlarge-scalar -fschedule-insns -fschedule-insns2
	@${FIXDEPS} "${OBJECTDIR}/MCHP_UART1_Interrupt.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/MCHP_UART4_Interrupt.o: MCHP_UART4_Interrupt.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/MCHP_UART4_Interrupt.o.d 
	@${RM} ${OBJECTDIR}/MCHP_UART4_Interrupt.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  MCHP_UART4_Interrupt.c  -o ${OBJECTDIR}/MCHP_UART4_Interrupt.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/MCHP_UART4_Interrupt.o.d"      -mno-eds-warn  -omf=elf -legacy-libc  -mlarge-code -mlarge-data -O0 -I"AUAV3_AND_SLUGS_SENSOR.X" -I"." -I".." -I"../mavlink/include/slugs" -I"C:/Users/sharon/Documents/GitHub/AUAV_V3_TestMavLink/../clib" -I"../clib" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/include" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/dsPIC33E/h" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/generic/h" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/PERIPH~2" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/ert" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/extern/include" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/simulink/include" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src/ext_mode/common" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src/ext_mode/common" -msmart-io=1 -Wall -msfr-warn=off   -O3 -mlarge-data -mlarge-scalar -fschedule-insns -fschedule-insns2
	@${FIXDEPS} "${OBJECTDIR}/MCHP_UART4_Interrupt.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/761100751/MavlinkComm.o: ../clib/MavlinkComm.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/761100751" 
	@${RM} ${OBJECTDIR}/_ext/761100751/MavlinkComm.o.d 
	@${RM} ${OBJECTDIR}/_ext/761100751/MavlinkComm.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../clib/MavlinkComm.c  -o ${OBJECTDIR}/_ext/761100751/MavlinkComm.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/761100751/MavlinkComm.o.d"      -mno-eds-warn  -omf=elf -legacy-libc  -mlarge-code -mlarge-data -O0 -I"AUAV3_AND_SLUGS_SENSOR.X" -I"." -I".." -I"../mavlink/include/slugs" -I"C:/Users/sharon/Documents/GitHub/AUAV_V3_TestMavLink/../clib" -I"../clib" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/include" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/dsPIC33E/h" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/generic/h" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/PERIPH~2" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/ert" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/extern/include" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/simulink/include" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src/ext_mode/common" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src/ext_mode/common" -msmart-io=1 -Wall -msfr-warn=off   -O3 -mlarge-data -mlarge-scalar -fschedule-insns -fschedule-insns2
	@${FIXDEPS} "${OBJECTDIR}/_ext/761100751/MavlinkComm.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/761100751/adisCube16405.o: ../clib/adisCube16405.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/761100751" 
	@${RM} ${OBJECTDIR}/_ext/761100751/adisCube16405.o.d 
	@${RM} ${OBJECTDIR}/_ext/761100751/adisCube16405.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../clib/adisCube16405.c  -o ${OBJECTDIR}/_ext/761100751/adisCube16405.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/761100751/adisCube16405.o.d"      -mno-eds-warn  -omf=elf -legacy-libc  -mlarge-code -mlarge-data -O0 -I"AUAV3_AND_SLUGS_SENSOR.X" -I"." -I".." -I"../mavlink/include/slugs" -I"C:/Users/sharon/Documents/GitHub/AUAV_V3_TestMavLink/../clib" -I"../clib" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/include" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/dsPIC33E/h" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/generic/h" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/PERIPH~2" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/ert" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/extern/include" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/simulink/include" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src/ext_mode/common" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src/ext_mode/common" -msmart-io=1 -Wall -msfr-warn=off   -O3 -mlarge-data -mlarge-scalar -fschedule-insns -fschedule-insns2
	@${FIXDEPS} "${OBJECTDIR}/_ext/761100751/adisCube16405.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/761100751/apUtils.o: ../clib/apUtils.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/761100751" 
	@${RM} ${OBJECTDIR}/_ext/761100751/apUtils.o.d 
	@${RM} ${OBJECTDIR}/_ext/761100751/apUtils.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../clib/apUtils.c  -o ${OBJECTDIR}/_ext/761100751/apUtils.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/761100751/apUtils.o.d"      -mno-eds-warn  -omf=elf -legacy-libc  -mlarge-code -mlarge-data -O0 -I"AUAV3_AND_SLUGS_SENSOR.X" -I"." -I".." -I"../mavlink/include/slugs" -I"C:/Users/sharon/Documents/GitHub/AUAV_V3_TestMavLink/../clib" -I"../clib" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/include" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/dsPIC33E/h" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/generic/h" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/PERIPH~2" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/ert" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/extern/include" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/simulink/include" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src/ext_mode/common" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src/ext_mode/common" -msmart-io=1 -Wall -msfr-warn=off   -O3 -mlarge-data -mlarge-scalar -fschedule-insns -fschedule-insns2
	@${FIXDEPS} "${OBJECTDIR}/_ext/761100751/apUtils.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/761100751/circBuffer.o: ../clib/circBuffer.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/761100751" 
	@${RM} ${OBJECTDIR}/_ext/761100751/circBuffer.o.d 
	@${RM} ${OBJECTDIR}/_ext/761100751/circBuffer.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../clib/circBuffer.c  -o ${OBJECTDIR}/_ext/761100751/circBuffer.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/761100751/circBuffer.o.d"      -mno-eds-warn  -omf=elf -legacy-libc  -mlarge-code -mlarge-data -O0 -I"AUAV3_AND_SLUGS_SENSOR.X" -I"." -I".." -I"../mavlink/include/slugs" -I"C:/Users/sharon/Documents/GitHub/AUAV_V3_TestMavLink/../clib" -I"../clib" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/include" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/dsPIC33E/h" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/generic/h" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/PERIPH~2" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/ert" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/extern/include" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/simulink/include" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src/ext_mode/common" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src/ext_mode/common" -msmart-io=1 -Wall -msfr-warn=off   -O3 -mlarge-data -mlarge-scalar -fschedule-insns -fschedule-insns2
	@${FIXDEPS} "${OBJECTDIR}/_ext/761100751/circBuffer.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/761100751/gpsPort.o: ../clib/gpsPort.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/761100751" 
	@${RM} ${OBJECTDIR}/_ext/761100751/gpsPort.o.d 
	@${RM} ${OBJECTDIR}/_ext/761100751/gpsPort.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../clib/gpsPort.c  -o ${OBJECTDIR}/_ext/761100751/gpsPort.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/761100751/gpsPort.o.d"      -mno-eds-warn  -omf=elf -legacy-libc  -mlarge-code -mlarge-data -O0 -I"AUAV3_AND_SLUGS_SENSOR.X" -I"." -I".." -I"../mavlink/include/slugs" -I"C:/Users/sharon/Documents/GitHub/AUAV_V3_TestMavLink/../clib" -I"../clib" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/include" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/dsPIC33E/h" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/generic/h" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/PERIPH~2" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/ert" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/extern/include" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/simulink/include" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src/ext_mode/common" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src/ext_mode/common" -msmart-io=1 -Wall -msfr-warn=off   -O3 -mlarge-data -mlarge-scalar -fschedule-insns -fschedule-insns2
	@${FIXDEPS} "${OBJECTDIR}/_ext/761100751/gpsPort.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/761100751/gpsUblox.o: ../clib/gpsUblox.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/761100751" 
	@${RM} ${OBJECTDIR}/_ext/761100751/gpsUblox.o.d 
	@${RM} ${OBJECTDIR}/_ext/761100751/gpsUblox.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../clib/gpsUblox.c  -o ${OBJECTDIR}/_ext/761100751/gpsUblox.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/761100751/gpsUblox.o.d"      -mno-eds-warn  -omf=elf -legacy-libc  -mlarge-code -mlarge-data -O0 -I"AUAV3_AND_SLUGS_SENSOR.X" -I"." -I".." -I"../mavlink/include/slugs" -I"C:/Users/sharon/Documents/GitHub/AUAV_V3_TestMavLink/../clib" -I"../clib" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/include" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/dsPIC33E/h" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/generic/h" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/PERIPH~2" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/ert" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/extern/include" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/simulink/include" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src/ext_mode/common" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src/ext_mode/common" -msmart-io=1 -Wall -msfr-warn=off   -O3 -mlarge-data -mlarge-scalar -fschedule-insns -fschedule-insns2
	@${FIXDEPS} "${OBJECTDIR}/_ext/761100751/gpsUblox.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/761100751/hil.o: ../clib/hil.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/761100751" 
	@${RM} ${OBJECTDIR}/_ext/761100751/hil.o.d 
	@${RM} ${OBJECTDIR}/_ext/761100751/hil.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../clib/hil.c  -o ${OBJECTDIR}/_ext/761100751/hil.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/761100751/hil.o.d"      -mno-eds-warn  -omf=elf -legacy-libc  -mlarge-code -mlarge-data -O0 -I"AUAV3_AND_SLUGS_SENSOR.X" -I"." -I".." -I"../mavlink/include/slugs" -I"C:/Users/sharon/Documents/GitHub/AUAV_V3_TestMavLink/../clib" -I"../clib" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/include" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/dsPIC33E/h" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/generic/h" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/PERIPH~2" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/ert" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/extern/include" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/simulink/include" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src/ext_mode/common" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src/ext_mode/common" -msmart-io=1 -Wall -msfr-warn=off   -O3 -mlarge-data -mlarge-scalar -fschedule-insns -fschedule-insns2
	@${FIXDEPS} "${OBJECTDIR}/_ext/761100751/hil.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/761100751/mavlinkSensorMcu.o: ../clib/mavlinkSensorMcu.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/761100751" 
	@${RM} ${OBJECTDIR}/_ext/761100751/mavlinkSensorMcu.o.d 
	@${RM} ${OBJECTDIR}/_ext/761100751/mavlinkSensorMcu.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../clib/mavlinkSensorMcu.c  -o ${OBJECTDIR}/_ext/761100751/mavlinkSensorMcu.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/761100751/mavlinkSensorMcu.o.d"      -mno-eds-warn  -omf=elf -legacy-libc  -mlarge-code -mlarge-data -O0 -I"AUAV3_AND_SLUGS_SENSOR.X" -I"." -I".." -I"../mavlink/include/slugs" -I"C:/Users/sharon/Documents/GitHub/AUAV_V3_TestMavLink/../clib" -I"../clib" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/include" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/dsPIC33E/h" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/generic/h" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/PERIPH~2" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/ert" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/extern/include" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/simulink/include" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src/ext_mode/common" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src/ext_mode/common" -msmart-io=1 -Wall -msfr-warn=off   -O3 -mlarge-data -mlarge-scalar -fschedule-insns -fschedule-insns2
	@${FIXDEPS} "${OBJECTDIR}/_ext/761100751/mavlinkSensorMcu.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/761100751/novatel.o: ../clib/novatel.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/761100751" 
	@${RM} ${OBJECTDIR}/_ext/761100751/novatel.o.d 
	@${RM} ${OBJECTDIR}/_ext/761100751/novatel.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../clib/novatel.c  -o ${OBJECTDIR}/_ext/761100751/novatel.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/761100751/novatel.o.d"      -mno-eds-warn  -omf=elf -legacy-libc  -mlarge-code -mlarge-data -O0 -I"AUAV3_AND_SLUGS_SENSOR.X" -I"." -I".." -I"../mavlink/include/slugs" -I"C:/Users/sharon/Documents/GitHub/AUAV_V3_TestMavLink/../clib" -I"../clib" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/include" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/dsPIC33E/h" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/generic/h" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/PERIPH~2" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/ert" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/extern/include" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/simulink/include" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src/ext_mode/common" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src/ext_mode/common" -msmart-io=1 -Wall -msfr-warn=off   -O3 -mlarge-data -mlarge-scalar -fschedule-insns -fschedule-insns2
	@${FIXDEPS} "${OBJECTDIR}/_ext/761100751/novatel.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/rtGetInf.o: rtGetInf.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/rtGetInf.o.d 
	@${RM} ${OBJECTDIR}/rtGetInf.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  rtGetInf.c  -o ${OBJECTDIR}/rtGetInf.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/rtGetInf.o.d"      -mno-eds-warn  -omf=elf -legacy-libc  -mlarge-code -mlarge-data -O0 -I"AUAV3_AND_SLUGS_SENSOR.X" -I"." -I".." -I"../mavlink/include/slugs" -I"C:/Users/sharon/Documents/GitHub/AUAV_V3_TestMavLink/../clib" -I"../clib" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/include" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/dsPIC33E/h" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/generic/h" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/PERIPH~2" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/ert" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/extern/include" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/simulink/include" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src/ext_mode/common" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src/ext_mode/common" -msmart-io=1 -Wall -msfr-warn=off   -O3 -mlarge-data -mlarge-scalar -fschedule-insns -fschedule-insns2
	@${FIXDEPS} "${OBJECTDIR}/rtGetInf.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/rtGetNaN.o: rtGetNaN.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/rtGetNaN.o.d 
	@${RM} ${OBJECTDIR}/rtGetNaN.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  rtGetNaN.c  -o ${OBJECTDIR}/rtGetNaN.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/rtGetNaN.o.d"      -mno-eds-warn  -omf=elf -legacy-libc  -mlarge-code -mlarge-data -O0 -I"AUAV3_AND_SLUGS_SENSOR.X" -I"." -I".." -I"../mavlink/include/slugs" -I"C:/Users/sharon/Documents/GitHub/AUAV_V3_TestMavLink/../clib" -I"../clib" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/include" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/dsPIC33E/h" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/generic/h" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/PERIPH~2" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/ert" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/extern/include" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/simulink/include" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src/ext_mode/common" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src/ext_mode/common" -msmart-io=1 -Wall -msfr-warn=off   -O3 -mlarge-data -mlarge-scalar -fschedule-insns -fschedule-insns2
	@${FIXDEPS} "${OBJECTDIR}/rtGetNaN.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/rt_nonfinite.o: rt_nonfinite.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/rt_nonfinite.o.d 
	@${RM} ${OBJECTDIR}/rt_nonfinite.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  rt_nonfinite.c  -o ${OBJECTDIR}/rt_nonfinite.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/rt_nonfinite.o.d"      -mno-eds-warn  -omf=elf -legacy-libc  -mlarge-code -mlarge-data -O0 -I"AUAV3_AND_SLUGS_SENSOR.X" -I"." -I".." -I"../mavlink/include/slugs" -I"C:/Users/sharon/Documents/GitHub/AUAV_V3_TestMavLink/../clib" -I"../clib" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/include" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/dsPIC33E/h" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/generic/h" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/PERIPH~2" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/ert" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/extern/include" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/simulink/include" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src/ext_mode/common" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src/ext_mode/common" -msmart-io=1 -Wall -msfr-warn=off   -O3 -mlarge-data -mlarge-scalar -fschedule-insns -fschedule-insns2
	@${FIXDEPS} "${OBJECTDIR}/rt_nonfinite.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/761100751/updateSensorMcuState.o: ../clib/updateSensorMcuState.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/761100751" 
	@${RM} ${OBJECTDIR}/_ext/761100751/updateSensorMcuState.o.d 
	@${RM} ${OBJECTDIR}/_ext/761100751/updateSensorMcuState.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../clib/updateSensorMcuState.c  -o ${OBJECTDIR}/_ext/761100751/updateSensorMcuState.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/761100751/updateSensorMcuState.o.d"      -mno-eds-warn  -omf=elf -legacy-libc  -mlarge-code -mlarge-data -O0 -I"AUAV3_AND_SLUGS_SENSOR.X" -I"." -I".." -I"../mavlink/include/slugs" -I"C:/Users/sharon/Documents/GitHub/AUAV_V3_TestMavLink/../clib" -I"../clib" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/include" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/dsPIC33E/h" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/generic/h" -I"C:/PROGRA~2/MICROC~1/xc16/v1.26/support/PERIPH~2" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/ert" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/extern/include" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/simulink/include" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src/ext_mode/common" -I"C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src/ext_mode/common" -msmart-io=1 -Wall -msfr-warn=off   -O3 -mlarge-data -mlarge-scalar -fschedule-insns -fschedule-insns2
	@${FIXDEPS} "${OBJECTDIR}/_ext/761100751/updateSensorMcuState.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
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
dist/${CND_CONF}/${IMAGE_TYPE}/AUAV3_AND_SLUGS_SENSOR.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk  C:/PROGRA~2/MICROC~1/xc16/v1.26/lib/dsPIC33E/libp33EP512MU810-elf.a C:/PROGRA~2/MICROC~1/xc16/v1.26/lib/libpic30-elf.a C:/PROGRA~2/MICROC~1/xc16/v1.26/lib/libm-elf.a C:/PROGRA~2/MICROC~1/xc16/v1.26/lib/libc-elf.a C:/PROGRA~2/MICROC~1/xc16/v1.26/lib/libq-elf.a C:/PROGRA~2/MICROC~1/xc16/v1.26/lib/libq-dsp-elf.a  
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -o dist/${CND_CONF}/${IMAGE_TYPE}/AUAV3_AND_SLUGS_SENSOR.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}  ${OBJECTFILES_QUOTED_IF_SPACED}    C:\PROGRA~2\MICROC~1\xc16\v1.26\lib\dsPIC33E\libp33EP512MU810-elf.a C:\PROGRA~2\MICROC~1\xc16\v1.26\lib\libpic30-elf.a C:\PROGRA~2\MICROC~1\xc16\v1.26\lib\libm-elf.a C:\PROGRA~2\MICROC~1\xc16\v1.26\lib\libc-elf.a C:\PROGRA~2\MICROC~1\xc16\v1.26\lib\libq-elf.a C:\PROGRA~2\MICROC~1\xc16\v1.26\lib\libq-dsp-elf.a  -mcpu=$(MP_PROCESSOR_OPTION)        -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -omf=elf -legacy-libc   -mreserve=data@0x1000:0x101B -mreserve=data@0x101C:0x101D -mreserve=data@0x101E:0x101F -mreserve=data@0x1020:0x1021 -mreserve=data@0x1022:0x1023 -mreserve=data@0x1024:0x1027 -mreserve=data@0x1028:0x104F   -Wl,--local-stack,--defsym=__MPLAB_BUILD=1,--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1,--defsym=__MPLAB_DEBUGGER_PK3=1,$(MP_LINKER_FILE_OPTION),--stack=16,--check-sections,--data-init,--pack-data,--handles,--isr,--no-gc-sections,--fill-upper=0,--stackguard=16,--library-path="AUAV3_AND_SLUGS_SENSOR.X",--library-path=".",--library-path="..",--library-path="../mavlink/include/slugs",--library-path="C:/Users/sharon/Documents/GitHub/AUAV_V3_TestMavLink/../clib",--library-path="../clib",--library-path="C:/PROGRA~2/MICROC~1/xc16/v1.26/include",--library-path="C:/PROGRA~2/MICROC~1/xc16/v1.26/support/dsPIC33E/h",--library-path="C:/PROGRA~2/MICROC~1/xc16/v1.26/support/generic/h",--library-path="C:/PROGRA~2/MICROC~1/xc16/v1.26/support/PERIPH~2",--library-path="C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/ert",--library-path="C:/Program Files/MATLAB/MATLAB Production Server/R2015a/extern/include",--library-path="C:/Program Files/MATLAB/MATLAB Production Server/R2015a/simulink/include",--library-path="C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src",--library-path="C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src/ext_mode/common",--library-path="C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src/ext_mode/common",--no-force-link,--smart-io,-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map",--report-mem,--memorysummary,dist/${CND_CONF}/${IMAGE_TYPE}/memoryfile.xml$(MP_EXTRA_LD_POST) 
	
else
dist/${CND_CONF}/${IMAGE_TYPE}/AUAV3_AND_SLUGS_SENSOR.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk  C:/PROGRA~2/MICROC~1/xc16/v1.26/lib/dsPIC33E/libp33EP512MU810-elf.a C:/PROGRA~2/MICROC~1/xc16/v1.26/lib/libpic30-elf.a C:/PROGRA~2/MICROC~1/xc16/v1.26/lib/libm-elf.a C:/PROGRA~2/MICROC~1/xc16/v1.26/lib/libc-elf.a C:/PROGRA~2/MICROC~1/xc16/v1.26/lib/libq-elf.a C:/PROGRA~2/MICROC~1/xc16/v1.26/lib/libq-dsp-elf.a 
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -o dist/${CND_CONF}/${IMAGE_TYPE}/AUAV3_AND_SLUGS_SENSOR.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX}  ${OBJECTFILES_QUOTED_IF_SPACED}    C:\PROGRA~2\MICROC~1\xc16\v1.26\lib\dsPIC33E\libp33EP512MU810-elf.a C:\PROGRA~2\MICROC~1\xc16\v1.26\lib\libpic30-elf.a C:\PROGRA~2\MICROC~1\xc16\v1.26\lib\libm-elf.a C:\PROGRA~2\MICROC~1\xc16\v1.26\lib\libc-elf.a C:\PROGRA~2\MICROC~1\xc16\v1.26\lib\libq-elf.a C:\PROGRA~2\MICROC~1\xc16\v1.26\lib\libq-dsp-elf.a  -mcpu=$(MP_PROCESSOR_OPTION)        -omf=elf -legacy-libc  -Wl,--local-stack,--defsym=__MPLAB_BUILD=1,$(MP_LINKER_FILE_OPTION),--stack=16,--check-sections,--data-init,--pack-data,--handles,--isr,--no-gc-sections,--fill-upper=0,--stackguard=16,--library-path="AUAV3_AND_SLUGS_SENSOR.X",--library-path=".",--library-path="..",--library-path="../mavlink/include/slugs",--library-path="C:/Users/sharon/Documents/GitHub/AUAV_V3_TestMavLink/../clib",--library-path="../clib",--library-path="C:/PROGRA~2/MICROC~1/xc16/v1.26/include",--library-path="C:/PROGRA~2/MICROC~1/xc16/v1.26/support/dsPIC33E/h",--library-path="C:/PROGRA~2/MICROC~1/xc16/v1.26/support/generic/h",--library-path="C:/PROGRA~2/MICROC~1/xc16/v1.26/support/PERIPH~2",--library-path="C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/ert",--library-path="C:/Program Files/MATLAB/MATLAB Production Server/R2015a/extern/include",--library-path="C:/Program Files/MATLAB/MATLAB Production Server/R2015a/simulink/include",--library-path="C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src",--library-path="C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src/ext_mode/common",--library-path="C:/Program Files/MATLAB/MATLAB Production Server/R2015a/rtw/c/src/ext_mode/common",--no-force-link,--smart-io,-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map",--report-mem,--memorysummary,dist/${CND_CONF}/${IMAGE_TYPE}/memoryfile.xml$(MP_EXTRA_LD_POST) 
	${MP_CC_DIR}\\xc16-bin2hex dist/${CND_CONF}/${IMAGE_TYPE}/AUAV3_AND_SLUGS_SENSOR.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} -a  -omf=elf  
	
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
