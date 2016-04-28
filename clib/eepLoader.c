/*
The MIT License

Copyright (c) 2009 UCSC Autonomous Systems Lab

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.

*/

#include "eepLoader.h"

unsigned char EEPInit(void){
	unsigned char eepInitMsg = 0;
	
	// Initialize the EEPROM emulation and read the PID Data
	eepInitMsg = DataEEInit();
	
	if (eepInitMsg == 1){
                /*
		mlActionAck.action = SLUGS_ACTION_EEPROM; // EEPROM Action
		mlActionAck.result = EEP_PAGE_EXP; // Page Expired
                */
            mlPending.statustext++;

            mlStatustext.severity = MAV_SEVERITY_ERROR;
            strncat(mlStatustext.text, "Page expired while initializing EEPROM.", 49);
            //mlStatustext.text = "Page expired while initializing EEPROM.";
        } else if (eepInitMsg == 6){
            /*
		mlActionAck.action = SLUGS_ACTION_EEPROM; // EEPROM Action
		mlActionAck.result = EEP_MEMORY_CORR; // Memory Corrupted
            */
            
            mlPending.statustext++;

            mlStatustext.severity = MAV_SEVERITY_ERROR;
            strncat(mlStatustext.text, "Memory corrupted while initializing EEPROM.", 49);
            //mlStatustext.text = "Memory corrupted while initializing EEPROM.";
        }
	
	return eepInitMsg;
}

void loadEEPData(void){
	unsigned char i;
	tFloatToChar tempShData;
	
	readParamsInEeprom();
	
	for(i = 0; i < MAX_NUM_WPS; i++ ){		
		// Way Points
		tempShData.shData[0]= DataEERead(WPS_OFFSET+i*WP_SIZE_IN_EEPROM);   
		tempShData.shData[1]= DataEERead(WPS_OFFSET+i*WP_SIZE_IN_EEPROM+1);      
		mlWpValues.lat[i] = isFinite(tempShData.flData)? tempShData.flData : 0.0;      
		
		tempShData.shData[0]= DataEERead(WPS_OFFSET+i*WP_SIZE_IN_EEPROM+2);      
		tempShData.shData[1]= DataEERead(WPS_OFFSET+i*WP_SIZE_IN_EEPROM+3);      
		mlWpValues.lon[i] = isFinite(tempShData.flData)? tempShData.flData : 0.0;      
		
		tempShData.shData[0]= DataEERead(WPS_OFFSET+i*WP_SIZE_IN_EEPROM+4);      
		tempShData.shData[1]= DataEERead(WPS_OFFSET+i*WP_SIZE_IN_EEPROM+5);      
		mlWpValues.alt[i] = isFinite(tempShData.flData)? tempShData.flData : 0.0;     
		
		mlWpValues.type[i]	= (uint8_t)DataEERead(WPS_OFFSET+i*WP_SIZE_IN_EEPROM+6);
		
		mlWpValues.orbit[i]   = DataEERead(WPS_OFFSET+i*WP_SIZE_IN_EEPROM +7);         	
		         	
	}

	// Compute the waypoint count
	mlWpValues.wpCount = 0;
	while ((int)(mlWpValues.lat[mlWpValues.wpCount]) != 0 && mlWpValues.wpCount< MAX_NUM_WPS-1 ){
		mlWpValues.wpCount++;
	}
}

/**
 * @brief Stores a waypoint item to EEPROM.
 * @param mlSingleWp Pointer to mission
 * @return Zero on success, or an error code.
 * @todo Check if all mission parameters are recorded.
 */
uint8_t storeWaypointInEeprom (mavlink_mission_item_t* mlSingleWp){
	
	uint8_t indexOffset = 0, indx= 0, writeSuccess = 0;
	tFloatToChar tempFloat;
	
	// get the WP index
	indx = (uint8_t)mlSingleWp->seq;
					
	// Compute the adecuate index offset
	indexOffset = indx*WP_SIZE_IN_EEPROM;
	
	// Save the data to the EEPROM
	tempFloat.flData = mlSingleWp->x;
	writeSuccess += DataEEWrite(tempFloat.shData[0], WPS_OFFSET+indexOffset);   
	writeSuccess += DataEEWrite(tempFloat.shData[1], WPS_OFFSET+indexOffset+1);
	
	tempFloat.flData = mlSingleWp->y; 
	writeSuccess += DataEEWrite(tempFloat.shData[0], WPS_OFFSET+indexOffset+2);      
	writeSuccess += DataEEWrite(tempFloat.shData[1], WPS_OFFSET+indexOffset+3);
	
	tempFloat.flData = mlSingleWp->z;       
	writeSuccess += DataEEWrite(tempFloat.shData[0], WPS_OFFSET+indexOffset+4);      
	writeSuccess += DataEEWrite(tempFloat.shData[1], WPS_OFFSET+indexOffset+5);
	
	writeSuccess += DataEEWrite((unsigned short)mlSingleWp->command, WPS_OFFSET+indexOffset+6);
	
	writeSuccess += DataEEWrite((unsigned short)mlSingleWp->param3, WPS_OFFSET+indexOffset+7);          
		
	return writeSuccess;
}

/**
 * @brief Writes a parameter to EEPROM.
 * @param parameter Value of parameter.
 * @param pmIndex Index of paramter.
 * @return Zero on success, and non-zero on failure
 */
uint8_t storeParameterInEeprom (float parameter, uint8_t pmIndex){
	uint8_t indexOffset = 0,/* indx= 0 ,*/ writeSuccess = 0;
	tFloatToChar tempFloat;
	// Save the data to the EEPROM
	indexOffset = pmIndex*2;
	
	tempFloat.flData = parameter;
	writeSuccess += DataEEWrite(tempFloat.shData[0], PARAM_OFFSET+indexOffset);
	writeSuccess += DataEEWrite(tempFloat.shData[1], PARAM_OFFSET+indexOffset+1);
	
	
	return writeSuccess;
}

/**
 * @brief Saves all parameters to EEPROM.
 * @return Zero for success, or non-zero error code.
 */
uint8_t storeAllParamsInEeprom(void){
	uint8_t  indx= 0, writeSuccess = 0;
	
	for (indx = 0; indx < PAR_PARAM_COUNT; indx ++){
		writeSuccess += storeParameterInEeprom(mlParamInterface.param[indx], indx);
	}
	
	return writeSuccess;
}


/**
 * @brief Stores the specified mid-level command values to EEPROM.
 * @note Mid-level commands are read from the mlMidLevelCommands struct.
 * @return SUCCESS or FAILURE.
 */
uint8_t storeMidLevelCommandsInEeprom (void) {
	uint8_t indexOffset = 0,/* indx= 0 ,*/ writeSuccess = 0;
	tFloatToChar tempFloat;

	// Save the data to the EEPROM
	tempFloat.flData = mlMidLevelCommands.hCommand;
	writeSuccess += DataEEWrite(tempFloat.shData[0], MIDLEVEL_OFFSET + indexOffset++);
	writeSuccess += DataEEWrite(tempFloat.shData[1], MIDLEVEL_OFFSET + indexOffset++);

	tempFloat.flData = mlMidLevelCommands.uCommand;
	writeSuccess += DataEEWrite(tempFloat.shData[0], MIDLEVEL_OFFSET + indexOffset++);
	writeSuccess += DataEEWrite(tempFloat.shData[1], MIDLEVEL_OFFSET + indexOffset++);

	tempFloat.flData = mlMidLevelCommands.rCommand;
	writeSuccess += DataEEWrite(tempFloat.shData[0], MIDLEVEL_OFFSET + indexOffset++);
	writeSuccess += DataEEWrite(tempFloat.shData[1], MIDLEVEL_OFFSET + indexOffset);

	return writeSuccess;
}

/**
 * @brief Reads all paramters from EEPROM to RAM.
 * @return Zero on success, or non-zero error code.
 */
uint8_t readParamsInEeprom (void){
	uint8_t indx= 0, i=0;
	tFloatToChar tempFloat;
        uint8_t readSuccess = SUCCESS;
		
	for(indx = 0; indx < (PAR_PARAM_COUNT*2); indx+=2 ){
		tempFloat.shData[0]= DataEERead(PARAM_OFFSET+indx);
		tempFloat.shData[1]= DataEERead(PARAM_OFFSET+indx+1);

                // This doesn't seem to indicate success or failure.
                //if (tempFloat.shData[0] == 0xFFFF || tempFloat.shData[1] == 0xFFFF)
                //    readSuccess = FAILURE;
		
		// If the value read from memory is finite assign it, else assign 0
		mlParamInterface.param[i++]	= isFinite(tempFloat.flData)? tempFloat.flData : 0.0;
	}

        return readSuccess; 
}


/**
 * @brief Reads mid-level command values from EEPROM and loads into
 *  mlMidLevelCommand struct.
 */
uint8_t readMidLevelCommandsInEeprom (void) {
    uint8_t indx= 0, i=0;
    tFloatToChar tempFloat;
    uint8_t readSuccess = SUCCESS;

    tempFloat.shData[0]= DataEERead(MIDLEVEL_OFFSET + indx++);
    tempFloat.shData[1]= DataEERead(MIDLEVEL_OFFSET + indx++);
    // If the value read from memory is finite assign it, else assign 0
    mlMidLevelCommands.hCommand = isFinite(tempFloat.flData)? tempFloat.flData : 0.0f;

    tempFloat.shData[0]= DataEERead(MIDLEVEL_OFFSET + indx++);
    tempFloat.shData[1]= DataEERead(MIDLEVEL_OFFSET + indx++);
    // If the value read from memory is finite assign it, else assign 0
    mlMidLevelCommands.uCommand = isFinite(tempFloat.flData)? tempFloat.flData : 0.0f;

    tempFloat.shData[0]= DataEERead(MIDLEVEL_OFFSET + indx++);
    tempFloat.shData[1]= DataEERead(MIDLEVEL_OFFSET + indx);
    // If the value read from memory is finite assign it, else assign 0
    mlMidLevelCommands.rCommand = isFinite(tempFloat.flData)? tempFloat.flData : 0.0f;

    return readSuccess;
}

