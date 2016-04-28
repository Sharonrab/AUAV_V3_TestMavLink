#include "FCBEX1000.h"

void uartCameraInit(void) {


    // Configure and open the port;
    // U1MODE Register
    // ==============
    U1MODEbits.UARTEN = 0; // Disable the port		
    U1MODEbits.USIDL = 0; // Stop on idle
    U1MODEbits.IREN = 0; // No IR decoder
    U1MODEbits.RTSMD = 0; // Ready to send mode (irrelevant)
    U1MODEbits.UEN = 0; // Only RX and TX
    U1MODEbits.WAKE = 1; // Enable at startup
    U1MODEbits.LPBACK = 0; // Disable loopback
    U1MODEbits.ABAUD = 0; // Disable autobaud
    U1MODEbits.URXINV = 0; // Normal operation (high is idle)
    U1MODEbits.PDSEL = 0; // No parity 8 bit
    U1MODEbits.STSEL = 0; // 1 stop bit
    U1MODEbits.BRGH = 0; // Low speed mode

    // U1STA Register
    // ==============
    U1STAbits.URXISEL = 2; // RX interrupt when 3 chars are in
    U1STAbits.OERR = 0; // clear overun error

    // U1BRG Register
    // ==============
    U1BRG = CAMERA_UBRG; // Set the baud rate for initial config of GPS

    // Enable the port;
    U1MODEbits.UARTEN = 1; // Enable the port	
    U1STAbits.UTXEN = 1; // Enable TX

    // Configure the Camera
    configuracionInicial();

    // Configure the Pan Tilt
    panTiltInit();

}

void configuracionInicial(void) { //Configura la direccion de la camara, debiendo solamente ser cada vez que se encienda la camara
 
    /*VISCA is a protocol, which normally supports a daisy chain of up to seven connected 
     * cameras via RS- 232C interface. In such cases, the address set command can be used 
     * to assign addresses from 1 to 7 to each of the seven cameras, allowing you to control 
     * the seven cameras with the same personal computer. Although the FCB camera does 
     * not support direct connection of cameras in a daisy chain, be sure to use the address 
     * set command to set the address whenever a camera is connected for the first time.
     */
    /*
     0x88 0x30  0x01 0xff 0x00
     0x88 0x01 0x00 0x01 0xff 0x00
     0x81 0x01 0x04 0x06 0x03 0xFF
     0x81 0x01 0x04 0x36 0x01 0xFF
     0x81 0x01 0x04 0x53 0x01 0xFF
     0x81 0x01 0x04 0x34 0x02 0xFF
     * 
     */
    uint8_t cameraAddressSet[] = {0x88, 0x30, 0x01, 0xff, 0x00};
    uint8_t cameraIFClear[] = {0x88, 0x01, 0x00, 0x01, 0xff, 0x00};
    
    uint8_t dZoomOff[] = {0x81, 0x01, 0x04, 0x06, 0x03, 0xFF};
    uint8_t zoomSeparate[] ={0x81, 0x01, 0x04, 0x36, 0x01, 0xFF};
    
    uint8_t cameraNR[] = {0x81,0x01,0x04,0x53,0x01,0xFF};

    sendToCamera (cameraAddressSet, sizeof(cameraAddressSet));

    sendToCamera(cameraIFClear, sizeof(cameraIFClear));    

    hugeDelay(); // wait for camera address change to settle

    sendToCamera(dZoomOff, sizeof(dZoomOff));

    sendToCamera(zoomSeparate, sizeof(zoomSeparate));

    sendToCamera(cameraNR, sizeof(cameraNR));
}

void panTiltInit(void) {
    OC1CONbits.OCM = 0b000; // Turn off the module
    OC3CONbits.OCM = 0b000;
    OC6CONbits.OCM = 0b000;

    OC1CONbits.OCTSEL = 0b1; // TIMER3 is clock source
    OC3CONbits.OCTSEL = 0b1;
    OC6CONbits.OCTSEL = 0b1;

    OC1R = ZERO_PAN; // First duty cycle
    OC3R = ZERO_TILT;
    OC6R = ZERO_ROLL;

    OC1CONbits.OCM = 0b110; // PWM Mode with no fault detection
    OC3CONbits.OCM = 0b110;
    OC6CONbits.OCM = 0b110;

    T3CONbits.TON = 0; // Turn timer off
    T3CONbits.TCS = 0; // internal source (FOSC/2) i.e. 40 MHZ
    T3CONbits.TGATE = 0; // Disable gating
    T3CONbits.TCKPS = 0b10; // 1:64 scale
    TMR3 = 0x00; // Timer 3 initialize
    PR3 = 12499; // Expire at .02 Sec

    IPC2bits.T3IP = 0x04; // Interrupt priority 4
    IFS0bits.T3IF = 0; // Clear the Interrupt Flag
    IEC0bits.T3IE = 1; // Enable Interrupts

    T3CONbits.TON = 1;
}

void controlZoom(uint8_t order) { //Controla el zoom en 10 posiciones
    uint8_t mensaje[] = {0x81, 0x01, 0x04, 0x47, 0x00, 0x00, 0x00, 0x00, 0xff}; //Controla el zoom optico teniendo una entrada decimal de 1-10
    //uint8_t i;
    switch (order) {
        case 1:
            mensaje[4] = 0x00;
            mensaje[5] = 0x00;
            mensaje[6] = 0x00;
            mensaje[7] = 0x00;
            break;
        case 2:
            mensaje[4] = 0x01;
            mensaje[5] = 0x06;
            mensaje[6] = 0x06;
            mensaje[7] = 0x0F;
            break;
        case 3:
            mensaje[4] = 0x01;
            mensaje[5] = 0x0F;
            mensaje[6] = 0x0F;
            mensaje[7] = 0x00;
            break;
        case 4:
            mensaje[4] = 0x02;
            mensaje[5] = 0x05;
            mensaje[6] = 0x07;
            mensaje[7] = 0x0D;
            break;
        case 5:
            mensaje[4] = 0x02;
            mensaje[5] = 0x09;
            mensaje[6] = 0x04;
            mensaje[7] = 0x00;
            break;
        case 6:
            mensaje[4] = 0x02;
            mensaje[5] = 0x0C;
            mensaje[6] = 0x00;
            mensaje[7] = 0x02;
            break;
        case 7:
            mensaje[4] = 0x02;
            mensaje[5] = 0x0E;
            mensaje[6] = 0x02;
            mensaje[7] = 0x0B;
            break;
        case 8:
            mensaje[4] = 0x02;
            mensaje[5] = 0x0F;
            mensaje[6] = 0x0E;
            mensaje[7] = 0x0E;
            break;
        case 9:
            mensaje[4] = 0x03;
            mensaje[5] = 0x01;
            mensaje[6] = 0x06;
            mensaje[7] = 0x0A;
            break;
        case 10:
            mensaje[4] = 0x03;
            mensaje[5] = 0x02;
            mensaje[6] = 0x0B;
            mensaje[7] = 0x02;
            break;
    }

    sendToCamera(mensaje, sizeof (mensaje));
}

void changeZoom(uint8_t newZoom) {
    static uint8_t oldZoom = 1;

    if (oldZoom != newZoom) {
        controlZoom(newZoom);
        oldZoom = newZoom;
    }
}

uint8_t getCurrentZoom(void) {
    return (uint8_t) mlCameraOrder.zoom;

}

void setCameraConfig(void) {
    static uint8_t currentConfig[] = {0, 0, 0, 0, 0, 0, 0, 0, 0};

    if (currentConfig[mlCameraConfig.idOrder] != mlCameraConfig.order) {

        currentConfig[mlCameraConfig.idOrder] = mlCameraConfig.order;

        switch (mlCameraConfig.idOrder) {
            case 1:
                // Control Brightness
                controlBrightness(mlCameraConfig.order);
                mlCameraConfig.order = 0;
                break;

            case 2:
                // control Aperture
                break;

            case 3:
                // control Iris
                break;

            case 4:
                // ICR
                controlICR(mlCameraConfig.order);
                break;
                
            case 5: 
                // Stabilization
                controlStabilization(mlCameraConfig.order);
                break;
                
            case 6:
                controlSmoothing(mlCameraConfig.order);
                break;
        }

    }
}

void controlICR(uint8_t icr) {
    /*
     On   8x 01 04 01 02 FF   Infrared Mode ON/OFF
     Off   8x 01 04 01 03 FF
     */
    uint8_t mensaje[] = {0x81, 0x01, 0x04, 0x01, 0x00, 0xff}; //Controla el ICR

    if (icr == 1) {
        mensaje[4] = 02;
    } else {
        mensaje [4] = 03;
    }

    sendToCamera(mensaje, sizeof (mensaje));
}

void controlBrightness(uint8_t bright) {
    /*
     */

    uint8_t mensaje[] = {0x81, 0x01, 0x04, 0x0D, 0x00, 0xFF}; //Controla el ICR

    switch (bright) {
        case 1:
            mensaje[4] = 0x02;
            break;
        case 2:
            mensaje[4] = 0x03;
            break;
        case 3:
            mensaje[4] = 0x00;
            break;
    }

    sendToCamera(mensaje, sizeof (mensaje));
}

void controlStabilization(uint8_t stabil) {
    /*
     On   8x 01 04 01 02 FF   Infrared Mode ON/OFF
     Off   8x 01 04 01 03 FF
     */
    uint8_t mensaje[] = {0x81, 0x01, 0x04, 0x34, 0x03, 0xff}; //Apaga Estabilizador

    if (stabil != 1) {
        mensaje[4] = 02;
    } 

    sendToCamera(mensaje, sizeof (mensaje));
}

void controlSmoothing(uint8_t smooth) {
    /*
     */

    uint8_t mensaje[] = {0x81, 0x01, 0x04, 0x53, 0x00, 0xFF}; //Apaga Smoothing

    mensaje[4] = smooth;

    sendToCamera(mensaje, sizeof (mensaje));
}

void sendToCamera(uint8_t* mensaje, uint8_t size) {
    uint8_t i;

    for (i = 0; i < size; i++) {
        WriteUART1((unsigned int) mensaje[i]);
        while (BusyUART1());
    }
    tinyDelay();
}

uint16_t getPanValue(uint8_t goHome) {
    //    float result = (Usec * 0.625);

    static uint16_t currentPan = ZERO_PAN;
    uint16_t tempOrder = currentPan;

    if (mlCameraOrder.pan != 0) {
        tempOrder += (uint16_t) (mlCameraOrder.pan * PANTILT_INCREMENT);
        mlCameraOrder.pan = 0;

        if (tempOrder > MAX_PAN) {
            tempOrder = MAX_PAN;
        }

        if (tempOrder < MIN_PAN) {
            tempOrder = MIN_PAN;
        }

        currentPan = tempOrder;
    }

    if (goHome) {
        currentPan = ZERO_PAN;
        tempOrder = ZERO_PAN;
    }
    return tempOrder;

}

uint16_t getTiltValue(uint8_t goHome) {
    //    float result = (Usec * 0.625);

    static uint16_t currentTilt = ZERO_TILT;
    uint16_t tempOrder = currentTilt;

    if (mlCameraOrder.tilt != 0) {
        tempOrder += (uint16_t) (mlCameraOrder.tilt * PANTILT_INCREMENT);
        mlCameraOrder.tilt = 0;

        if (tempOrder > MAX_TILT) {
            tempOrder = MAX_TILT;
        }

        if (tempOrder < MIN_TILT) {
            tempOrder = MIN_TILT;
        }

        currentTilt = tempOrder;
    }

    if (goHome) {
        currentTilt = ZERO_TILT;
        tempOrder = ZERO_TILT;
    }


    return tempOrder;
}

void updatePan(uint16_t newPan) {
    if (newPan > MAX_PAN) {
        OC1RS = MAX_PAN;
        return;
    }

    if (newPan < MIN_PAN) {
        OC1RS = MIN_PAN;
        return;
    }

    OC1RS = newPan;
}

void updateTilt(uint16_t newTilt) {
    if (newTilt > MAX_TILT) {
        OC3RS = MAX_TILT;
        return;
    }

    if (newTilt < MIN_TILT) {
        OC3RS = MIN_TILT;
        return;
    }

    OC3RS = newTilt;
}

void updateRoll(uint16_t newRoll) {
    if (newRoll > MAX_ROLL) {
        OC6RS = MAX_ROLL;
        return;
    }

    if (newRoll < MIN_ROLL) {
        OC6RS = MIN_ROLL;
        return;
    }

    OC6RS = newRoll;
}

uint8_t getGoHome(void) {
    uint8_t localGoHome = mlCameraOrder.moveHome;

    if (mlCameraOrder.moveHome) {
        mlCameraOrder.moveHome = 0;
    }

    return localGoHome;
}

//TODO: getTilt and getTiltAide have a useless variable named 
// temp order, this needs to get eliminated and simplify the function.
int16_t getTiltISRAide(uint8_t goHome) {
    //    float result = (Usec * 0.625);

    static int16_t currentTiltAide = 0;
    int16_t tempOrder = currentTiltAide;

    if (mlCameraOrder.tilt != 0) {
        tempOrder += (int16_t) (mlCameraOrder.tilt * PANTILT_INCREMENT);
        mlCameraOrder.tilt = 0;

        if (tempOrder > MAX_TILT_AIDE) {
            tempOrder = MAX_TILT_AIDE;
        }

        if (tempOrder < MIN_TILT_AIDE) {
            tempOrder = MIN_TILT_AIDE;
        }

        currentTiltAide = tempOrder;
    }

    if (goHome) {
        currentTiltAide = 0;
        tempOrder = 0;
    }


    return tempOrder;
}

//TODO: getPan and getPanAide have a useless variable named 
// temp order, this needs to get eliminated and simplify the function.
int16_t getPanISRAide(uint8_t goHome) {
    //    float result = (Usec * 0.625);

    static int16_t currentPanAide = 0;
    int16_t tempOrder = currentPanAide;

    if (mlCameraOrder.pan != 0) {
        tempOrder += (int16_t) (mlCameraOrder.pan * PANTILT_INCREMENT);
        mlCameraOrder.pan = 0;

        if (tempOrder > MAX_PAN_AIDE) {
            tempOrder = MAX_PAN_AIDE;
        }

        if (tempOrder < MIN_PAN_AIDE) {
            tempOrder = MIN_PAN_AIDE;
        }

        currentPanAide = tempOrder;
    }

    if (goHome) {
        currentPanAide = 0;
        tempOrder = 0;
    }


    return tempOrder;
}
void __attribute__((__interrupt__, no_auto_psv)) _T3Interrupt(void) {
    IFS0bits.T3IF = 0;
}


#if 0

void controlCamara(void) { //Lee el mensaje y direcciona hacia la funcion que se requiera
    int idOrder, order;

    configuracionInicial();

    do {
        printf("Mensaje #186 (idOrder, order) separados por un espacio: ");
        scanf("%d %d", &idOrder, &order);

        switch (idOrder) {
                /*case 0:
                    controlZoom(order);
                    break; */
            case 1:
                controlBrillo(order);
                break;
            case 2:
                controlApertura(order);
                break;
            case 3:
                controlIris(order);
                break;
            case 4:
                controlICRAuto(order);
                break;
            case 5:
                controlICR(order);
                break;
            case 6:
                controlBacklight(order);
                break;
        }

    } while (idOrder != 0);
}

void controlZoom(void) { //Controla el zoom optico y digital teniendo una entrada decimal
    int numero;
    unsigned char hexa[9];

    do {
        printf("Numero decimal (0000-35808): ");
        scanf("%d", &numero);
        construyeZoom(hexa, numero);
        printf(" %02X %02X %02X %02X %02X %02X %02X %02X %02X \n", hexa[0], hexa[1], hexa[2], hexa[3], hexa[4], hexa[5], hexa[6], hexa[7], hexa[8]);
    } while (numero != 0);
}

construyeZoom(unsigned char* hexa, int numero) { //Convierte la entrada decimal a una posicion en hexadecimal del zoom
    int i, residuo, cociente;
    hexa[0] = 0x81;
    hexa[1] = 0x01;
    hexa[2] = 0x04;
    hexa[3] = 0x47;
    hexa[8] = 0xFF;

    for (i = 7; i >= 4; i--) {
        cociente = numero / 16;
        residuo = numero % 16;
        numero = cociente;
        hexa[i] = residuo;
    }
}

void controlBrillo(int order) { //Controla el brilla cambiando automaticamenta o modo manual para poder controlar el brillo
    unsigned char mensaje[7];

    mensaje[0] = 0x81;
    mensaje[1] = 0x01;
    mensaje[2] = 0x04;
    mensaje[3] = 0x0D;
    mensaje[5] = 0xFF;
    mensaje[6] = 0x39;

    switch (order) {
        case 1:
            mensaje[4] = 0x02;
            break;
        case 2:
            mensaje[4] = 0x03;
            break;
        case 3:
            mensaje[4] = 0x00;
            break;
    }

    if (order == 1 || order == 2) {
        printf(" %02X %02X %02X %02X %02X %02X \n", mensaje[0], mensaje[1], mensaje[2], mensaje[6], mensaje[3], mensaje[5]); //Cambia a modo manual
        printf(" %02X %02X %02X %02X %02X %02X \n", mensaje[0], mensaje[1], mensaje[2], mensaje[3], mensaje[4], mensaje[5]); //Modifica el brillo
    }
    if (order == 3) {
        printf(" %02X %02X %02X %02X %02X %02X \n", mensaje[0], mensaje[1], mensaje[2], mensaje[6], mensaje[4], mensaje[5]); //Cambia a modo automatico
    }
}

void controlApertura(int order) { //Controla la apertura
    unsigned char mensaje[6];

    mensaje[0] = 0x81;
    mensaje[1] = 0x01;
    mensaje[2] = 0x04;
    mensaje[3] = 0x02;
    mensaje[5] = 0xFF;

    switch (order) {
        case 1:
            mensaje[4] = 0x02;
            break;
        case 2:
            mensaje[4] = 0x03;
            break;
        case 3:
            mensaje[4] = 0x00;
            break;
    }
    printf(" %02X %02X %02X %02X %02X %02X \n", mensaje[0], mensaje[1], mensaje[2], mensaje[3], mensaje[4], mensaje[5]);
}

void controlIris(int order) { //Controla el iris
    unsigned char mensaje[7];

    mensaje[0] = 0x81;
    mensaje[1] = 0x01;
    mensaje[2] = 0x04;
    mensaje[3] = 0x0B;
    mensaje[5] = 0xFF;
    mensaje[6] = 0x39;

    switch (order) {
        case 1:
            mensaje[4] = 0x02;
            break;
        case 2:
            mensaje[4] = 0x03;
            break;
        case 3:
            mensaje[4] = 0x00;
            break;
    }

    if (order == 1 || order == 2) {
        printf(" %02X %02X %02X %02X %02X %02X \n", mensaje[0], mensaje[1], mensaje[2], mensaje[6], mensaje[3], mensaje[5]); //Cambia a modo manual
        printf(" %02X %02X %02X %02X %02X %02X \n", mensaje[0], mensaje[1], mensaje[2], mensaje[3], mensaje[4], mensaje[5]); //Modifica el brillo
    }
    if (order == 3) {
        printf(" %02X %02X %02X %02X %02X %02X \n", mensaje[0], mensaje[1], mensaje[2], mensaje[6], mensaje[4], mensaje[5]);
    }
}

void controlICRAuto(int order) { //Enciende y Apaga el ICR Automatico
    unsigned char mensaje[6];

    mensaje[0] = 0x81;
    mensaje[1] = 0x01;
    mensaje[2] = 0x04;
    mensaje[3] = 0x51;
    mensaje[5] = 0xFF;

    switch (order) {
        case 1:
            mensaje[4] = 0x02;
            break;
        case 2:
            mensaje[4] = 0x03;
            break;
    }
    printf(" %02X %02X %02X %02X %02X %02X \n", mensaje[0], mensaje[1], mensaje[2], mensaje[3], mensaje[4], mensaje[5]); //Cambia a modo automatico
}

void controlICR(int order) { //Enciende y Apaga el ICR Manual
    /*
     On   8x 01 04 01 02 FF   Infrared Mode ON/OFF
     Off   8x 01 04 01 03 FF

     */
    unsigned char mensaje[6];
    mensaje[0] = 0x81;
    mensaje[1] = 0x01;
    mensaje[2] = 0x04;
    mensaje[3] = 0x01;
    mensaje[5] = 0xFF;

    switch (order) {
        case 1:
            mensaje[4] = 0x02;
            break;
        case 2:
            mensaje[4] = 0x03;
            break;
    }
    printf(" %02X %02X %02X %02X %02X %02X \n", mensaje[0], mensaje[1], mensaje[2], mensaje[3], mensaje[4], mensaje[5]);
}

void controlBacklight(int order) { //Enciende y Apaga el Backlight
    unsigned char mensaje[6];
    mensaje[0] = 0x81;
    mensaje[1] = 0x01;
    mensaje[2] = 0x04;
    mensaje[3] = 0x33;
    mensaje[5] = 0xFF;

    switch (order) {
        case 1:
            mensaje[4] = 0x02;
            break;
        case 2:
            mensaje[4] = 0x03;
            break;
    }
    printf(" %02X %02X %02X %02X %02X %02X \n", mensaje[0], mensaje[1], mensaje[2], mensaje[3], mensaje[4], mensaje[5]);
}

#endif

