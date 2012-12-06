#include <MathHelper.h>


/*
 * ArduinoDUE.cpp
 *
 * Created: 05-Dec-12 2:16:34 PM
 *  Author: M.Hefny
 */ 

#define DEBUG_SERIAL

#include <Arduino.h>
#include "typedef.h"
#include "Common.h"
#include "Init.h"
#include "RX.h"
#include "Motor.h"



/**
 * \brief Application entry point.
 *
 * \return Unused (ANSI-C compatibility).
 */
 
 
 
void setup()
{
    /* Initialize the SAM system */
    SystemInit();

    WDT->WDT_MR = WDT_MR_WDDIS;
    Serial.begin(115200);
#ifdef DEBUG_SERIAL
    Serial.print("started\r\n");
#endif
    InitBoard();
    GetStabilizationOutput(PhysicalMotor,&VirtualMotors);
    GetNavigationOutput(PhysicalMotor,&RXRemote[1]);

}

void loop ()
{
    /*  while (1) 
    {
        GetStabilizationOutput(PhysicalMotor,VirtualMotors);
        GetNavigationOutput(PhysicalMotor,&RXRemote[1]);
    }*/
}



