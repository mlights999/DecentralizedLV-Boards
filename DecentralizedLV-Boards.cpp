
#include "DecentralizedLV-Boards.h"
#include "Particle.h"
#include <mcp_can.h>


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////       DASHBOARD CONTROLLER FUNCTIONS      //////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

/// @brief Creates an instance of the controller to either send or receive CAN frames for this board. Takes agreed upon address for this board. Example: 'DashboardController_CAN dc(0x99);' would create a Dashboard Controller that transmits on 0x99.
/// @param boardAddr The 32-bit CAN Bus address the Dashboard Controller transmits on.
DashController_CAN::DashController_CAN(uint32_t boardAddr){
    boardAddress = boardAddr;
}

/// @brief Initializes the control fields of the Dashboard Controller to a default value. 
void DashController_CAN::initialize(){
    rightTurnPWM = 0;
    leftTurnPWM = 0;
    occupantFanPWM = 0;
    frontLeftFan1PWM = 0; // Front-Left Fan 1 (HP1)
    frontLeftFan2PWM = 0; // Front-Left Fan 2 (HP0)
    frontRightFanPWM = 0; // Front-Right Fan (HP0)
    headlight = false;
    highbeam = false;
    runningLights = false;
    reversePress = false;
    driveMode = 0;
    radiatorFanPWM = 0;
    radiatorPump = false;
    bmsFaultDetected = false;
    rmsFaultDetected = false;
    boardDetected = false;
}

/// @brief Takes the variables that you've previously updated and sends them out in the agreed CAN bus format for this board.
/// @param controller The CAN bus controller attached to this microcontroller.
/// Encodes new fan PWM values into CAN message bytes
void DashController_CAN::sendCANData(ICANController &controller){
    byte tx2 = frontLeftFan1PWM; // Front-Left Fan 1 PWM (byte 2)
    byte tx4 = headlight + (highbeam << 1) + (runningLights << 2) + (reversePress << 5);
    byte tx5 = frontLeftFan2PWM; // Front-Left Fan 2 PWM (byte 5)
    byte tx6 = driveMode;
    byte tx7 = (radiatorFanPWM ? 1 : 0) + (radiatorPump << 1) + ((frontRightFanPWM >> 5) << 2); // Radiator fan on/off (bit 0, only 1 bit available - see radiatorFanPWM comment); Front-Right Fan upper bits (bits 2-4 of byte 7)
    controller.send(boardAddress, rightTurnPWM,leftTurnPWM,tx2,occupantFanPWM,tx4,tx5,tx6,tx7);   //Send out the main message to the corner boards
}

/// @brief Extracts CAN frame data into the object's variables so you can use them for controlling other things
/// @param msg The CAN frame that was received by can.receive(). Need to convert from CANBusMessage to CANBusMessage by copying address and byte.
/// Decondes new fan PWM values from CAN message bytes
void DashController_CAN::receiveCANData(CANBusMessage msg){
    if(msg.addr == boardAddress){   //Our message that we received was from this board. Go ahead and import the data to the packets.
        boardDetected = true;
        rightTurnPWM = msg.bytes[0];
        leftTurnPWM = msg.bytes[1];
        frontLeftFan1PWM = msg.bytes[2]; // Extract Front-Left Fan 1 from byte 2
        occupantFanPWM = msg.bytes[3];
        headlight = msg.bytes[4] & 1;
        highbeam = (msg.bytes[4] >> 1) & 1;
        runningLights = (msg.bytes[4] >> 2) & 1;
        reversePress = (msg.bytes[4] >> 5) & 1;
        frontLeftFan2PWM = msg.bytes[5]; // Extract Front-Left Fan 2 from byte 5
        driveMode = msg.bytes[6];
        radiatorFanPWM = (msg.bytes[7] & 1) ? 255 : 0; // Only 1 bit is transmitted (see radiatorFanPWM comment) - expand to full on/off
        radiatorPump = (msg.bytes[7] >> 1) & 1;
        frontRightFanPWM = ((msg.bytes[7] >> 2) & 0x07) << 5; // Extract Front-Right Fan from bits 2-4, shift back to full byte
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////         HIGH VOLTAGE CONTROLLER FUNCTIONS        ///////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

/// @brief Creates an instance of the controller to either send or receive CAN frames for this board. Takes agreed upon address for this board. Example: 'PowerController_CAN pc(0x120);' would create a Power Controller that transmits on 0x120.
/// @param boardAddr The 32-bit CAN Bus address the Power Controller transmits on.
HVController_CAN::HVController_CAN(uint32_t boardAddr){
    boardAddress = boardAddr;
}


/// @brief Initializes the control fields of the Power Controller to a default value. 
void HVController_CAN::initialize(){
    Killswitch = false;
    BMSFault = false;
    boardDetected = false;
    dischargeContactorOn = false;
    chargeContactorOn = false;
    chargeSafetyOn = false;
    motorTemperatureC = 0;
    inverterTemperatureC = 0;
    thermistorHighTempC = 0;
    batteryFanPWM = 0;
}

/// @brief Takes the variables that you've previously updated and sends them out in the agreed CAN bus format for this board.
/// @param controller The CAN bus controller attached to this microcontroller.
void HVController_CAN::sendCANData(ICANController &controller){
    byte tx0 = Killswitch + (BMSFault << 1) + (dischargeContactorOn << 2) + (chargeContactorOn << 3) + (chargeSafetyOn << 4);
    uint16_t motorTemperatureTemp = (uint16_t)(motorTemperatureC * 10);        //Convert to 0.1C increments
    uint16_t inverterTemperatureTemp = (uint16_t)(inverterTemperatureC * 10);  //Convert to 0.1C increments
    controller.send(boardAddress, tx0, packSOC, (uint8_t)(motorTemperatureTemp >> 8), (uint8_t)(motorTemperatureTemp & 0xFF), (uint8_t)(inverterTemperatureTemp >> 8), (uint8_t)(inverterTemperatureTemp & 0xFF), thermistorHighTempC, batteryFanPWM);

}

/// @brief Extracts CAN frame data into the object's variables so you can use them for controlling other things
/// @param msg The CAN frame that was received by can.receive(). Need to convert from CANBusMessage to CANBusMessage by copying address and byte.
void HVController_CAN::receiveCANData(CANBusMessage msg){
    if(msg.addr == boardAddress){
        boardDetected = true;
        //do something with the hv controller data
        Killswitch = msg.bytes[0] & 1;
        BMSFault = (msg.bytes[0] >> 1) & 1;
        dischargeContactorOn = (msg.bytes[0] >> 2) & 1;
        chargeContactorOn = (msg.bytes[0] >> 3) & 1;
        chargeSafetyOn = (msg.bytes[0] >> 4) & 1;
        packSOC = msg.bytes[1];

        uint16_t motorTemperatureTemp = (uint16_t)(msg.bytes[2] << 8 | msg.bytes[3]);            //Convert to 0.1C increments
        uint16_t inverterTemperatureTemp = (uint16_t)(msg.bytes[4] << 8 | msg.bytes[5]);            //Convert to 0.1C increments
        motorTemperatureC = (float)(motorTemperatureTemp / 10.0);                          //Convert to degrees C
        inverterTemperatureC = (float)(inverterTemperatureTemp / 10.0);                       //Convert to degrees C

        thermistorHighTempC = msg.bytes[6];
        batteryFanPWM = msg.bytes[7];
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////         POWER CONTROLLER FUNCTIONS        //////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

/// @brief Creates an instance of the controller to either send or receive CAN frames for this board. Takes agreed upon address for this board. Example: 'PowerController_CAN pc(0x120);' would create a Power Controller that transmits on 0x120.
/// @param boardAddr The 32-bit CAN Bus address the Power Controller transmits on.
PowerController_CAN::PowerController_CAN(uint32_t boardAddr){
    boardAddress = boardAddr;
}

/// @brief Initializes the control fields of the Power Controller to a default value. 
void PowerController_CAN::initialize(){
    BrakeSense = false;
    PushToStart = false;
    ACCharge = false;
    SolarCharge = false;
    Horn = false;
    Acc = false;
    Ign = false;
    FullStart = false;
    CarOn = false;
    StartUp = false;
    LowPowerMode = false;
    LowACCBattery = false;
    boardDetected = false;
}

/// @brief Takes the variables that you've previously updated and sends them out in the agreed CAN bus format for this board.
/// @param controller The CAN bus controller attached to this microcontroller.
void PowerController_CAN::sendCANData(ICANController &controller){
    byte tx0 = BrakeSense + (PushToStart << 1) + (ACCharge << 2) + (SolarCharge << 3) + (Horn << 4);
    byte tx1 = Acc + (Ign << 1) + (FullStart << 2) + (CarOn << 3) + (StartUp << 4);
    byte tx2 = LowPowerMode + (LowACCBattery << 1);
    controller.send(boardAddress, tx0, tx1, tx2, 0, 0, 0, 0, 0);
}
/// @brief Extracts CAN frame data into the object's variables so you can use them for controlling other things
/// @param msg The CAN frame that was received by can.receive(). Need to convert from CANBusMessage to CANBusMessage by copying address and byte.
void PowerController_CAN::receiveCANData(CANBusMessage msg){
    if(msg.addr == boardAddress){
        boardDetected = true;
        //do something with the power controller data
        BrakeSense = (msg.bytes[0]) & 1;
        PushToStart = (msg.bytes[0] >> 1) & 1;
        ACCharge = (msg.bytes[0] >> 2) & 1;
        SolarCharge = (msg.bytes[0] >> 3) & 1;
        Horn = (msg.bytes[0] >> 4) & 1;
        Acc = (msg.bytes[1]) & 1;
        Ign = (msg.bytes[1] >> 1) & 1;
        FullStart = (msg.bytes[1] >> 2) & 1;
        CarOn = (msg.bytes[1] >> 3) & 1;
        StartUp = (msg.bytes[1] >> 4) & 1;
        LowPowerMode = (msg.bytes[2]) & 1;
        LowACCBattery = (msg.bytes[2] >> 1) & 1;
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////         REAR LEFT DRIVER FUNCTIONS        //////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

/// @brief Creates an instance of the controller to either send or receive CAN frames for this board. Takes agreed upon address for this board. Example: 'LPDRV_RearLeft_CAN pc(0x95);' would create a Rear Left Driver that transmits on 0x95.
/// @param boardAddr The 32-bit CAN Bus address the Rear Left Driver transmits on.
LPDRV_RearLeft_CAN::LPDRV_RearLeft_CAN(uint32_t boardAddr){
    boardAddress = boardAddr;
}

/// @brief Initializes the control fields of the Rear Left Driver to a default value. 
void LPDRV_RearLeft_CAN::initialize(){
    bmsFaultInput = false;
    switchFaultInput = false;
    boardDetected = false;
}

/// @brief Takes the variables that you've previously updated and sends them out in the agreed CAN bus format for this board.
/// @param controller The CAN bus controller attached to this microcontroller.
void LPDRV_RearLeft_CAN::sendCANData(ICANController &controller){
    controller.send(boardAddress, bmsFaultInput, switchFaultInput, 0, 0, 0, 0, 0, 0);
}

/// @brief Extracts CAN frame data into the object's variables so you can use them for controlling other things
/// @param msg The CAN frame that was received by can.receive(). Need to convert from CANBusMessage to CANBusMessage by copying address and byte.
void LPDRV_RearLeft_CAN::receiveCANData(CANBusMessage msg){
    if(msg.addr == boardAddress){
        boardDetected = true;
        bmsFaultInput = msg.bytes[0] & 1;  //Extract BMS fault from the first bit of byte 0
        bmsFaultInput = msg.bytes[1] & 1;  //Extract BMS fault from the first bit of byte 1
    }
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////         IBOOSTER FUNCTIONS         /////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

/// @brief Creates an instance of the controller to either send or receive CAN frames for this board. Takes agreed upon address for this board. Example: 'LPDRV_RearLeft_CAN pc(0x95);' would create a Rear Left Driver that transmits on 0x95.
/// @param boardAddr The 32-bit CAN Bus address the Rear Left Driver transmits on.
IBOOSTER_CAN::IBOOSTER_CAN(uint32_t boardAddr){
    boardAddress = boardAddr;
}

/// @brief Initializes the control fields of the Rear Left Driver to a default value. 
void IBOOSTER_CAN::initialize(){
    brakePercentage = 255;  //Null value
    boardDetected = false;
}

/// @brief Extracts CAN frame data into the object's variables so you can use them for controlling other things
/// @param msg The CAN frame that was received by can.receive(). Need to convert from CANBusMessage to CANBusMessage by copying address and byte.
void IBOOSTER_CAN::receiveCANData(CANBusMessage msg){
    if(msg.addr == boardAddress){
        boardDetected = true;

        //msg.bytes[5] ranges from 0x50 (fully released) to 0xC0 (fully pressed)
        static int brakeMin = 0x50;
        static int brakeMax = 0xC0;
        int brakeVal = constrain(msg.bytes[5], brakeMin, brakeMax);
        brakePercentage = (uint8_t)((100 * (brakeVal - brakeMin)) / (brakeMax - brakeMin));
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////         POWER CONTROLLER FUNCTIONS        //////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

/// @brief Initializes the control fields of the cluster to a default value. Must call sendCANData for the cluster to actually update.
void CamryCluster_CAN::initialize(){
    brakeIcon = false;
    parkingBrakeCircle = false;
    seatBeltIcon = false;
    checkEngineOn = false;
    clusterBacklight = true;
    oilPressureLow = false;
    chargingSystemMalfunction = false;
    motorTempDegC = 25;
    powerSteeringIcon = false;
    powerSteeringPrompt = 0x00;
    LCD_PowerPrompt = LCD_POWER_GOOD;
    LCD_Brightness = LCD_BRIGHTNESS_HIGH;
    trunkOpen = false;
    frontLeftDoor = false;
    frontRightDoor = false;
    rearLeftDoor = false;
    rearRightDoor = false;
    animateStartup = false;
    LCD_EngineStoppedCode = LCD_ENGINE_NORMAL;
    LCD_CheckEnginePrompt = LCD_CHECK_ENGINE_NONE;
    LCD_ParkingBrakePrompt = LCD_PBRK_GOOD;
    rpmGauge = 0;
    speedGauge = 0;
    ecoGauge = 0x3C;
    ecoLeaf = false;
    fogLightOrange = false;
    fogLightGreen = false;
    headlight = false;
    last_headlight = !headlight;
    highbeam = false;
    last_highbeam = !highbeam;
    driveMode = DRIVE_MODE_PARK;
    gearNumber = 0;
    sportMode = false;
    ecoMode = false;
    readyToDrive = false;
    timer25ms = 0;
    timer250ms = 0;
    timer1000ms = 0;
    crashBrakePrompt = 0;
    clusterBeeps = BEEP_RATE_OFF;
    hudBlueLeftLane = false;
    hudBlueRightLane = false;
    hudLeftLaneColor = HUD_LANE_OFF;
    hudRightLaneColor = HUD_LANE_OFF;
    LCD_TakeBreak_Prompt = LCD_TAKE_BREAK_NONE;
}

void CamryCluster_CAN::send25msPackets(ICANController &controller){
    

    //Periodically send out the rest of the 1000ms packets if no changes in value have occurred.
    if(millis() - timer25ms < 25) return;       //If we haven't hit 25ms yet, don't send the packets
    timer25ms = millis();                       //Reset the timer

    uint16_t speedMask = speedGauge * 160;

    static uint16_t speedFakeTimer = 0;
    static bool everyOther = false;

    if(everyOther){
        speedFakeTimer += speedGauge * 0.56;   //33 = ~1 mile per minute (60mph)
        everyOther = false;
    }
    else{
        everyOther = true;
    } 

    uint8_t economyBitmask = (uint8_t)((ecoGauge * 0x3C) / 100) & 0x3C;    //Calculate the economy bitmask based on the percentage of bars
    economyBitmask = ecoLeaf ? economyBitmask + 0xC0 : economyBitmask;          //Eco leaf sets upper two bits

    controller.send(PARKING_BRAKE_CAN_ADDR, 0x88, parkingBrakeCircle ? 0x02 : 0x00, LCD_ParkingBrakePrompt, 0, 0, 0, 0, 0xC7);
    controller.send(SPEED_CAN_ADDR, 0, 0, 0, 0, speedFakeTimer & 255, speedMask >> 8, speedMask & 255, 0xBC);                           //Spoof for speedometer
    if(driveMode != DRIVE_MODE_PARK) controller.send(FUEL_ECONOMY_CAN_ADDR, 0, 0, 0, 0, 0, 1, economyBitmask, 0);                 //Spoof for fuel economy meter
    //Update the last values for the next time we send out the 25ms packets
    
}

void CamryCluster_CAN::send250msPackets(ICANController &controller){

    uint8_t powerSteerState = powerSteeringIcon ? 0x38:0x00;    //Clear steering wheel icon if no power steering error

    uint8_t otherGear = 0;                                      //Variable to check if in neutral or reverse, default to no gear
    switch (driveMode){
        case DRIVE_MODE_PARK:
            otherGear = readyToDrive ? 0x20:0x00;
            break;
        case DRIVE_MODE_REVERSE:
            otherGear = 0x10;
            break;
        case DRIVE_MODE_SPORT:
            otherGear = 0x09;
            break;
        case DRIVE_MODE_NEUTRAL:
            otherGear = 0x08;
            break;
    }

    uint8_t driveSet = 0;                                       //Drive setting variable, default of 0 (no gear)
    if(driveMode != DRIVE_MODE_PARK && driveMode != DRIVE_MODE_REVERSE) driveSet = 0x80;           //If not in park mode, set this to 0x80

    uint8_t driveModifier = 0;                                  //Default normal drive mode (not eco or sport)      
    if(sportMode) driveModifier = 0x10;                         //If bit 1 is set, then we are in sport mode (0x10 to instrument cluster signals sport)
    if(ecoMode) driveModifier = 0x30;                           //If bit 2 is set, then we are in eco mode (0x30 to instrument cluster signals eco)

    //Allow for instant update if values have changed for the BRAKE icon
    static bool lastBrakeIcon;
    if(brakeIcon != lastBrakeIcon){
        controller.send(ABS_CAN_ADDR, (brakeIcon ? 0x40:0x00), 0, 0, 0, 0, 0, 0, 0);                         //Spoof Anti-Lock brakes (All 0's clears errors)
        lastBrakeIcon = brakeIcon;
    }


    //Periodically send out the rest of the 1000ms packets if no changes in value have occurred.
    if(millis() - timer250ms < 250) return;     //If we haven't hit 250ms yet, don't send the packets
    timer250ms = millis();                      //Reset the timer

    controller.send(ABS_CAN_ADDR, (brakeIcon ? 0x40:0x00), 0, 0, 0, 0, 0, 0, 0x08);                         //Spoof Anti-Lock brakes (All 0's clears errors)
    controller.send(POWER_STEER_CAN_ADDR, 0, powerSteerState, powerSteeringPrompt, 0, 0, 0, 0, 0);     //Spoof for Power Steering, byte 1 controls steering wheel icon on cluster
    controller.send(PARK_ASSIST_CAN_ADDR, 0, 0, 0, 0, 0, 0, 0, 0);
    controller.send(MOTOR_SPOOF_CAN_ADDR, 0, 0, 0, 0, 0, 0, rpmGauge/200, rpmGauge%200);                 //Motor spoof for RPM dial
    controller.send(TRANSMISSION_CAN_ADDR, 0, otherGear, 0, 0, (gearNumber << 4), driveSet, 0, driveModifier);           //Transmission controller spoof, sets drive gear and sport/eco/normal modes

    //Update the last values for the next time we send out the 250ms packets
    lastBrakeIcon = brakeIcon;
}

void CamryCluster_CAN::send1000msPackets(ICANController &controller){
    
    //Calculations for CAN Bus values based on the flags in the class
    uint8_t lowACC = 0;                                         //Default to not a low accessory battery
    if(chargingSystemMalfunction) lowACC = 0x04;                //If ACC battery is low, display prompt on LCD
    else if(oilPressureLow) lowACC = 0x03;                      //Show oil pressure low if battery is OK

    uint8_t engineFault = checkEngineOn ? 0x00:0x40;                  //0x40 turns off check engine light from instrument cluster
    if(!clusterBacklight) engineFault = checkEngineOn ? 0xB0:0xC0;    //Turn off backlight if needed

    uint8_t dashAnimationMask = animateStartup ? 0x00:0x40;
    dashAnimationMask += trunkOpen + (rearLeftDoor << 2) + (rearRightDoor << 3) + (frontRightDoor << 4) + (frontLeftDoor << 5);

    uint8_t precollisionMask = crashBrakePrompt ? 0x10 : 0x00;

    uint8_t hudLaneMask = hudBlueLeftLane + (hudBlueRightLane << 1) + ((hudLeftLaneColor && 0x3) << 2) + ((hudRightLaneColor && 0x3) << 4);

    //Allow for instant update if values have changed for the headlight or highbeam
    if(headlight != last_headlight || highbeam != last_highbeam){
        controller.send(LIGHTING_CAN_ADDR, 0x12, 0, 0xE8,(headlight << 5) + (highbeam << 6), 0, 0, 0, 0); //Send out spoof for headlights/high beam system when headlight switches have changed
        last_headlight = headlight;
        last_highbeam = highbeam;
    }

    //Allow for instant update if values have changed for the seatbelt icon
    static bool lastSeatBeltIcon;
    if(lastSeatBeltIcon != seatBeltIcon){
        controller.send(AIRBAG_CAN_ADDR, 0, 0, 0, (seatBeltIcon ? 0x05:0x00), 0, 0, 0, 0);                   //Spoof SRS Airbag system (All 0's clears errors)
        lastSeatBeltIcon = seatBeltIcon;
    }

    static uint8_t lastEngineFault;
    static uint8_t lastLowACC;
    static uint16_t lastMotorTemp;
    if(lastEngineFault != engineFault || lastLowACC != lowACC || lastMotorTemp != motorTempDegC){
        controller.send(ENGINE_CONTROL_CAN_ADDR, engineFault,lowACC,(motorTempDegC*1.59)+65,0,0,0,0,0);      //Spoof for engine controller. Takes a flag that sets check engine, alternator failure and motor temperature
        lastEngineFault = engineFault;
        lastLowACC = lowACC;
        lastMotorTemp = motorTempDegC;
    }

    static uint8_t lastEngineStoppedCode;
    static uint8_t lastCheckEnginePrompt;
    if(lastEngineStoppedCode != LCD_EngineStoppedCode || lastCheckEnginePrompt != LCD_CheckEnginePrompt){
        controller.send(ENGINE_PROMPTS_CAN_ADDR, 0, 0 , 0, 0, 0, 0, LCD_EngineStoppedCode, LCD_CheckEnginePrompt);
        lastEngineStoppedCode = LCD_EngineStoppedCode;
        lastCheckEnginePrompt = LCD_CheckEnginePrompt;
    }
    
    //Sort-of approximation for temperature. Gets within +/- 1 degree on display.
    float tempC = ((double)(outsideTemperatureF - 32) * 5.0) / 9.0;
    float temperatureMaskUpper;
    float temperatureMaskLower = std::modf(tempC, &temperatureMaskUpper);
    temperatureMaskLower = temperatureMaskLower * 100;
    temperatureMaskUpper += 48;


    //Periodically send out the rest of the 1000ms packets if no changes in value have occurred.
    if(millis() - timer1000ms < 1000) return;   //If we haven't hit 1000ms yet, don't send the packets
    timer1000ms = millis();                     //Reset the timer

    controller.send(AIRBAG_CAN_ADDR, 0, 0, 0, (seatBeltIcon ? 0x05:0x00), 0, 0x08, 0, 0xC5);             //Spoof SRS Airbag system (All 0's clears errors)
    controller.send(LANE_DEPART_CAN_ADDR, hudLaneMask, 0, 0, 0, 0, 0, LCD_TakeBreak_Prompt, 10);          //Lane departure spoof
    controller.send(PRECOLLISION_CAN_ADDR, precollisionMask, 0, 0, clusterBeeps, 0, 0, 0, 0);            //Precollision spoof
    controller.send(PARKING_CAN_ADDR, 1, 1, 1, 1, 0, 0, 0, 0);                                           //Parking sonar spoof
    controller.send(LIGHTING_CAN_ADDR, 0x12, 0, 0xE8,(headlight << 5) + (highbeam << 6), 0, 0, 0, 0);    //Send out spoof for headlights/high beam system when headlight switches have changed
    controller.send(ENGINE_CONTROL_CAN_ADDR, engineFault,lowACC,(motorTempDegC*1.59)+65,0,0,0,0,0);      //Spoof for engine controller. Takes a flag that sets check engine, alternator failure and motor temperature
    controller.send(SMART_KEY_CAN_ADDR, 0x81, 0, 0, 0, 0, 0, LCD_PowerPrompt, LCD_PowerPrompt ? 0x0D:0); //Smart Key and Push to Start instructions
    controller.send(ANIMATIONS_CAN_ADDR, 0x10, 0, 0, 0, LCD_Brightness, dashAnimationMask, 0x08, seatBeltIcon ? 0x50:0x00);    //Spoof for instrument cluster animations and backlight dimming
    controller.send(ENGINE_PROMPTS_CAN_ADDR, 0, 0 , 0, 0, 0, 0, LCD_EngineStoppedCode, LCD_CheckEnginePrompt);
    controller.send(OUTDOOR_TEMP_CAN_ADDR, 0, 0, 0, (uint8_t)temperatureMaskUpper, 0, (uint8_t)temperatureMaskLower, 0, 0);

    //Update the last values for the next time we send out the 1000ms packets
    last_headlight = headlight;
    last_highbeam = highbeam;
    lastEngineFault = engineFault;
    lastLowACC = lowACC;
    lastMotorTemp = motorTempDegC;
    lastEngineStoppedCode = LCD_EngineStoppedCode;
    lastCheckEnginePrompt = LCD_CheckEnginePrompt;

}

/// @brief Taskes the locally populated fields and generates the CAN bus frames needed to spoof the Camry Cluster components.
/// @param controller The CAN bus controller attached to this microcontroller.
void CamryCluster_CAN::sendCANData(ICANController &controller){
    //SEE THIS SHEET FOR HOW THE SPOOF WORKS: https://docs.google.com/spreadsheets/d/1bL61UoguuONFQnytRpy7xj2nyJdYXmgT9HQCQns6Ij0/edit?usp=sharing
    
    send25msPackets(controller);
    send250msPackets(controller);
    send1000msPackets(controller);

}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////         APP CONTROL FUNCTIONS         //////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

/// @brief Constructor for AppController_CAN. Sets the board address (if needed in future).
AppController_CAN::AppController_CAN(uint32_t boardAddr) {
    boardAddress = boardAddr;  // Set the CAN bus address for this controller
}

/// @brief Initializes the control fields of the App Controller to default values.
void AppController_CAN::initialize() {
    usingAppControl = false;
    leftTurnSignal = false;
    rightTurnSignal = false;
    headlight = false;
    highbeam = false;
    horn = false;
    hazards = false;
    stereo = true;           //Default ON when flashed
    ipadCharger = true;      //Default ON when flashed
    runningLights = true;    //Default ON when flashed - Dash Controller's persisted preference overrides this after first boot
    Acc = false;
    Ign = false;
    FullStart = false;
    driveMode = 0;
    occupantFanPWM = 0;
    boardDetected = false;
}

void AppController_CAN::sendCANData(ICANController &controller) {
    byte tx0 = (leftTurnSignal ? 1 : 0)
             | ((rightTurnSignal ? 1 : 0) << 1)
             | ((headlight ? 1 : 0) << 2)
             | ((highbeam ? 1 : 0) << 3)
             | ((horn ? 1 : 0) << 4)
             | ((hazards ? 1 : 0) << 5)
             | ((stereo ? 1 : 0) << 6)
             | ((ipadCharger ? 1 : 0) << 7);
    byte tx1 = (Acc ? 1 : 0)
             | ((Ign ? 1 : 0) << 1)
             | ((FullStart ? 1 : 0) << 2);
    byte tx2 = driveMode;
    byte tx3 = (usingAppControl ? 1 : 0) | ((runningLights ? 1 : 0) << 1);
    controller.send(boardAddress, tx0, tx1, tx2, tx3, occupantFanPWM, 0, 0, 0);
}

void AppController_CAN::receiveCANData(CANBusMessage msg) {
    if(msg.addr == boardAddress) {
        boardDetected = true;
        leftTurnSignal = msg.bytes[0] & 0x01;
        rightTurnSignal = (msg.bytes[0] >> 1) & 0x01;
        headlight = (msg.bytes[0] >> 2) & 0x01;
        highbeam = (msg.bytes[0] >> 3) & 0x01;
        horn = (msg.bytes[0] >> 4) & 0x01;
        hazards = (msg.bytes[0] >> 5) & 0x01;
        stereo = (msg.bytes[0] >> 6) & 0x01;
        ipadCharger = (msg.bytes[0] >> 7) & 0x01;
        Acc = msg.bytes[1] & 0x01;
        Ign = (msg.bytes[1] >> 1) & 0x01;
        FullStart = (msg.bytes[1] >> 2) & 0x01;
        driveMode = msg.bytes[2];
        usingAppControl = msg.bytes[3] & 0x01;  // Extract the usingAppControl flag from byte3
        runningLights = (msg.bytes[3] >> 1) & 0x01;  // Extract the app-requested running lights preference from byte3
        occupantFanPWM = msg.bytes[4];
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////         CAN BUS CONTROLLER SUBMODULE         ///////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

/// @brief Converts baud rate from MCP_CAN_RK library macro to true Particle speed
/// @param baudRate Baud rate in bits per second.
unsigned long convertBaudRateToParticle(unsigned long baudRate){
    if(baudRate <= CAN_1000KBPS){   //Check if this is a MCP_CAN_RK baud rate     
        switch (baudRate){
        case CAN_1000KBPS:
            return 1000000;
        case CAN_500KBPS:
            return 500000;
        case CAN_250KBPS:
            return 250000;
        case CAN_200KBPS:
            return 200000;
        case CAN_125KBPS:
            return 125000;
        case CAN_100KBPS:
            return 100000;
        case CAN_50KBPS:
            return 50000;
        default:
            return 500000;  //Use 500kbps CAN if using unrecognized macro
        }
    }
    return baudRate;    //If baudRate > CAN_1000KBPS, then assumes we're already in Particle format
}

/// @brief Converts baud rate from MCP_CAN_RK library macro to true Particle speed
/// @param baudRate Baud rate in bits per second.
unsigned long convertBaudRateToMCP(unsigned long baudRate){
    if(baudRate > CAN_1000KBPS){   //Check if this is a MCP_CAN_RK baud rate     
        switch (baudRate){
        case 1000000:
            return CAN_1000KBPS;
        case 500000:
            return CAN_500KBPS;
        case 250000:
            return CAN_250KBPS;
        case 200000:
            return CAN_200KBPS;
        case 125000:
            return CAN_125KBPS;
        case 100000:
            return CAN_100KBPS;
        case 50000:
            return CAN_50KBPS;
        default:
            return CAN_500KBPS;  //Use 500kbps CAN if using unrecognized macro
        }
    }
    return baudRate;            //If baudRate <= CAN_1000KBPS, then assumes we're already in MCP format
}


