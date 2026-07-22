#ifndef DECENTRALIZED_LV_BOARDS_H
#define DECENTRALIZED_LV_BOARDS_H

#include "Particle.h"
#include <mcp_can.h>
#include "API/CAN/CANBusMessage.h"
#include "API/CAN/ICANController.h"

//////////////////////////////////////////////////////////////////////////////////////////////////
// MACROS FOR SYSTEM OPERATION
#define DRIVE_MODE_PARK     0
#define DRIVE_MODE_FORWARD  1
#define DRIVE_MODE_SPORT    3
#define DRIVE_MODE_ECO      5
#define DRIVE_MODE_REVERSE  8
#define DRIVE_MODE_NEUTRAL  16
//////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////////////
// FAULT NETWORK
// Single, shared definition of the car-wide fault model. A fault is defined once here, at the CAN
// layer, and flows unchanged through every hop (CAN -> BLE -> iPad -> cellular -> server -> pit web).
// Every board sets its own fault bits; the Power Controller aggregates them and forwards to the app.
//
// Severity drives how a fault is surfaced (lights/sounds on the car, banner colour in the app/pit).
#define FAULT_SEV_NONE      0   // no fault
#define FAULT_SEV_INFO      1   // informational state, not a problem (e.g. charging active)
#define FAULT_SEV_CAUTION   2   // advisory - worth noting, not urgent
#define FAULT_SEV_WARNING   3   // degraded - needs attention soon
#define FAULT_SEV_CRITICAL  4   // safety/shutdown - immediate attention

// Board / subsystem identifiers. Also used as the heartbeat board ID and as the fault "source".
// Keep in sync with the app's board list (AppStatus::boardStatus order) when adding entries.
#define BOARD_ID_POWER      0
#define BOARD_ID_DASH       1
#define BOARD_ID_HV         2
#define BOARD_ID_IBOOSTER   3
#define BOARD_ID_BMS        4
#define BOARD_ID_RMS        5
#define BOARD_ID_LPDRV_FL   6   // front-left corner driver (BDFL)
#define BOARD_ID_LPDRV_FR   7   // front-right corner driver (BDFR)
#define BOARD_ID_LPDRV_RL   8   // rear-left corner driver (BDRL)
#define BOARD_ID_LPDRV_RR   9   // rear-right corner driver (BDRR)
#define BOARD_COUNT         10

// Board liveness state (matches the app's boardStatus encoding).
#define BOARD_STATE_OFF     0   // not powered / not expected
#define BOARD_STATE_ONLINE  1   // heartbeat received within the timeout window
#define BOARD_STATE_FAULT   2   // expected but silent past BOARD_ONLINE_TIMEOUT_MS

// How long a board may go silent before it is flagged as a fault (ms).
#define BOARD_ONLINE_TIMEOUT_MS   1000
// How often boards without another periodic frame should emit their heartbeat (ms).
#define BOARD_HEARTBEAT_PERIOD_MS 200
//////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////////////
// MACROS FOR CAMRY CLUSTER

//LCD Power sequence prompts
#define LCD_POWER_GOOD                  0x00    //Value for LCD_PowerPrompt to clear all prompts
#define LCD_TURN_POWER_OFF              0x01    //Value for LCD_PowerPrompt to show "Turn Power Off Before Exiting Vehicle"
#define LCD_SHIFT_PARK_BEFORE_EXITING   0x02    //Value for LCD_PowerPrompt to show "Shift to Park Before Exiting Vehicle"
#define LCD_HYBRID_SYSTEM_STOPPED       0x04    //Value for LCD_PowerPrompt to show "Hybrid System Stopped"
#define LCD_SHIFT_TO_NEUTRAL_RESTART    0x08    //Value for LCD_PowerPrompt to show "Shift to Neutral and Push Engine Switch to Restart"
#define LCD_PRESS_BRAKE_PREDAL_AND_PTS  0x30    //Value for LCD_PowerPrompt to show the turn on instructions "Press Brake Pedal and Push Power Switch to Start"
#define LCD_KEY_DETECTED_IN_VEHICLE     0x40    //Value for LCD_PowerPrompt to show "Key Not Detected In Vehicle"
#define LCD_IGNITION_PROMPT             0x50    //Value for LCD_PowerPrompt to show that we're in ignition. "Not ready to drive" with icon for brake pedal and push to start

//LCD Brightness values
#define LCD_BRIGHTNESS_LOW              0xF0    //Value for LCD_Brightness to set brightness to low
#define LCD_BRIGHTNESS_HIGH             0xB0    //Value for LCD_Brightness to set brightness to high

// Engine Stopped LCD codes
#define LCD_ENGINE_NORMAL               0x00    //Value for LCD_EngineStoppedCode to not show engine prompt
#define LCD_ENGINE_STOPPED              0x10    //Value for LCD_EngineStoppedCode to show "Engine Stopped, stop in a safe place"
#define LCD_ENGINE_STOPPED_BEEP         0x1B    //Value for LCD_EngineStoppedCode to show "Engine Stopped, stop in a safe place" and make beeping noise.

//LCD Check Engine Prompts
#define LCD_CHECK_ENGINE_NONE           0x00    //Value for LCD_CheckEnginePrompt to show no check engine light or message
#define LCD_CHECK_ENGINE                0x10    //Value for LCD_CheckEnginePrompt to show "Check Engine" on the LCD
#define LCD_CHECK_ENGINE_REDUCED        0x30    //Value for LCD_CheckEnginePrompt to show "Reduced Engine Power" on the LCD
#define LCD_CHECK_ENGINE_MAINTENANCE    0x40    //Value for LCD_CheckEnginePrompt to show "Engine Maintenance Required" on the LCD

//LCD Parking Brake Prompts
#define LCD_PBRK_GOOD                   0x00    //Value for LCD_ParkingBrakePrompt to show no parking brake warning
#define LCD_PBRK_MAY_ROLL               0x10    //Value for LCD_ParkingBrakePrompt to show "Parking Brake May Roll" on the LCD
#define LCD_PBRK_AUTO_APPLY             0x20    //Value for LCD_ParkingBrakePrompt to show "Parking Brake Automatically Applied" on the LCD
#define LCD_PBRK_BRAKE_ON               0x30    //Value for LCD_ParkingBrakePrompt to show "Parking Brake On" on the LCD
#define LCD_PBRK_NO_DISENGAGE           0x40    //Value for LCD_ParkingBrakePrompt to show "Parking Brake Unable to Disengage" on the LCD
#define LCD_PBRK_SHIFT_INTERLOCK_EN     0x50    //Value for LCD_ParkingBrakePrompt to show "Shift Interlock Activated" on the LCD
#define LCD_PBRK_SHIFT_INTERLOCK_DIS    0x60    //Value for LCD_ParkingBrakePrompt to show "Shift Interlock Deactivated" on the LCD
#define LCD_PBRK_UNABLE_DISENGAGE       0x70    //Value for LCD_ParkingBrakePrompt to show "Parking Brake Unable to Disengage" on the LCD
#define LCD_PBRK_UNAVAIL_ROLL           0x80    //Value for LCD_ParkingBrakePrompt to show "Parking Brake Unavailable, May Roll" on the LCD
#define LCD_PBRK_UNAVAIL                0x90    //Value for LCD_ParkingBrakePrompt to show "Parking Brake Unavailable" on the LCD
#define LCD_PBRK_TEMP_UNAVAIL           0xA0    //Value for LCD_ParkingBrakePrompt to show "Parking Brake Temporarily Unavailable" on the LCD
#define LCD_PBRK_MALFUNCTION            0xB0    //Value for LCD_ParkingBrakePrompt to show "Parking Brake Malfunction" on the LCD
#define LCD_PBRK_PBRK_UNAVAIL           0xC0    //Value for LCD_ParkingBrakePrompt to show "Parking Brake Unavailable" on the LCD
#define LCD_PBRK_PBRK_MALFUNCTION       0xD0    //Value for LCD_ParkingBrakePrompt to show "Parking Brake Malfunction" on the LCD  

//LCD Take a Break Prompts
#define LCD_TAKE_BREAK_NONE             0x00    //Value for LCD_TakeBreak_Prompt to show no take a break message
#define LCD_PLEASE_TAKE_BREAK           0x40    //Value for LCD_TakeBreak_Prompt to show "Would you like to take a Break" on the LCD
#define LCD_TAKE_BREAK_WARN             0x80    //Value for LCD_TakeBreak_Prompt to show "Please take a Break" on the LCD

//Instrument cluster beep rate macros
#define BEEP_RATE_OFF                   0x00    //Value for clusterBeeps to turn off the beep
#define BEEP_RATE_CONTINUOUS            0x01    //Value for clusterBeeps to beep continuously
#define BEEP_RATE_1                     0x02    //Value for clusterBeeps to beep at rate 1 (Fastest)
#define BEEP_RATE_2                     0x03    //Value for clusterBeeps to beep at rate 2 (Faster)
#define BEEP_RATE_3                     0x04    //Value for clusterBeeps to beep at rate 3 (Fast)
#define BEEP_RATE_4                     0x05    //Value for clusterBeeps to beep at rate 4 (Medium)
#define BEEP_RATE_5                     0x06    //Value for clusterBeeps to beep at rate 5 (Slow) 
#define BEEP_RATE_6                     0x07    //Value for clusterBeeps to beep at rate 6 (Slower)
#define BEEP_RATE_7                     0x08    //Value for clusterBeeps to beep at rate 7 (Slowest)

//HUD Lane Departure Macros
#define HUD_LANE_OFF                    0x00    //Value for hudLeftLaneColor or hudRightLaneColor to turn off the lane departure indicators
#define HUD_LANE_CLEAR                  0x01    //Value for hudLeftLaneColor or hudRightLaneColor have the lane show a clear outline
#define HUD_LANE_WHITE                  0x02    //Value for hudLeftLaneColor or hudRightLaneColor have the lane be filled white
#define HUD_LANE_ORANGE                 0x03    //Value for hudLeftLaneColor or hudRightLaneColor have the lane flash orange

//CAN Message Addresses
#define SPEED_CAN_ADDR                  0xB4    //Address to send to the instrument cluster to fake the speedometer
#define ABS_CAN_ADDR                    0x3B7   //Address to send to the instrument cluster to fake the anti-lock brakes
#define AIRBAG_CAN_ADDR                 0x3B1   //Address to send to the instrument cluster to fake the airbag system
#define ENGINE_CONTROL_CAN_ADDR         0x3BB   //Address to send to the instrument cluster to fake the engine controller
#define POWER_STEER_CAN_ADDR            0x394   //Address to send to the instrument cluster to fake the power steering system
#define PARKING_BRAKE_CAN_ADDR          0x32C   //Address to send to the instrument cluster to fake the parking brake
#define PARK_ASSIST_CAN_ADDR            0x378   //Address to send to the instrument cluster to fake the power steering system
#define LANE_DEPART_CAN_ADDR            0x412   //Address to send to the instrument cluster to fake the lane departure system
#define PRECOLLISION_CAN_ADDR           0x411   //Address to send to the instrument cluster to fake the precollision system
#define PARKING_CAN_ADDR                0x43A   //Address to send to the instrument cluster to fake the parking assist system
#define SMART_KEY_CAN_ADDR              0x633   //Address to send to the instrument cluster to fake the smart key system
#define MOTOR_SPOOF_CAN_ADDR            0x1EA   //Address to send to the instrument cluster to fake the motor controller
#define TRANSMISSION_CAN_ADDR           0x3BC   //Address to send to the instrument cluster to fake the transmission controller
#define ANIMATIONS_CAN_ADDR             0x620   //Address to send to the instrument cluster to fake the animations
#define FUEL_ECONOMY_CAN_ADDR           0x1C4   //Address to send to the instrument cluster to fake the fuel economy system
#define LIGHTING_CAN_ADDR               0x622   //Address to send to the instrument cluster to fake the lighting controller
#define ENGINE_PROMPTS_CAN_ADDR         0x400   //Address to send to the instrument cluster to fake the engine unit
#define OUTDOOR_TEMP_CAN_ADDR           0x3B0   //Address to send to the instrument cluster to fake the outdoor temperature

//////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////
//Dashboard Controller CAN Message Format. UPDATE THIS WHEN YOU ADD FIELDS OR ADDITIONAL CAN DATA!
#define DASH_CONTROL_ADDR   0x99
// byte 0: Right Turn PWM 0-255
// byte 1: Left Turn PWM 0-255
// byte 2: animationTick
// byte 4: b0:headlight b1:highbeam b2:runningLights b3:bmsFaultDetected b4:rmsFaultDetected b5:reversePress b6:eyesMode (app-toggled "eyes" animation override on BDFL - takes priority over all other front-grid output, never persisted)
// byte 5: Radiator Fan PWM 0-255
// byte 6: Drive Mode: b0: Drive, b1: Sport, b2: Eco, b3: Reverse, b4: Neutral (BPS fault)
// byte 7: Radiator pump
// EXAMPLE FRAME: CANSend(0x99, 0xFF, 0xFF, 0x00, 0xFF, 0x03, 0x00, 0x01, 0x03);
// - Right and Left turn signal, headlight, and highbeam are on (at full brightness for L and R signal)
// - Car is not in low power mode, not doing startup animations
// - Car is in Drive Forward mode
// - Radiator Fan and Pump are both on
///////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////
//Power Controller CAN Message Format. UPDATE THIS WHEN YOU ADD FIELDS OR ADDITIONAL CAN DATA!
#define POWER_CONTROL_ADDR   0x120
// byte 0: 
// byte 1: 
// byte 2: 
// byte 3: 
// byte 4:
// byte 5:
// byte 6:
// byte 7: 
///////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////
//Orion BMS Message Forwarding Format for the HV pack statistics. UPDATE THIS WHEN YOU ADD FIELDS OR ADDITIONAL CAN DATA!
#define ORION_PACK_STAT_ADDR   0x112
// byte 0: pack current (upper 8 bits) (0.1A increments)
// byte 1: pack current (lower 8 bits) (0.1A increments)
// byte 2: pack voltage (upper 8 bits) (0.1V increments)
// byte 3: pack voltage (lower 8 bits) (0.1V increments)
// byte 4: pack amp-hours (0.1Ah increments)
// byte 5: pack resistance (1mOhm increments)
// byte 6: pack state of charge (0-100%)
// byte 7: 12V voltage (0.1V increments)
///////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////
//Orion BMS Message Forwarding Format for cell stats and DTC error codes. UPDATE THIS WHEN YOU ADD FIELDS OR ADDITIONAL CAN DATA!
#define ORION_DTC_CELLV_ADDR   0x113
// byte 0: average cell voltage (0.1V increments)
// byte 1: highest cell voltage (0.1V increments)
// byte 2: lowest cell voltage (0.1V increments)
// byte 3: lowest cell resistance (0.1mOhm increments)
// byte 4: DTC error flags 1 (upper 8 bits)
// byte 5: DTC error flags 1 (lower 8 bits)
// byte 6: DTC error flags 2 (upper 8 bits)
// byte 7: DTC error flags 2 (lower 8 bits)
///////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////
//Orion BMS Message Forwarding Format for the current limits and cell temperatures. UPDATE THIS WHEN YOU ADD FIELDS OR ADDITIONAL CAN DATA!
#define ORION_CUR_LMT_TEMP_ADDR   0x114
// byte 0: discharge current limit (upper 8 bits) (1A increments)
// byte 1: discharge current limit (lower 8 bits) (1A increments)
// byte 2: charge current limit (upper 8 bits) (1A increments)
// byte 3: charge current limit (lower 8 bits) (1A increments)
// byte 4: BMS average temperature (degrees C)
// byte 5: BMS internal temperature (degrees C)
// byte 6: Highest thermistor temp (degrees C)
// byte 7: Lowest thermistor temp (degrees C)
///////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////
//Orion BMS Message Forwarding Format for the J1772 charger. UPDATE THIS WHEN YOU ADD FIELDS OR ADDITIONAL CAN DATA!
#define ORION_J1772_STATS_ADDR   0x115
// byte 0: J1772 charger connected
// byte 1: J1772 Charger Current Limit (1V increments)
// byte 2: J1772 AC Charger Voltage (1V increments)
// byte 3: relayState (upper 8 bits) - Orion contactor/relay bitmask
// byte 4: relayState (lower 8 bits)
// byte 5: failsafeStatuses - Orion failsafe bitmask (b0 voltage, b1 current, b2 relay, b3 cell balancing)
// byte 6:
// byte 7:
///////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////
//RMS Motor Controller Message Forwarding Format for voltages and currents. UPDATE THIS WHEN YOU ADD FIELDS OR ADDITIONAL CAN DATA!
#define RMS_POWER_STAT_ADDR   0x116
// byte 0: 12V accessory Voltage (upper 8 bits) (0.01V increments)
// byte 1: 12V accessory Voltage (lower 8 bits) (0.01V increments)
// byte 2: HV Bus Voltage (upper 8 bits) (0.1V increments)
// byte 3: HV Bus Voltage (lower 8 bits) (0.1V increments)
// byte 4: HV Bus Current (upper 8 bits) (0.1A increments)
// byte 5: HV Bus Current (lower 8 bits) (0.1A increments)
// byte 6: Motor Phase A Current (upper 8 bits) (0.1A increments)
// byte 7: Motor Phase A Current (lower 8 bits) (0.1A increments)
///////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////
//RMS Motor Controller Message Forwarding Format for motor RPM, torque and temperature and inverter temperature. UPDATE THIS WHEN YOU ADD FIELDS OR ADDITIONAL CAN DATA!
#define RMS_MTR_TEMP_ADDR   0x117
// byte 0: Motor RPM (upper 8 bits) (RPM increments, 1 RPM = 1 increment)
// byte 1: Motor RPM (lower 8 bits) (RPM increments, 1 RPM = 1 increment)
// byte 2: Commanded Torque (upper 8 bits) (0.1Nm increments, 1 Nm = 10 increments)
// byte 3: Commanded Torque (lower 8 bits) (0.1Nm increments, 1 Nm = 10 increments)
// byte 4: Motor Temperature C (upper 8 bits) (degrees C, signed, 0.1C increments)
// byte 5: Motor Temperature C (lower 8 bits) (degrees C, signed, 0.1C increments)
// byte 6: Inverter Temperature C (upper 8 bits) (degrees C, signed, 0.1C increments)
// byte 7: Inverter Temperature C (lower 8 bits) (degrees C, signed, 0.1C increments)
///////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////
//RMS Motor Controller Message Forwarding Format for error codes. UPDATE THIS WHEN YOU ADD FIELDS OR ADDITIONAL CAN DATA!
#define RMS_POST_FAULTS_ADDR   0x118
// byte 0: Post Faults (High) (upper 8 bits) (see RMS documentation for fault codes)
// byte 1: Post Faults (High) (lower 8 bits) (see RMS documentation for fault codes)
// byte 2: Post Faults (Low) (upper 8 bits) (see RMS documentation for fault codes)
// byte 3: Post Faults (Low) (lower 8 bits) (see RMS documentation for fault codes)
// byte 4: Run Faults (High) (upper 8 bits) (see RMS documentation for fault codes)
// byte 5: Run Faults (High) (lower 8 bits) (see RMS documentation for fault codes) 
// byte 6: Run Faults (Low) (upper 8 bits) (see RMS documentation for fault codes)
// byte 7: Run Faults (Low) (lower 8 bits) (see RMS documentation for fault codes) 
///////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////
//HV Controller CAN Message Format. UPDATE THIS WHEN YOU ADD FIELDS OR ADDITIONAL CAN DATA!
#define HV_CONTROL_ADDR   0x130
// byte 0: b0: Killswitch b1: BMSFault b2: dischargeContactorOn b3: chargeContactorOn b4: chargeSafetyOn b5: rmsFaultActive b6: contactorMismatch
// byte 1: packSOC
// byte 2: motorTemperatureC (upper 8 bits, 0.1C increments)
// byte 3: motorTemperatureC (lower 8 bits)
// byte 4: inverterTemperatureC (upper 8 bits, 0.1C increments)
// byte 5: inverterTemperatureC (lower 8 bits)
// byte 6: thermistorHighTempC (hottest pack cell, degrees C) - front-left LPDRV ramps the battery-box fan from this
// byte 7: bmsFailsafe (Orion failsafe bitmask passthrough: b0 voltage, b1 current, b2 relay, b3 cell balancing)
///////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////
//Rear-Left LPDRV (corner driver) CAN Message Format. UPDATE THIS WHEN YOU ADD FIELDS OR ADDITIONAL CAN DATA!
#define REAR_LEFT_DRIVER   0x95
// byte 0: bmsFaultInput (rear-left reads the BMS fault line)
// byte 1: switchFaultInput (rear-left reads the manual kill-switch fault line)
// byte 2:
// byte 3:
// byte 4:
// byte 5:
// byte 6:
// byte 7:
///////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////
// Corner LPDRV heartbeat/fault frames. The three corner boards that historically transmitted nothing
// (front-left, front-right, rear-right) now each emit a heartbeat on their own address so the Power
// Controller can monitor their liveness like every other board. Same byte layout for all three.
#define FRONT_LEFT_DRIVER    0x96
#define FRONT_RIGHT_DRIVER   0x97
#define REAR_RIGHT_DRIVER    0x98
// byte 0: board ID (BOARD_ID_LPDRV_FL / _FR / _RR) so the receiver knows which corner spoke
// byte 1: local fault severity summary (FAULT_SEV_*)
// byte 2: output fault bitmap low  (per-output open-load/short flags; 0 until sense hardware exists)
// byte 3: output fault bitmap high
// byte 4: b0 bmsFaultInput, b1 switchFaultInput (from IP pins where wired; 0 otherwise)
// byte 5:
// byte 6:
// byte 7:
///////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////
//iBooster Controller CAN Message Format. UPDATE THIS WHEN YOU ADD FIELDS OR ADDITIONAL CAN DATA!
#define IBOOSTER_ADDR   0x214
// byte 0:
// byte 1: 
// byte 2: 
// byte 3: 
// byte 4:
// byte 5: Amount the pedal has been pressed down
// byte 6:
// byte 7: 
///////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////
//App Controller CAN Message Format. UPDATE THIS WHEN YOU ADD FIELDS OR ADDITIONAL CAN DATA!
#define APPCONTROL_ADDR   0x101
// byte 0: b0: leftTurnSignal, b1: rightTurnSignal, b2: headlight, b3: highbeam, b4: horn, b5: hazards, b6: stereo, b7: ipadCharger
// byte 1: b0: Acc, b1: Ignition, b2: FullStart
// byte 2: driveMode (see DRIVE_MODE_* macros)
// byte 3: b0: usingAppControl, b1: runningLights (app-requested preference), b2: batteryFanOverride, b3: eyesMode ("eyes" animation override, never persisted)
// byte 4: occupantFanPWM
// byte 5: batteryFanManualPWM (fan speed to use while batteryFanOverride is set)
// byte 6: ledStripBrightness (interior dash strip brightness, 0-255; app-controlled dimmer, defaults to 255)
// byte 7:
///////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////
// Music Light-Show CAN Message Format. UPDATE THIS WHEN YOU ADD FIELDS OR ADDITIONAL CAN DATA!
// The iPad app is the "conductor": it plays the song and, using the audio playback position as the
// master clock, streams one of these frames per lighting cue over BLE -> Power Controller -> CAN.
// Each board renders ITS OWN zone locally (so we send coarse per-zone cues, not per-pixel streams).
// The app also sends a keepalive/control cue (zone SHOW_ZONE_CONTROL) a few times a second carrying
// showActive; a board treats the show as OFF if showActive is cleared OR no cue arrives for
// SHOW_MODE_TIMEOUT_MS, so a dropped BLE link always fails safe back to normal lighting.
#define SHOW_CONTROL_ADDR   0x102
// byte 0: zone (SHOW_ZONE_*). SHOW_ZONE_CONTROL (0xFF) is a keepalive - carries showActive, renders nothing.
// byte 1: b0 showActive (1 = a show is running, 0 = stop show / return to normal lighting)
// byte 2: red   (0-255)   - ignored by PWM-only zones (front/rear side markers)
// byte 3: green (0-255)
// byte 4: blue  (0-255)
// byte 5: intensity (0-255) - overall brightness for this cue
// byte 6: effect (SHOW_FX_*) - how the receiving board animates toward the cue color
// byte 7: seq - rolling cue counter, for debug/dedup only
//
// Zones - one per addressable/controllable lighting area on the car:
#define SHOW_ZONE_INTERIOR    0   // Dash Controller 100px WS2815 interior/dash strip (full RGB)
#define SHOW_ZONE_EYES        1   // Front-left LPDRV (BDFL) 32x8 RGB matrices          (full RGB)
#define SHOW_ZONE_REAR        2   // Rear-left  LPDRV (BDRL) tail/brake/turn strips     (full RGB)
#define SHOW_ZONE_FRONT_SIDE  3   // Front-right LPDRV (BDFR) PWM side marker    (brightness only, color ignored)
#define SHOW_ZONE_REAR_SIDE   4   // Rear-right  LPDRV (BDRR) PWM side marker    (brightness only, color ignored)
#define SHOW_ZONE_BULBS       5   // Dash-driven headlight/highbeam/running bulbs (on/off strobe)
#define SHOW_ZONE_CONTROL     0xFF // Keepalive/control cue - sets showActive, renders nothing
#define SHOW_ZONE_COUNT       6    // Number of renderable zones (0..5)
// Effects (byte 6). Each board renders these locally, allowing a compact CAN cue to drive a
// complete RGBIC pattern without per-pixel traffic. PWM-only zones use the timing envelope.
#define SHOW_FX_SOLID         0   // hold the cue colour until the next cue
#define SHOW_FX_PULSE         1   // snap to the cue colour, then decay toward black
#define SHOW_FX_STROBE        2   // hard on/off flash of the cue colour
#define SHOW_FX_SWEEP         3   // one broad travelling band
#define SHOW_FX_CHASE         4   // several narrow travelling bands
#define SHOW_FX_RIPPLE        5   // expanding waves from the centre
#define SHOW_FX_THEATER_CHASE 6   // alternating pixel groups, shifting on each frame
#define SHOW_FX_SPARKLE       7   // deterministic glitter over a dim colour wash
// A board leaves show mode if no show cue is heard for this long (ms). The app's keepalive rate must
// be comfortably faster than this. Fails safe: a dropped BLE link returns the car to normal lighting.
#define SHOW_MODE_TIMEOUT_MS  600
///////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////
// Battery-box cooling-fan temperature ramp.
// The front-left LPDRV board (BDFL) drives the physical battery-box fan on LP6 from this ramp using
// the hottest pack thermistor (HVController_CAN::thermistorHighTempC, received over CAN). The Power
// Controller uses the same helper to report the fan level to the app. Kept here so there is one
// single source of truth for the curve. NOTE: the app can override this for testing/validation via
// AppController_CAN::batteryFanOverride - that override is never persisted.
#define BATT_FAN_TEMP_ON     30      // deg C - fans begin spinning (lithium cells like to stay cool)
#define BATT_FAN_TEMP_FULL   45      // deg C - fans at full speed
#define BATT_FAN_MIN_PWM     60      // Minimum PWM once spinning, so fans reliably start (0-255)

/// @brief Maps the highest cell/pack temperature to a battery-box fan PWM value.
///        Returns 0 below BATT_FAN_TEMP_ON, ramps from BATT_FAN_MIN_PWM up to 255 between
///        BATT_FAN_TEMP_ON and BATT_FAN_TEMP_FULL, then holds 255.
/// @param tempC Highest cell/thermistor temperature in degrees Celsius.
uint8_t battTempToPWM(float tempC);
///////////////////////////////////////////////////////////////////////////////////////////////////





unsigned long convertBaudRateToParticle(unsigned long baudRate);
unsigned long convertBaudRateToMCP(unsigned long baudRate);

/// @brief Class to send data from Dash Controller OR to receive CAN data from the Dash Controller on other boards.
class DashController_CAN{
    public:
    uint32_t boardAddress;      //The CAN Bus address that this controller runs at, should be defined by DASH_CONTROL_ADDR
    byte rightTurnPWM;          //Brightness of the right turn signal. Value ranges from 0 (fully off) to 255 (fully on).
    byte leftTurnPWM;           //Brightness of the left turn signal. Value ranges from 0 (fully off) to 255 (fully on).
    byte occupantFanPWM;      //Occupant-cell fan speed (front cabin fans), 0 (off) to 255 (max). Set by the app; carried on Dash frame byte3. Battery-box fans are now driven by the HV Controller.
    byte frontLeftFan1PWM;
    byte frontLeftFan2PWM;
    byte frontRightFanPWM;
    bool headlight;             //Toggle switch for the car headlights. True turns on headlights, false turns off headlights.
    bool highbeam;              //Toggle switch for the car highbeams. True turns on highbeams, false turns off highbeams.
    bool runningLights;         //Final running-lights output state (app-toggleable preference ANDed with car power state). Drive corner board outputs from this.
    bool eyesMode;              //App-toggled "eyes" animation override for the front matrix headlights (BDFL). When true, BDFL renders the eyes animation instead of headlight/highbeam/running-light/blinker output - eyes take full priority. App-only preference, never persisted: always defaults false on boot. Carried on Dash frame byte4 bit6.
    bool reversePress;          //Toggle switch for being in reverse mode. Use to turn on/off reverse lights, backup camera, etc.
    byte driveMode;             //The gear that the user has requested (Park, Reverse, Forward, ...). Use the macros like DRIVE_MODE_PARK, DRIVE_MODE_NORMAL, etc.
    byte radiatorFanPWM;           //Cooling fan for the motor controller. NOTE: byte7 on the wire only has 1 free bit for this, so sendCANData/receiveCANData only transmit it as on/off (0 or 255), not true PWM.
    bool radiatorPump;          //Toggle to control the cooling pump for the motor controller.
    bool bmsFaultDetected;      //Flag that is set true if a Battery Management System fault has been detected.
    bool rmsFaultDetected;      //Flag that is set true if a Motor Controller fault has been detected.
    bool boardDetected;         //Flag set true in receiveCANData when a message from the Dash Controller has been received. Use this on other boards to check if you're hearing from the Dash Controller.
    uint8_t animationTick;      //A tick counter that is used to synchronize animations across the system. Increments every 10ms, resets to 0 after reaching 255.

    DashController_CAN(uint32_t boardAddr);
    void initialize();
    void sendCANData(ICANController &controller);
    void receiveCANData(CANBusMessage msg);
    
};

/// @brief Class to send data from Power Controller OR to receive CAN data from the Power Controller on other boards.
class PowerController_CAN{
    public:
    uint32_t boardAddress;      //The CAN Bus address that this controller runs at, should be defined by DASH_CONTROL_ADDR
    bool BrakeSense;            //Flag indicating if the brake pedal is being pressed.
    bool PushToStart;           //Flag indicating if the push to start button is being pressed.
    bool ACCharge;              //Flag indicating if the car is being charged from the wall.
    bool SolarCharge;           //Flag indicating if the car is in solar charge mode.
    bool Horn;                  //Flag indicating if the horn is being pressed.
    bool Acc;                   //Flag indicating if the car has its Accessory busbar active.
    bool Ign;                   //Flag indicating if the car has its Ignition busbar active.
    bool FullStart;
    bool CarOn;
    bool StartUp;               
    bool LowPowerMode;          //Flag indicating to the rest of the system that we are operating in Low Power Mode. Use this to update controls of other boards!
    bool LowACCBattery;         //Flag indicating that the 12V accessory is low (true) or normal (false).
    bool boardDetected;         //Flag set true in receiveCANData when a message from the Power Controller has been received. Use this on other boards to check if you're hearing from the Power Controller.
    bool usingAppControl;       // New field: true if using app control, false otherwise

    PowerController_CAN(uint32_t boardAddr);
    void initialize();
    void sendCANData(ICANController &controller);
    void receiveCANData(CANBusMessage msg);

};

class LPDRV_RearLeft_CAN{
    public:
    uint32_t boardAddress;      //The CAN Bus address that this controller runs at, should be defined by DASH_CONTROL_ADDR
    bool bmsFaultInput;         //This board reads in the Battery Management System fault line and tells the rest of the system if we have a fault.
    bool switchFaultInput;      //This board reads in the manual kill switch fault line and tells the rest of the system if we have a fault.
    bool boardDetected;         //Flag set true in receiveCANData when a message from the Power Controller has been received. Use this on other boards to check if you're hearing from the Power Controller.
    LPDRV_RearLeft_CAN(uint32_t boardAddr);
    void initialize();
    void sendCANData(ICANController &controller);
    void receiveCANData(CANBusMessage msg);
};

/// @brief Heartbeat/fault frame for the corner LPDRV boards that otherwise transmit nothing
///        (front-left, front-right, rear-right). Each corner constructs one with its own board
///        address (FRONT_LEFT_DRIVER / FRONT_RIGHT_DRIVER / REAR_RIGHT_DRIVER) and board ID, and
///        calls sendCANData periodically (every BOARD_HEARTBEAT_PERIOD_MS) so the Power Controller
///        can track its liveness. The Power Controller constructs one per corner (with matching
///        address) and calls receiveCANData to read the fault fields back off the bus.
class LPDRVCorner_CAN{
    public:
    uint32_t boardAddress;      //CAN address this corner runs at (FRONT_LEFT_DRIVER / FRONT_RIGHT_DRIVER / REAR_RIGHT_DRIVER)
    uint8_t boardID;            //Which corner this is (BOARD_ID_LPDRV_FL / _FR / _RR)
    uint8_t faultSeverity;      //Local fault severity summary (FAULT_SEV_*). Highest active local fault.
    uint16_t outputFaultBitmap; //Per-output open-load/short flags. 0 until sense hardware exists.
    bool bmsFaultInput;         //BMS fault line read from an IP pin (0 if not wired on this corner).
    bool switchFaultInput;      //Kill-switch fault line read from an IP pin (0 if not wired on this corner).
    bool boardDetected;         //Set true in receiveCANData when this corner's heartbeat has been heard.

    LPDRVCorner_CAN(uint32_t boardAddr, uint8_t id);
    void initialize();
    void sendCANData(ICANController &controller);
    void receiveCANData(CANBusMessage msg);
};

/// @brief Class to send data from Dash Controller to Camry Instrument Cluster.
class CamryCluster_CAN{
    private:
    bool last_headlight;                //Internal flag to check last state of headlight before sending again to cluster
    bool last_highbeam;                 //Internal flag to check last state of highbeam before sending again to cluster
    uint32_t timer25ms;                 //Internal timer to keep track of 25ms packets
    uint32_t timer250ms;                //Internal timer to keep track of 250ms packets
    uint32_t timer1000ms;               //Internal timer to keep track of 1000ms packets
    void send25msPackets(ICANController &controller);       //Send all the packets needed every 25ms to the instrument cluster
    void send250msPackets(ICANController &controller);      //Send all the packets needed every 250ms to the instrument cluster
    void send1000msPackets(ICANController &controller);     //Send all the packets needed every 1000ms to the instrument cluster

    public:
    bool brakeIcon;                     //Set true to turn on red BRAKE text on instrument cluster, false to turn off. See spreadsheet linked in CamryCluster_CAN::sendCANData for details.
    bool parkingBrakeCircle;            //Set true to turn on yellow circle with exclamation mark, false to turn off. See spreadsheet linked in CamryCluster_CAN::sendCANData for details.
    bool seatBeltIcon;                  //Set true to turn on red seat belt icon on instrument cluster, false to turn off. See spreadsheet linked in CamryCluster_CAN::sendCANData for details.
    bool checkEngineOn;                 //Set true to turn on check engine indicator on instrument cluster, false to turn off. See spreadsheet linked in CamryCluster_CAN::sendCANData for details.
    bool clusterBacklight;              //Set true to turn on backlight on instrument cluster, false to turn off. See spreadsheet linked in CamryCluster_CAN::sendCANData for details.
    bool oilPressureLow;                //Set true to turn on low oil pressure prompt on instrument cluster, false to turn off. See spreadsheet linked in CamryCluster_CAN::sendCANData for details.
    bool chargingSystemMalfunction;     //Set true to turn on low accessory battery indicator on instrument cluster, false to turn off. See spreadsheet linked in CamryCluster_CAN::sendCANData for details.
    uint16_t motorTempDegC;             //Set to the motor temperature in degrees Celcius. Changes position of temperature dial on instrument cluster. See spreadsheet linked in CamryCluster_CAN::sendCANData for details.
    bool powerSteeringIcon;             //Set true to turn on power steering icon on instrument cluster, false to turn off. See spreadsheet linked in CamryCluster_CAN::sendCANData for details.
    uint8_t powerSteeringPrompt;        //Set value to show power steering prompt on LCD. See spreadsheet linked in CamryCluster_CAN::sendCANData for details.
    uint8_t LCD_PowerPrompt;            //Set value to show powerup prompt on LCD. Use the macros such as LCD_HYBRID_SYSTEM_STOPPED and LCD_IGNITION_PROMPT to set the value. See spreadsheet linked in CamryCluster_CAN::sendCANData for details.
    uint8_t LCD_Brightness;             //Set the brightness of the cluster (low or high). Use macros LCD_BRIGHTNESS_HIGH and LCD_BRIGHTNESS_LOW. See spreadsheet linked in CamryCluster_CAN::sendCANData for details.
    bool trunkOpen;                     //Set true to show the trunk being open on the LCD. See spreadsheet linked in CamryCluster_CAN::sendCANData for details.
    bool frontLeftDoor;                 //Set true to show the front left door being open on the LCD. See spreadsheet linked in CamryCluster_CAN::sendCANData for details.
    bool frontRightDoor;                //Set true to show the front right door being open on the LCD. See spreadsheet linked in CamryCluster_CAN::sendCANData for details.
    bool rearLeftDoor;                  //Set true to show the rear left door being open on the LCD. See spreadsheet linked in CamryCluster_CAN::sendCANData for details.
    bool rearRightDoor;                 //Set true to show the rear right door being open on the LCD. See spreadsheet linked in CamryCluster_CAN::sendCANData for details.
    bool animateStartup;                //Set true to show a fancy animation on the LCD when powering on. See spreadsheet linked in CamryCluster_CAN::sendCANData for details.
    uint8_t LCD_EngineStoppedCode;      //Set value to show engine error codes. Use macros LCD_ENGINE_NORMAL, LCD_ENGINE_STOPPED, LCD_ENGINE_STOPPED_BEEP. See spreadsheet linked in CamryCluster_CAN::sendCANData for details.
    uint8_t LCD_CheckEnginePrompt;      //Set value to show check engine codes. Use macros LCD_CHECK_ENGINE_NONE, LCD_CHECK_ENGINE, LCD_CHECK_ENGINE_REDUCED, LCD_CHECK_ENGINE_MAINTENANCE. See spreadsheet linked in CamryCluster_CAN::sendCANData for details.
    uint8_t LCD_ParkingBrakePrompt;     //Set value to show parking brake error codes. Use macros LCD_PBRK_GOOD, LCD_PBRK_BRAKE_ON, etc. See spreadsheet linked in CamryCluster_CAN::sendCANData for details.
    uint16_t rpmGauge;                  //Set value to the motor RPM. Changes the instrument cluster gauge.
    uint16_t speedGauge;                //Set value to the vehicle speed. Changes the instrument cluster gauge.
    uint8_t ecoGauge;                   //Percentage of bars on the economy display. Ranges from 0 to 100.
    bool ecoLeaf;                       //Set the ECO leaf to show on the bottom right of the LCD. 
    bool fogLightOrange;                //Set true to turn on orange fog light indicator on instrument cluster, false to turn off. See spreadsheet linked in CamryCluster_CAN::sendCANData for details.
    bool fogLightGreen;                 //Set true to turn on green fog light indicator on instrument cluster, false to turn off. See spreadsheet linked in CamryCluster_CAN::sendCANData for details.
    bool headlight;                     //Set true to turn on headlight indicator on instrument cluster, false to turn off. See spreadsheet linked in CamryCluster_CAN::sendCANData for details.
    bool highbeam;                      //Set true to turn on high beam indicator on instrument cluster, false to turn off. See spreadsheet linked in CamryCluster_CAN::sendCANData for details.
    uint8_t driveMode;                  //Set to the drive mode the Dashboard Controller is in such as DRIVE_MODE_PARK, DRIVE_MODE_FORWARD, DRIVE_MODE_REVERSE, etc. See spreadsheet linked in CamryCluster_CAN::sendCANData for details.
    uint8_t gearNumber;                 //Set this to a sport gear (1-10) if you're feeling adventurous. Shows sport gear next to drive mode on LCD. See spreadsheet linked in CamryCluster_CAN::sendCANData for details.
    bool sportMode;                     //Turns on sport mode on the instrument cluster (makes top banner red and shows sport text on bottom). See spreadsheet linked in CamryCluster_CAN::sendCANData for details.
    bool ecoMode;                       //Turns on eco mode on the instrument cluster (makes top banner blue and shows ECO Mode text on bottom). See spreadsheet linked in CamryCluster_CAN::sendCANData for details.
    bool readyToDrive;                  //Set true if the car is ready to move forward/backwards. Allows shifting to occur. See spreadsheet linked in CamryCluster_CAN::sendCANData for details.
    bool crashBrakePrompt;              //Set true to show the crash brake prompt on the LCD. See spreadsheet linked in CamryCluster_CAN::sendCANData for details.
    uint8_t clusterBeeps;               //Set if the cluster should be making noise. Use the BEEP_RATE macros to set the value. See spreadsheet linked in CamryCluster_CAN::sendCANData for details.
    bool hudBlueLeftLane;               //Show the blue left lane departure on the HUD. Can be shown at same time as the regular left lane departure.
    bool hudBlueRightLane;              //Show the blue right lane departure on the HUD. Can be shown at same time as the regular right lane departure.
    uint8_t hudLeftLaneColor;           //Set the color of the left lane departure on the HUD. Use the macros HUD_LANE_OFF, HUD_LANE_CLEAR, HUD_LANE_WHITE, HUD_LANE_ORANGE. See spreadsheet linked in CamryCluster_CAN::sendCANData for details.
    uint8_t hudRightLaneColor;          //Set the color of the left lane departure on the HUD. Use the macros HUD_LANE_OFF, HUD_LANE_CLEAR, HUD_LANE_WHITE, HUD_LANE_ORANGE. See spreadsheet linked in CamryCluster_CAN::sendCANData for details.
    uint8_t LCD_TakeBreak_Prompt;       //Set the value to show the "Take a Break" prompt on the LCD. See spreadsheet linked in CamryCluster_CAN::sendCANData for details.
    int outsideTemperatureF;            //Set the outside temperature in degrees F. Shows on the LCD.

    void initialize();
    void sendCANData(ICANController &controller);
    //void receiveCANData(CANBusMessage msg);
};

/// @brief Class to send data from HV Controller OR to receive CAN data from the HV Controller on other boards.
class HVController_CAN{
    public:
    uint32_t boardAddress;      //The CAN Bus address that this controller runs at, should be defined by HV_CONTROL_ADDR
    bool Killswitch;                  //Killswitch on the outside of the car
    bool BMSFault;                    //Indicator for a fault in the BMS
    bool boardDetected;                    //Flag to ensure we have heard from the board
    bool dischargeContactorOn;        //Reads from the Orion CANBUS to determine if the discharge contactor is enabled
    bool chargeContactorOn;           //Reads from the Orion CANBUS to determine if the charge contactor is enabled
    bool chargeSafetyOn;              //Reads from the Orion CANBUS to determine if the charge safety contactor is enabled
    bool rmsFaultActive;              //Rolled-up motor-controller (RMS) fault flag, so consumers of 0x130 get one fault bit without decoding 0x118. Carried on byte0 bit5.
    bool contactorMismatch;           //True when a contactor the HV Controller commanded does not match the Orion's reported relay state (stuck/failed contactor). Carried on byte0 bit6.
    uint8_t bmsFailsafe;              //Orion failsafe status bitmask passthrough (see failsafeStatuses). Carried on byte7 so 0x130 consumers see it without decoding 0x115.
    uint8_t packSOC;                  //This is a copy from the OrionBMS packSOC field. Putting this here so you only need the HVController to see this stat and not all of OrionBMS.
    float motorTemperatureC;          //This is a copy from the RMS motorTemperatureC field. Putting this here so you only need the HVController to see this stat and not all of RMSController.
    float inverterTemperatureC;       //This is a copy from the RMS inverterTemperatureC field. Putting this here so you only need the HVController to see this stat and not all of RMSController.
    uint8_t thermistorHighTempC;      //This is a copy from the OrionBMS thermistorHighTempC field. Putting this here so you only need the HVController to see this stat and not all of OrionBMS. The front-left LPDRV board ramps the battery-box fan from this value (see battTempToPWM).

    HVController_CAN(uint32_t boardAddr);
    void initialize();
    void sendCANData(ICANController &controller);
    void receiveCANData(CANBusMessage msg);

};

/// @brief Class to send data from HV Controller OR to receive CAN data from the HV Controller on other boards.
class IBOOSTER_CAN{
    public:
    uint32_t boardAddress;      //The CAN Bus address that this controller runs at, should be defined by HV_CONTROL_ADDR
    uint8_t brakePercentage;    //Percentage the brake pedal has been pressed down
    bool boardDetected;                    //Flag to ensure we have heard from the board


    IBOOSTER_CAN(uint32_t boardAddr);
    void initialize();
    //void sendCANData(ICANController &controller); No controls yet
    void receiveCANData(CANBusMessage msg);

};

/// @brief Class to send data from the in-app controls to the system. It should only contain the fields that the app itself controls, not the telemetry that the app only receives.
class AppController_CAN{
    public:
    uint32_t boardAddress;      //The CAN Bus address that this controller runs at, should be defined by DASH_CONTROL_ADDR
    bool usingAppControl;    //Flag to indicate if the app is controlling the car. If false, the car is controlled by the Dash Controller/PowerController buttons.
    bool leftTurnSignal;
    bool rightTurnSignal;
    bool headlight;
    bool highbeam;
    bool horn;
    bool hazards;            //Hazard lights state
    bool stereo;             //Stereo power state (default ON)
    bool ipadCharger;        //iPad/Cigarette lighter charger power state (default ON)
    bool telemetry;          //Telemetry radio power state (default ON). App-toggleable low-power output on the Dash Controller. Carried on App frame byte1 bit3.
    bool radio;              //Ham/comms radio power state (default ON). App-toggleable low-power output on the Dash Controller. Carried on App frame byte1 bit4.
    bool wiper;              //Windshield wiper power state (default OFF). App-toggleable low-power output on the Dash Controller. Carried on App frame byte1 bit5.
    bool runningLights;      //App-requested running lights preference (on/off). Dash Controller persists this and ANDs it with car power state.
    bool eyesMode;           //App-toggled "eyes" animation override for the front matrix headlights. TESTING/NOVELTY FEATURE - never persisted, always defaults false on boot. Carried on byte3 bit3.
    bool Acc;                //Accessory state
    bool Ign;                //Ignition state
    bool FullStart;          //Full start state (ready to drive)
    uint8_t driveMode;       //Current drive mode (park, reverse, drive, sport, eco, etc.)
    uint8_t occupantFanPWM;  //Occupant-cell fan speed the app is requesting (0=off .. 255=max). Carried on byte4.
    bool batteryFanOverride; //App-requested manual override of the battery-box fan. When true, the HV Controller ignores battery temperature and drives the fan at batteryFanManualPWM. TESTING/VALIDATION ONLY - never persisted, always defaults false on boot. Carried on byte3 bit2.
    uint8_t batteryFanManualPWM; //Battery-box fan speed to drive while batteryFanOverride is true (0=off .. 255=max). Carried on byte5.
    uint8_t ledStripBrightness; //Interior dash LED strip brightness the app is requesting (0=off .. 255=max). Defaults to 255 (full) on boot. Carried on byte6.
    bool boardDetected;       //Flag to ensure we have heard from the board

    AppController_CAN(uint32_t boardAddr);
    void initialize();
    void sendCANData(ICANController &controller);
    void receiveCANData(CANBusMessage msg);
};

/// @brief Music light-show cue frame (SHOW_CONTROL_ADDR). Sent by the Power Controller (BLE gateway)
///        as it relays cues from the iPad app; received by every board that renders a lighting zone
///        (Dash Controller interior strip, LPDRV corner boards). Each frame targets a single zone.
///        The receiving board keeps its own per-zone state and animation timing - this class only
///        encodes/decodes the wire format. See the SHOW_* macros above for the byte layout.
class ShowController_CAN{
    public:
    uint32_t boardAddress;   //Always SHOW_CONTROL_ADDR. The zone lives in the payload, not the address.
    bool showActive;         //True while a show is running. When a received frame clears this, boards return to normal lighting.
    uint8_t zone;            //Which zone this cue targets (SHOW_ZONE_*), or SHOW_ZONE_CONTROL for a keepalive.
    uint8_t red;             //Cue color red   (0-255). Ignored by PWM-only zones.
    uint8_t green;           //Cue color green (0-255).
    uint8_t blue;            //Cue color blue  (0-255).
    uint8_t intensity;       //Overall cue brightness (0-255).
    uint8_t effect;          //How to animate toward the cue color (SHOW_FX_*).
    uint8_t seq;             //Rolling cue counter (debug/dedup only).
    bool freshCue;           //Set true by receiveCANData when a show frame was just consumed; the caller
                             //clears it after copying the cue into its per-zone state. Lets a board that
                             //renders multiple zones (e.g. the Dash: interior + bulbs) demux by zone.
    bool boardDetected;      //Set true once any show frame has been heard on the bus.

    ShowController_CAN(uint32_t boardAddr);
    void initialize();
    /// @brief Build and transmit one show cue frame. Called by the gateway (Power Controller).
    void sendCue(ICANController &controller, uint8_t cueZone, uint8_t r, uint8_t g, uint8_t b, uint8_t cueIntensity, uint8_t cueEffect, bool active, uint8_t sequence);
    /// @brief Decode a received frame into this object. Sets freshCue=true only for SHOW_CONTROL_ADDR frames.
    void receiveCANData(CANBusMessage msg);
};

#endif
