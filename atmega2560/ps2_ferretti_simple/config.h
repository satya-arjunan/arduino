
//====================================================================
//Project Lynxmotion Phoenix
//Description: 
//    This is the hardware configuration file for the Hex Robot.
//    This Header file is specific for Phoenix with 3 DOF
//  
//    This version of the Configuration file is set up to run on the
//    Lynxmotion BotboardDuino board, which is similar to the Arduino Duemilanove
//
//    This version of configuration file assumes that the servos will be controlled
//    by a Lynxmotion Servo controller SSC-32 and the user is using a Lynxmotion 
//    PS2 to control the robot.
//
//Date: March 18, 2012
//Programmer: Kurt (aka KurtE)
//
//
//NEW IN V1.0
//   - First Release
//
//====================================================================
#ifndef CONFIG_H
#define CONFIG_H 

//[CONDITIONAL COMPILING] - COMMENT IF NOT WANTED
// Define other optional compnents to be included or not...
//#define OPT_TERMINAL_MONITOR  

#define STABILISATOR
#define STABILISATOR_TRIGGERVALUE 5
//#define USE_EEPROM

#ifdef OPT_TERMINAL_MONITOR   // turning off terminal monitor will turn these off as well...
#define OPT_SSC_FORWARDER  // only useful if terminal monitor is enabled
#define OPT_FIND_SERVO_OFFSETS    // Only useful if terminal monitor is enabled
#endif

//#define OPT_GPPLAYER

// Which type of control(s) do you want to compile in
#define DBGSerial         Serial
//#define DEBUG_IOPINS

#if defined(UBRR1H)
#define SSCSerial         Serial1
#else
#endif

#define USEPS2

//==================================================================================================================================
//==================================================================================================================================
//==================================================================================================================================
// THex-3
//==================================================================================================================================
#define USE_SSC32
//#define	cSSC_BINARYMODE	1			// Define if your SSC-32 card supports binary mode.

//[SERIAL CONNECTIONS]


// Warning I will undefine some components as the non-megas don't have enough memory...
//#undef OPT_FIND_SERVO_OFFSETS 

#define cSSC_BAUD        9600   //SSC32 BAUD rate


#define FEEDBACK_LED   3

//--------------------------------------------------------------------
//[Botboarduino Pin Numbers]
#define SOUND_PIN    2        // Botboarduino JR pin number
#define PS2_DAT      A0        
#define PS2_CMD      A1
#define PS2_SEL      A2
#define PS2_CLK      A3
// If we are using a SSC-32 then:
// If were are running on an Arduino Mega we will use one of the hardware serial port, default to Serial1 above.
// If on Non mega, if the IO pins are set to 0, we will overload the hardware Serial port 
// Else we will user SoftwareSerial to talk to the SSC-32
//#define cSSC_OUT     A6      	//Output pin for (SSC32 RX) on BotBoard (Yellow)
//#define cSSC_IN      A7      	//Input pin for (SSC32 TX) on BotBoard (Blue)
#define cSSC_OUT     1      	//Output pin for (SSC32 RX) on BotBoard (Yellow)
#define cSSC_IN      0      	//Input pin for (SSC32 TX) on BotBoard (Blue)

//====================================================================
//[SSC PIN NUMBERS]
#define LED_EYE_PWM_PIN 0
#define cRFCoxaPin      18   //Front Right leg Hip Horizontal
#define cRFFemurPin     19   //Front Right leg Hip Vertical
#define cRFTibiaPin     20   //Front Right leg Knee
#define cRFTarsPin      1   // Tar

#define cRMCoxaPin      21   //Middle Right leg Hip Horizontal
#define cRMFemurPin     22   //Middle Right leg Hip Vertical
#define cRMTibiaPin     23   //Middle Right leg Knee
#define cRMTarsPin      1   // Tar

#define cRRCoxaPin      27   //Rear Right leg Hip Horizontal
#define cRRFemurPin     28   //Rear Right leg Hip Vertical
#define cRRTibiaPin     29   //Rear Right leg Knee
#define cRRTarsPin      1   // Tar

#define cLFCoxaPin      13   //Front Left leg Hip Horizontal
#define cLFFemurPin     14   //Front Left leg Hip Vertical
#define cLFTibiaPin     15   //Front Left leg Knee
#define cLFTarsPin      1   // Tar

#define cLMCoxaPin      10   //Middle Left leg Hip Horizontal
#define cLMFemurPin     11   //Middle Left leg Hip Vertical
#define cLMTibiaPin     12   //Middle Left leg Knee
#define cLMTarsPin      1   // Tar

#define cLRCoxaPin      4   //Rear Left leg Hip Horizontal
#define cLRFemurPin     5   //Rear Left leg Hip Vertical
#define cLRTibiaPin     6   //Rear Left leg Knee
#define cLRTarsPin      1   // Tar

#define COXA_MinANG -800
#define COXA_MaxANG 800
#define FEMUR_MinANG -800
#define FEMUR_MaxANG 800
#define TIBIA_MinANG -800
#define TIBIA_MaxANG 800
//--------------------------------------------------------------------
//[MIN/MAX ANGLES]
#define cRRCoxaMin1	  COXA_MinANG	//Mechanical limits of the Right Rear Leg, decimals = 1
#define cRRCoxaMax1	  COXA_MaxANG
#define cRRFemurMin1	FEMUR_MinANG
#define cRRFemurMax1	FEMUR_MaxANG
#define cRRTibiaMin1	TIBIA_MinANG
#define cRRTibiaMax1	TIBIA_MaxANG

#define cRMCoxaMin1	  COXA_MinANG	//Mechanical limits of the Right Middle Leg, decimals = 1
#define cRMCoxaMax1	  COXA_MaxANG
#define cRMFemurMin1	FEMUR_MinANG
#define cRMFemurMax1	FEMUR_MaxANG
#define cRMTibiaMin1	TIBIA_MinANG
#define cRMTibiaMax1	TIBIA_MaxANG

#define cRFCoxaMin1	  COXA_MinANG	//Mechanical limits of the Right Front Leg, decimals = 1
#define cRFCoxaMax1	  COXA_MaxANG
#define cRFFemurMin1	FEMUR_MinANG
#define cRFFemurMax1	FEMUR_MaxANG
#define cRFTibiaMin1	TIBIA_MinANG
#define cRFTibiaMax1	TIBIA_MaxANG

#define cLRCoxaMin1	  COXA_MinANG	//Mechanical limits of the Left Rear Leg, decimals = 1
#define cLRCoxaMax1	  COXA_MaxANG
#define cLRFemurMin1	FEMUR_MinANG
#define cLRFemurMax1	FEMUR_MaxANG
#define cLRTibiaMin1	TIBIA_MinANG
#define cLRTibiaMax1	TIBIA_MaxANG

#define cLMCoxaMin1	  COXA_MinANG	//Mechanical limits of the Left Middle Leg, decimals = 1
#define cLMCoxaMax1	  COXA_MaxANG
#define cLMFemurMin1	FEMUR_MinANG
#define cLMFemurMax1	FEMUR_MaxANG
#define cLMTibiaMin1	TIBIA_MinANG
#define cLMTibiaMax1	TIBIA_MaxANG

#define cLFCoxaMin1	  COXA_MinANG	//Mechanical limits of the Left Front Leg, decimals = 1
#define cLFCoxaMax1	  COXA_MaxANG
#define cLFFemurMin1	FEMUR_MinANG
#define cLFFemurMax1	FEMUR_MaxANG
#define cLFTibiaMin1	TIBIA_MinANG
#define cLFTibiaMax1	TIBIA_MaxANG


//--------------------------------------------------------------------
//[LEG DIMENSIONS]
//Universal dimensions for each leg in mm
#define cXXCoxaLength     45    // This is for TH3-R legs
#define cXXFemurLength    35
#define cXXTibiaLength    80   // updated tibia length by Ferretti
#define cXXTarsLength     1    // 4DOF only...

#define cRRCoxaLength     cXXCoxaLength	    //Right Rear leg
#define cRRFemurLength    cXXFemurLength
#define cRRTibiaLength    cXXTibiaLength
#define cRRTarsLength	  cXXTarsLength	    //4DOF ONLY

#define cRMCoxaLength     cXXCoxaLength	    //Right middle leg
#define cRMFemurLength    cXXFemurLength
#define cRMTibiaLength    cXXTibiaLength
#define cRMTarsLength	  cXXTarsLength	    //4DOF ONLY

#define cRFCoxaLength     cXXCoxaLength	    //Rigth front leg
#define cRFFemurLength    cXXFemurLength
#define cRFTibiaLength    cXXTibiaLength
#define cRFTarsLength	  cXXTarsLength    //4DOF ONLY

#define cLRCoxaLength     cXXCoxaLength	    //Left Rear leg
#define cLRFemurLength    cXXFemurLength
#define cLRTibiaLength    cXXTibiaLength
#define cLRTarsLength	  cXXTarsLength    //4DOF ONLY

#define cLMCoxaLength     cXXCoxaLength	    //Left middle leg
#define cLMFemurLength    cXXFemurLength
#define cLMTibiaLength    cXXTibiaLength
#define cLMTarsLength	  cXXTarsLength	    //4DOF ONLY

#define cLFCoxaLength     cXXCoxaLength	    //Left front leg
#define cLFFemurLength    cXXFemurLength
#define cLFTibiaLength    cXXTibiaLength
#define cLFTarsLength	  cXXTarsLength	    //4DOF ONLY


//--------------------------------------------------------------------
//[BODY DIMENSIONS]
#define cRRCoxaAngle1    (-450)       //Default Coxa setup angle, decimals = 1
#define cRMCoxaAngle1    (0   )       //Default Coxa setup angle, decimals = 1
#define cRFCoxaAngle1    (450)       //Default Coxa setup angle, decimals = 1
#define cLRCoxaAngle1    (-450)       //Default Coxa setup angle, decimals = 1
#define cLMCoxaAngle1    (0   )       //Default Coxa setup angle, decimals = 1
#define cLFCoxaAngle1    (450)       //Default Coxa setup angle, decimals = 1


#define HALF_BODY_LENGHT 63
#define HALF_BODY_MIDDLE_WIDTH 50
#define HALF_BODY_FB_WIDTH 45

#define cRROffsetX 	-HALF_BODY_FB_WIDTH     //Distance X from center of the body to the Right Rear coxa
#define cRROffsetZ 	HALF_BODY_LENGHT	      //Distance Z from center of the body to the Right Rear coxa
#define cRMOffsetX 	-HALF_BODY_MIDDLE_WIDTH //Distance X from center of the body to the Right Middle coxa
#define cRMOffsetZ 	0	                      //Distance Z from center of the body to the Right Middle coxa
#define cRFOffsetX 	-HALF_BODY_FB_WIDTH	    //Distance X from center of the body to the Right Front coxa
#define cRFOffsetZ 	-HALF_BODY_LENGHT	      //Distance Z from center of the body to the Right Front coxa

#define cLROffsetX 	HALF_BODY_FB_WIDTH	    //Distance X from center of the body to the Left Rear coxa
#define cLROffsetZ 	HALF_BODY_LENGHT	      //Distance Z from center of the body to the Left Rear coxa
#define cLMOffsetX 	HALF_BODY_MIDDLE_WIDTH	//Distance X from center of the body to the Left Middle coxa
#define cLMOffsetZ 	0	                      //Distance Z from center of the body to the Left Middle coxa
#define cLFOffsetX 	HALF_BODY_FB_WIDTH	    //Distance X from center of the body to the Left Front coxa
#define cLFOffsetZ 	-HALF_BODY_LENGHT	      //Distance Z from center of the body to the Left Front coxa

//--------------------------------------------------------------------
//[START POSITIONS FEET]
//#define cHexInitXZ	 105 
//#define CHexInitXZCos60  53        // COS(60) = .5
//#define CHexInitXZSin60  91        // sin(60) = .866
//#define CHexInitY	 25
#define cHexInitXZ	 100
#define CHexInitXZCos60  ((long)(cHexInitXZ*0.7))        // COS(60) = .5
#define CHexInitXZSin60  ((long)(cHexInitXZ*0.7))        // sin(60) = .866
#define CHexInitY	 40

// Lets try some multi leg positions depending on height settings.
#if 0 // Start first without...
#define CNT_HEX_INITS 3
#define MAX_BODY_Y  90
#ifdef DEFINE_HEX_GLOBALS
const byte g_abHexIntXZ[] PROGMEM = {
  cHexInitXZ, 99, 86};
const byte g_abHexMaxBodyY[] PROGMEM = { 
  20, 50, MAX_BODY_Y};
#else
extern const byte g_abHexIntXZ[] PROGMEM;
extern const byte g_abHexMaxBodyY[] PROGMEM;
#endif
#endif  // if 0
#define cRRInitPosX     CHexInitXZCos60      //Start positions of the Right Rear leg
#define cRRInitPosY     CHexInitY
#define cRRInitPosZ     CHexInitXZSin60

#define cRMInitPosX     cHexInitXZ      //Start positions of the Right Middle leg
#define cRMInitPosY     CHexInitY
#define cRMInitPosZ     0

#define cRFInitPosX     CHexInitXZCos60      //Start positions of the Right Front leg
#define cRFInitPosY     CHexInitY
#define cRFInitPosZ     -CHexInitXZSin60

#define cLRInitPosX     CHexInitXZCos60      //Start positions of the Left Rear leg
#define cLRInitPosY     CHexInitY
#define cLRInitPosZ     CHexInitXZSin60

#define cLMInitPosX     cHexInitXZ      //Start positions of the Left Middle leg
#define cLMInitPosY     CHexInitY
#define cLMInitPosZ     0

#define cLFInitPosX     CHexInitXZCos60      //Start positions of the Left Front leg
#define cLFInitPosY     CHexInitY
#define cLFInitPosZ     -CHexInitXZSin60
//--------------------------------------------------------------------
//[Tars factors used in formula to calc Tarsus angle relative to the ground]
#define cTarsConst	720	//4DOF ONLY
#define cTarsMulti	2	//4DOF ONLY
#define cTarsFactorA	70	//4DOF ONLY
#define cTarsFactorB	60	//4DOF ONLY
#define cTarsFactorC	50	//4DOF ONLY

#endif
