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

	/* ==================================================
This is the definition file for the the UCSC AP code
it creates all the common defines, unions, enumerations
and data types.

 Code by: Mariano I. Lizarraga
 First Revision: Aug 18 2008 @ 17:42
 ====================================================*/
#ifndef _APDEFINITIONS_H_
#define _APDEFINITIONS_H_

#ifdef __cplusplus
       extern "C"{
#endif

// =========== Global Definitions ==========

// Circular Buffer Size
// ===================
#define BSIZE			512

// UAV System ID
// =============
#define SLUGS_SYSTEMID		100
#define SLUGS_COMPID			1

#define GS_SYSTEMID				127
#define GS_COMPID					0

// Cube Model Used
// ===============
#define USE_CUBE_16405	0

// GPS Header IDs
// ==============
#define GGAID			1
#define RMCID			2
#define UNKID			254


// DMA Maximum Char Sending
// ========================
#define MAXSEND					109


// Maximun Number of WPs and PIDs
#define MAX_NUM_WPS		11
#define MAX_NUM_PIDS	10

// Define log raw data at 100 hz. Comment out to have
// XYZ data come at 100 Hz instead. COMMENT not Change to 0 (using #ifdef)
//#define LOGRAW100	1

// Define diagnostic data at 100 hz. Comment out to have
// XYZ data come at 100 Hz instead. COMMENT not Change to 0 (using #ifdef)
//#define DIAG100		1

// ERROR MESSAGES
// ==============

// PID EEPROM Error Messages

enum EEPROM_STATUS{
	EEP_MEMORY_OK,
	EEP_WRITE_FAIL = 10,
	EEP_PAGE_EXP = 20,
	EEP_MEMORY_CORR = 30
};

enum SLUGS_ACTION {
	SLUGS_ACTION_NONE,
	SLUGS_ACTION_SUCCESS,
	SLUGS_ACTION_FAIL,
	SLUGS_ACTION_EEPROM,
	SLUGS_ACTION_MODE_CHANGE,
	SLUGS_ACTION_MODE_REPORT,
	SLUGS_ACTION_PT_CHANGE,
	SLUGS_ACTION_PT_REPORT,
	SLUGS_ACTION_PID_CHANGE,
	SLUGS_ACTION_PID_REPORT,
	SLUGS_ACTION_WP_CHANGE,
	SLUGS_ACTION_WP_REPORT,
	SLUGS_ACTION_MLC_CHANGE,
	SLUGS_ACTION_MLC_REPORT
};

enum WP_PROTOCOL {
	WP_PROT_IDLE,
	WP_PROT_LIST_REQUESTED,
	WP_PROT_NUM_SENT,
	WP_PROT_TX_WP,
	WP_PROT_RX_WP,
	WP_PROT_SENDING_WP_IDLE,
	WP_PROT_GETTING_WP_IDLE
};



// WP EEPROM Error Messages
#define WPSEEP_WRITE_FAIL	21
#define WPSEEP_PAGE_EXP		22
#define WPSEEP_MEMORY_CORR	23

// EEPROM Emulation Address Offsets
// ================================
#define PID_OFFSET		0
#define WPS_OFFSET		60


// Standard characters used in the parsing of messages
// ===================================================
#define DOLLAR		36
#define STAR			42
#define CR				13
#define LF				10
#define AT				64


// Standard Units
// ==============
#define KTS2MPS 		0.514444444
#define PI          3.141592653589793

// Periphereal Configurations
// ==========================
#define APFCY			40000000

	
#define GPSBAUDF		19200
#define GPSBAUDI		4800
#define UCSCAP_UBRGF 	129
#define UCSCAP_UBRGI 	520

#define LOGBAUD			115200
#define LOG_UBRG		21


// ifdef switches for debugging and conditional inclusion
// ======================================================
#define __IN_DSPIC__ 	1 // switch for use in PC

#if __IN_DSPIC__
	#ifdef DEBUG
		#undef DEBUG
	#endif	
#else
	#define DEBUG 1
#endif

// Uncomment if there is no magentometer
#define NO_MAGNETO 

// Uncomment to allow full gyro calibration
#define DO_FULL_CALL


// ============= Unions Used for Data Transmission ====
//Type definitions for standard unions used in sending
// and receiving data
typedef union{
	unsigned char    chData[2];
	unsigned short   usData;
} tUnsignedShortToChar; 

typedef union{
	unsigned char    chData[2];
 	short   		 shData;
} tShortToChar; 

typedef union{
	unsigned char   chData[4];
 	unsigned int   	uiData;
} tUnsignedIntToChar; 

typedef union{
	unsigned char   chData[4];
 	int   			inData;
} tIntToChar; 

typedef union{
	unsigned char   chData[4];
 	float   		flData;
	unsigned short	shData[2];
} tFloatToChar; 

// ============= Structures used for data ============
typedef struct tGpsData{
	unsigned char	 		year;
	unsigned char			month;
	unsigned char			day;
	unsigned char			hour;
	unsigned char			min;
	unsigned char			sec;	 
	tFloatToChar 			lat;
	tFloatToChar 			lon;
	tFloatToChar 			height;
	tUnsignedShortToChar	cog;
	tUnsignedShortToChar	sog;
	tUnsignedShortToChar	hdop;	
	unsigned char			fix;
	unsigned char 			sats;	
	unsigned char			newValue;
}tGpsData;

typedef struct tRawData{
	tShortToChar 	gyroX;
	tShortToChar 	gyroY;
	tShortToChar 	gyroZ;
	tShortToChar 	accelX;
	tShortToChar 	accelY;
	tShortToChar 	accelZ;
	tShortToChar 	magX;
	tShortToChar 	magY;
	tShortToChar 	magZ;
	// included in SLUGS MKII
	tShortToChar 	baro;
	tShortToChar 	pito;
	tShortToChar 	powr;
	tShortToChar 	ther;
}tRawData;

typedef struct tAttitudeData{
	tFloatToChar			roll;
	tFloatToChar			pitch;
	tFloatToChar			yaw;
	tFloatToChar			p;
	tFloatToChar			q;
	tFloatToChar			r;
	tUnsignedShortToChar	timeStamp;
}tAttitudeData;

typedef struct tSensStatus{
	char					load;
	char					vdetect;
	tUnsignedShortToChar	bVolt;
}tSensStatus;

typedef struct tDynTempData{
	tFloatToChar	dynamic;
	tFloatToChar	stat;
	tShortToChar	temp;
}tDynTempData;

typedef struct tBiasData{
	tFloatToChar			axb;
	tFloatToChar			ayb;
	tFloatToChar			azb;
	tFloatToChar			gxb;
	tFloatToChar			gyb;
	tFloatToChar			gzb;	
}tBiasData;

typedef struct tDiagData{
	tFloatToChar			fl1;
	tFloatToChar			fl2;
	tFloatToChar			fl3;
	tShortToChar			sh1;
	tShortToChar			sh2;
	tShortToChar			sh3;	
}tDiagData;

typedef struct tXYZData{
	tFloatToChar	Xcoord;
	tFloatToChar	Ycoord;
	tFloatToChar	Zcoord;	
	tFloatToChar	VX;
	tFloatToChar	VY;
	tFloatToChar	VZ;	
}tXYZData;

typedef struct tAknData{
	unsigned char 	WP;
	unsigned char 	commands;
	unsigned char 	pidCal;
	unsigned char 	sensorReboot;
	unsigned char 	filOnOff;	
	unsigned char	reboot;
}tAknData;

typedef struct tPilotData{
	tUnsignedShortToChar	dt ;
	tUnsignedShortToChar	dla;
	tUnsignedShortToChar	dra;
	tUnsignedShortToChar	dr ; 
	tUnsignedShortToChar	de ; 
}tPilotData;

typedef struct tPWMData{
	tUnsignedShortToChar	dt_c;
	tUnsignedShortToChar	dla_c;
	tUnsignedShortToChar	dra_c;
	tUnsignedShortToChar	dr_c; 
	tUnsignedShortToChar	dle_c; 
	tUnsignedShortToChar	dre_c; 
	tUnsignedShortToChar	dlf_c; 
	tUnsignedShortToChar	drf_c; 
	tUnsignedShortToChar	da1_c; 
	tUnsignedShortToChar	da2_c; 
}tPWMData;

typedef struct tPIDData{
	tFloatToChar   P[10];
	tFloatToChar   I[10];
	tFloatToChar   D[10];	
}tPIDData;


typedef struct tQueryData{
	unsigned char 	pendingRequest;
	unsigned char 	idReq;
	unsigned char 	indxReq;
}tQueryData;

typedef struct tWPData{
	tFloatToChar	lat[11];
	tFloatToChar	lon[11];
	tFloatToChar	hei[11];
	unsigned char	typ[11];
	tShortToChar	val[11];
	unsigned char	wpCount;
}tWPData;

typedef struct tCommandsData{
	tFloatToChar	hCommand;
	tFloatToChar	airspeedCommand;
	
	tFloatToChar	phiCommand;
	tFloatToChar	thetaCommand;
	tFloatToChar	psiCommand;
	
	tFloatToChar	pCommand;
	tFloatToChar	qCommand;
	tFloatToChar	rCommand;
	
	char			currWPCommand;
	char			nextWPCommand;	
}tCommandsData;

typedef struct tAPStatusData{
	unsigned char	controlType;
	unsigned char	beaconStatus;
	unsigned char hilStatus;
	
	unsigned char	dt_pass; 
	unsigned char	dla_pass;
	unsigned char	dra_pass;
	unsigned char	dr_pass; 
	unsigned char	dle_pass;
	unsigned char dre_pass;
	unsigned char	dlf_pass;
	unsigned char	drf_pass;
}tAPStatusData;



typedef struct tNavData {
	tFloatToChar	uMeasured;
	tFloatToChar	thetaCommanded;
	tFloatToChar	psiDotCommanded;
	tFloatToChar	phiCommanded;
	tFloatToChar	rHighPass;
	tFloatToChar	totRun;
	tFloatToChar	distance2Go;
	unsigned char	fromWp;
	unsigned char toWp;	
}tNavData;

typedef struct tSensData{
	tFloatToChar	Ax;
	tFloatToChar	Ay;
	tFloatToChar	Az;
	tFloatToChar	Mx;
	tFloatToChar	My;
	tFloatToChar	Mz;
}tSensData;

typedef struct tLogFloats{
	tFloatToChar	fl1;
	tFloatToChar	fl2;
	tFloatToChar	fl3;
	tFloatToChar	fl4;
	tFloatToChar	fl5;
	tFloatToChar	fl6;
}tLogFloats;

#ifdef __cplusplus
      }
#endif

#endif /* _APDEFINITIONS_H_ */
