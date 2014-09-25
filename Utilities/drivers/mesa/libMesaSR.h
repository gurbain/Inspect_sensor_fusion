/*------------------------------------------------------------------------*/
/*                                                                        */
/*  Copyright (c) 2008 by MESA Imaging SA,                                */
/*                     http://www.mesa-imaging.ch                         */
/*------------------------------------------------------------------------*/
// $Author$
//    $URL$
//    $Rev$
//   $Date$
/*------------------------------------------------------------------------*/
//!\file
//!contains functions for SR API

#pragma once
#ifndef __LIBMESA_SR__H
#define __LIBMESA_SR__H
#endif

#ifdef __cplusplus
extern "C" {
#endif


#ifdef _WIN32
// The following ifdef block is the standard way of creating macros which make import/export
// from a DLL simpler. All files within this DLL are compiled with the _SR_API_DLL
// symbol defined on the command line. this symbol should not be defined on any project
// that uses this DLL.
  #if defined(_SR_API_DLL)//using file libMesaSR.h, libMesaSR.cpp to generate libMesaSR.dll
    #define SR_API(RETURN) __declspec(dllexport) RETURN __cdecl
  #elif defined(MATLAB_PREPROC)//Preprocessing for matlab interface
    #define SR_API(RETURN) RETURN
  #elif !defined(DOXYGEN) //using file libMesaSR.h with libMesaSR.dll
    #define SR_API(RETURN)  __declspec(dllimport) RETURN __cdecl
  #endif
#else //LINUX
  #define SR_API(RETURN) __attribute__ ((visibility("default"))) RETURN
#endif

#include "definesSR.h"

//!\addtogroup  libMesaSR
//!@{
//--------- interfaces for C++, Delphi etc. ------------
#ifdef _WIN32
//!Performs a check for a new driver release (Windows only).
//!If a new driver is available, a dialog is displayed containing a hyperlink to the download website.
//! It requires internet connection and does not support Autoproxy.
//!- To force the check to the last available version set the mode to 3
//!- if mode = 0 it only does the check if the last check has been done a long time before
//!- if mode & 1 it always checks the version for new drivers
//!-if mode & 2 it also informs about version entries with no info tag
//!
//!if mode is 0  the driver update check can be disabled by setting the registry key:
//!\verbatim
//!HKEY_CURRENT_USER\Software\MesaImaging\Swissranger\nextHttpDllTestDate 
//!to REG_QWORD 0x ff ff ff ff ff ff ff (14 F 's)
//!\endverbatim
SR_API(int) SR_CheckForNewDllVersion(int mode);

//!Opens the Swissranger File and returns a virtual device ID.
//!This can be used for offline development of software using test data streams (.srs files).
//!Once the software is working, a real camera can be substituted for the test stream with minimal changes to the software.
//!Such a file can be generated with with SR_StreamToFile().
//!Any settings that affect the data sent from the camera will be ignored.
//! \return the return value is the number of opened cameras: therefore <=0 means FAILED.
SR_API(int) SR_OpenFile(SRCAM* srCam, const char* filename);

//!SR_StreamToFile opens a file and streams received Data from the camera to that file.
//!This file can subsequently be opened and treated as a virtual camera using SR_OpenFile().
//!The mode can be:
//!- 0 : Open-Create
//!- 1 : Open-Append NOT IMPLEMENTED
//!- 2 : Close
SR_API(int) SR_StreamToFile(SRCAM srCam, const char* filename, int mode);

//!SR_FileStreamCmd jump/setup the reading of srs streams.
SR_API(int) SR_FileStreamCmd(SRCAM srCam, FSCmd cmd, unsigned long data, void* ptr=0);

#endif

#ifdef __BFIN__
//! Generic Open for the Blackfin ported libMesaSR.
SR_API(int) SR_Open(SRCAM* srCam);
SR_API(int) SR_IsFrameAvailable(SRCAM srCam);
#else
//! Generic Open for any kind of camera.
//! This opens a dialog box and lists all available USB and Ethernet swissranger Cameras.
//! One of them can be selected and opened.
//! A reference of the most recently connected camera is stored in the registry. 
//! The mode is used to select how the dialog is handled.
//!
//!the <tt>mode</tt> is used to select how the dialog is handled<br>
//!the mode bits are:
//! - 0 use registry if existing: tries to open the same device as last time
//! - 1 open dialog
//! - 2 open without configuration (internal usage only)
//!
//!so following modes make sense:
//! - 1 use registry, if it failed error is returned, no GUI opened
//! - 2 always open dialog
//! - 3 use registry, if it failed open the GUI dialog
//!
//!The parent is a HWND window handle.
//! \return the return value is the number of opened cameras: therefore <=0 means FAILED.
SR_API(int) SR_OpenDlg(SRCAM* srCam, int mode, int parent);

//!Seeks for all visible devices and opens them all.
//!the caller gives a srCam array with size num.
//! \n\intDoc{function}
//! \return the return value is the number of opened cameras: therefore <=0 means FAILED.
SR_API(int) SR_OpenAll(SRCAM* srCam, unsigned int numSrCam, unsigned long inAddr, unsigned long inMask);

//!Opens the Swissranger USB device and returns a device ID.
//!The serial number is only used, if more than one camera is connected to the computer.\n
//!Setting the serialNumber to 0 opens the first found camera
//!To open a specific camera, serialNumber is set to the last 8 digits of the printed serial number,
//!as a hexadecimal value, e.g. 0x4000002f  for the camera SN: 00-00-40-00-00-2F.
//! \return the return value is the number of opened cameras: therefore <=0 means FAILED.
SR_API(int) SR_OpenUSB(SRCAM* srCam, unsigned int serialNumber);

//!Opens the Swissranger Ethernet device and returns a device ID.
//!The addr is the ip address of the camera. e.g. "192.168.2.14"
//! \return the return value is the number of opened cameras: therefore <=0 means FAILED.
SR_API(int) SR_OpenETH(SRCAM* srCam,  const char* addr);

//!Opens a dialog which allows control of important camera settings.  The parent is an HWND. 
//!The parent is a HWND.
//!The Window is Non Modal and will be destroyed if it is closed or if the camera is closed.
SR_API(int) SR_OpenSettingsDlg(SRCAM srCam, int parent);
#endif //__BFIN__

//! Closes the swissranger device, first releasing the claimed
//! interface.
//!
//! @param srCam a valid device returned by the SR_Open()
//! function.
//! @return a negative number in case of error.
//!  0 success
//! -1 wrong device
//! -2 can't release interface
//! -3 can't close device
SR_API(int) SR_Close(SRCAM srCam);

//!The mode controls a number of characteristics of the image acquisition process, as described in the enumeration #AcquireMode.
//!The mode is an ored value of the enumerator #AcquireMode.
//!The recommended mode is described in #AcquireMode.\n
SR_API(int) SR_SetMode(SRCAM srCam, int mode);

//!Returns the current mode setting. It is an ored value of the enumerator #AcquireMode.
//!\sa SR_SetMode\n
SR_API(int) SR_GetMode(SRCAM srCam);

//!SR_Acquire() triggers image acquisition on the camera and transfers the data from the camera to the PC.
//!The acquired images can be retrieved with SR_GetImage().
//!After this function normally the spherical coordinates are transformed to cartesian coordinates.
//!for that purpose use one of the following functions:\n
//!SR_CoordTrfUint16(), SR_CoordTrfFlt(),SR_CoordTrfDbl()\n
//!The #AcquireMode can be set with the function SR_SetMode().
//!\return the number of transfered bytes or a negative number if failed
//!\sa SR_GetImageList(), SR_SetMode(), #AcquireMode
SR_API(int) SR_Acquire(SRCAM srCam);

//!Transforms spherical coordinates to cartesian coordinates.
//!The pointers x,y,z are unsigned short arrays of the length SR_GetRows() * SR_GetCols().
//!The result values are in mm.
//!The pitches is the distance in byte from one value to the next. Default value is sizeof(unsigned short)=2
//!\sa SR_CoordTrfFlt(),\ref coordTrf
SR_API(int) SR_CoordTrfUint16(SRCAM srCam, short *x, short *y, unsigned short *z, int pitchX,int pitchY,int pitchZ);

//!Transforms spherical coordinates to cartesian coordinates.
//!The pointers x,y,z are float arrays of the length SR_GetRows() * SR_GetCols().
//!The result values are in m.
//!The pitches is the distance in byte from one value to the next. Default value is sizeof(float)=4
//!\sa \ref coordTrf
SR_API(int) SR_CoordTrfFlt(SRCAM srCam, float *x,float *y,float *z, int pitchX,int pitchY,int pitchZ);

//!Transforms spherical coordinates to cartesian coordinates.
//!The pointers x,y,z are double arrays of the length SR_GetRows() * SR_GetCols().
//!The result values are in m.
//!The pitches is the distance in byte from one value to the next. Default value is sizeof(double)=8
//!This function is not more precise than SR_CoordTrfFlt(), but can be useful, if double values are used.
//!\sa SR_CoordTrfFlt(), \ref coordTrf
SR_API(int) SR_CoordTrfDbl(SRCAM srCam, double *x,double *y,double *z, int pitchX,int pitchY,int pitchZ);

//!Transforms spherical coordinates to cartesian coordinates.
//!\sa SR_CoordTrfPntFlt(), \ref coordTrf
SR_API(int) SR_CoordTrfPntUint16(SRCAM srCam, const unsigned char* iX, const unsigned char* iY, const unsigned short* iDst, short *oX, short *oY, unsigned short *oZ, int num);
//!Transforms spherical coordinates to cartesian coordinates.
//!\sa SR_CoordTrfPntFlt(), \ref coordTrf
SR_API(int) SR_CoordTrfPntDbl(SRCAM srCam, const unsigned char* iX, const unsigned char* iY, const unsigned short* iDst, double *oX, double *oY, double *oZ, int num);
//!Transforms spherical coordinates to cartesian coordinates.
//!All pointers are arrays with the length num.
//!The pointers iX,iY,iDst are input pixel coordinates and distance value.
//!The pointers oX,oY,oZ are output cartesian coordinates.
//!\sa SR_CoordTrfPntFlt(), \ref coordTrf
SR_API(int) SR_CoordTrfPntFlt(SRCAM srCam, const unsigned char* iX, const unsigned char* iY, const unsigned short* iDst, float *oX, float *oY, float *oZ, int num);

//!returns the version of this libMesaSR library.
SR_API(int) SR_GetVersion(unsigned short version[4]);

//!SR_SetCallback() can set a user callback function of type \ref SR_FuncCB that is called on special events.
//!Such special events can be:
//! - image buffers will be/have been changed.
//! - a message should be displayed to the user
//! - a register or buffer changes in the camera dialog box
//! - etc. (for a list of all events look at \ref SR_FuncCB)
//!This allows the main program to reallocate image pointers
//!and enter e.g. critical sections to avoid access of freed buffers.
//!
//!\return this function returns the previous function pointer.
//! This gives the possibility to forward not handled message to the default handling
//! \sa \ref SR_FuncCB, SR_GetDefaultCallback()
SR_API (SR_FuncCB*) SR_SetCallback(SR_FuncCB* cb);

//!\return the default callback function.
//! This gives the possibility to forward unhandled message to the API default handling function.
//! \sa \ref SR_FuncCB, SR_SetCallback()
SR_API (SR_FuncCB*) SR_GetDefaultCallback();

//!Returns a identification string of the device:
//!The string is formated as:
//!\verbatim
//!"VendorID:0x%04x, ProductID:0x%04x, Manufacturer:'%s', Product:'%s'" e.g.:
//!VendorID:0x0852, ProductID:0x0071, Manufacturer:'CSEM SA', Product:'3D-SR2 16Bit'
//!\endverbatim
SR_API(int) SR_GetDeviceString(SRCAM srCam, char* buf, int buflen);

//!sets the timeout in ms. This timeout is used at reading and writing on USB or Ethernet port.
//!Any function which communicates with the camera will fail if it exeeds the timeout.
SR_API(void) SR_SetTimeout(SRCAM srCam, int ms);

//!returns the number of rows the camera will deliver
SR_API(unsigned int) SR_GetRows(SRCAM srCam);

//!returns the number of columns the camera will deliver
SR_API(unsigned int)  SR_GetCols(SRCAM srCam);

//!Returns the pointer to the idx-th image. The idx has a value from 0 to SR_GetImageList()-1.
//!Details about which image contains what kind of data can be extracted with SR_GetImageList()
//!Usually index is 0 for distance image and 1 for amplitude image
//!\sa SR_GetRows(), SR_GetCols(), SR_GetImageList(), _ImgEntry::ImgType, #AcquireMode
SR_API(void*) SR_GetImage(SRCAM srCam, unsigned char idx);

//!Returns the number of images available and a pointer to an imgEntryArray.
//!Images may be accessed directly by the data member of the ImEntry elements, or using SR_GetImage() with the corresponding array index.
//!The list of available images can is affected by #AcquireMode that is set with function SR_SetMode()
//!\sa  SR_GetRows(), SR_GetCols(), SR_GetImage(), _ImgEntry::ImgType, #AcquireMode
SR_API(int) SR_GetImageList(SRCAM srCam, ImgEntry** imgEntryArray);

//!Sets the Integration time of the camera
//!The intTime is a value from 0 to 255.
//!The integration time in ms depends on the camera:
//!
//! <TABLE>
//! <tr><td><tt> SR2A           </tt></td><td><tt> intTime*0.256 ms     </tt></td></tr>
//! <tr><td><tt> SR2B,SR3k </tt></td><td><tt> (intTime+1)*0.200 ms </tt></td></tr>
//! <tr><td><tt> SR4k </tt></td><td><tt> 0.300ms+(intTime)*0.100 ms </tt></td></tr>
//! </TABLE>
//!
//! \sa SR_GetIntegrationTime()
SR_API(int) SR_SetIntegrationTime(SRCAM srCam, unsigned char intTime);

//!Gets the Integration time of the camera
//!The intTime is a value from 0 to 255.
//!The integration time in ms depends on the camera
//! \sa SR_SetIntegrationTime()
SR_API(unsigned char) SR_GetIntegrationTime(SRCAM srCam);

//!Sets the dual integration time of the camera<br>
//!The ratio is a value from 0 to 100.<br>
//!This mode requires FW version >= 0x73 and SR4k camera type<br>
//! \sa SR_SetIntegrationTime()
SR_API(int) SR_SetDualIntegrationTime(SRCAM srCam, int ratio);

//!Sets the Amplitude Threshold of the camera.
//!The default amplitude threshold is 0. Setting this value will set all distance values to 0 if
//!their amplitude is lower than the amplitude threshold
//! \sa SR_GetAmplitudeThreshold()
SR_API(int) SR_SetAmplitudeThreshold(SRCAM srCam, unsigned short val);

//!Gets the Amplitude Threshold of the camera
//! \sa SR_SetAmplitudeThreshold()
SR_API(unsigned short) SR_GetAmplitudeThreshold(SRCAM srCam);

//!Sets the modulation frequency of the LEDs.
//!This also changes the full-scale (0xFFFF) measurement range of the raw distance.<br>
//!The supported frequencies depend on the camera.<br>
//!A table of the supported frequency is listed in \ref ModulationFrq .<br>
//!SR_SetModulationFrequency allows simultaneous operation of multiple cameras 
//!without interference by using different frequencies, e.g. using 29, 30 and 31MHz.
//!\return this function returns:
//! - 0 if the frequency change was successful.
//! - a negative value if it failed.
//(NOT YET SUPPORTED) - 0 if there are calibrated data for that frequency
//(NOT YET SUPPORTED) - 1 if the frequency is set but there are no calibrated data for that frequency.
//!\sa SR_GetModulationFrequency(), ModulationFrq
SR_API(int) SR_SetModulationFrequency(SRCAM srCam, enum ModulationFrq modFrq);

//!Gets the modulation frequency of the LEDs. This determins the measurement range.
//!\sa SR_SetModulationFrequency(), ModulationFrq
SR_API(enum ModulationFrq) SR_GetModulationFrequency(SRCAM srCam);

//!Sets the Distance Offset. This function should only be used on SR2 cameras.
//!On newer cameras the distance offset correction is handeled with a camera specific calibration file.
//!\sa SR_GetDistanceOffset()
SR_API(int) SR_SetDistanceOffset(SRCAM srCam, unsigned short distOfs);

//!Gets the Distance Offset.  This function should only be used on SR2 cameras.
//!On newer cameras the distance offset correction is handeled with a camera specific calibration file.
//!\sa SR_SetDistanceOffset()
SR_API(unsigned short) SR_GetDistanceOffset(SRCAM srCam);

//!Turns the AutoExposure on/off and set the desired parameters.
//!if <tt>minIntTime=0xff</tt> the AutoExposure is turned off.<br>
//!good values are:
//! - for SR3k: 5,255,10,45
//! - for SR4k: 1,150,5,70
//!The function sets the integration time to a value between <tt>minIntTime</tt> and <tt>maxIntTime</tt>.<br>
//!Therefore a special position is searched in the intensity histogram (reduced from 16bit to 8bit) described as <tt>percentOverPos</tt>.<br>
//!\image html autoillum.png
//! <tt>percentOverPos</tt> (values 0-100) is the x-position of the histogram where <tt>percentOverPos</tt> percent of the points
//!are above. The integration time is adapted slowly to move this position to the <tt>desiredPos</tt>(value 0-255)intensity value.
//!In case of background light saturation saturation the histogram is not a linear function. This can lead to an instable,
//!swinging system, if there are strong background light in the image.
SR_API(int) SR_SetAutoExposure(SRCAM srCam, unsigned char minIntTime, unsigned char maxIntTime,
                               unsigned char percentOverPos, unsigned char desiredPos);


//!Returns the serial number of the camera
//!- SR4k USB: The serial number is written on the back of the camera (in hex-Format)
//!- SR4k USB: The serial number is written on the back of the camera (in hex-Format). It is the Mac address
SR_API(unsigned int) SR_ReadSerial(SRCAM srCam);

//!@}

//!\internal
//!\addtogroup  libMesaSRextend
//!@{
//!\warning
//!This module contains extended functions for non comon customers.\n
//!These functions are only for user specific cameras that have a different hardware or firmware.\n
//!These functions must not be used without explicit instructions from Mesa-Imaging\n
//!Wrong usage of these functions can lead to unexpected errors.\n

//!\internal
//!Sets a desired register value of the camera. The register values are given to the customer only for special usages.
//! @param srCam a valid device id returned by the SR_Open()
//! function.
//! @param reg the register of the descriptor to write to.
//! @param val the value to send to the given descriptor.
//! @return the number of bytes sent to the camera, which should be 2,
//! or a negative number if an error occurs.
//! -1 wrong device
//! -2 error in request frame (usb_bulk_write)
//!\sa SR_GetReg()
//!\warning <b>This must not be used without explicit instructions from Mesa-Imaging</b>
SR_API(int) SR_SetReg(SRCAM srCam, unsigned char reg, unsigned char val);

//!\internal
//!Returns a desired register value of the camera.
//!\sa SR_SetReg()
SR_API(unsigned char)  SR_GetReg(SRCAM srCam, unsigned char reg);

//!\internal
//!Setup the image processing block.
//!\sa IPBArg
SR_API(int) SR_SetImgProcBlk(SRCAM srCam, enum IPBArg, ...);

//!\internal
//!Same as SR_SetImgProcBlk with variable arguments pointer.
//!\sa SR_SetImgProcBlk()
SR_API(int) SR_SetImgProcBlkVA(SRCAM srCam, enum IPBArg, void*);

//!\internal
//!Sends a TCP Package to an ETH cam
SR_API(int) SR_TCPSend(SRCAM srCam, const void *buf, int len, int flags);

//!\internal
//!Receive a TCP Package from an ETH cam
SR_API(int) SR_TCPRecv(SRCAM srCam, void *buf, int len, int flags);

//!@}

#ifdef __cplusplus
}
#endif


