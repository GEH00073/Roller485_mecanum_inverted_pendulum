//=====================================================================================================
// MadgwickAHRS.h
//=====================================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Copyright (c) Sebastian Madgwick, Arduino LLC
// Modifications Copyright (c) 2026 Atsushi Kataoka
// SPDX-License-Identifier: GPL-3.0-only
//
// Derived from https://github.com/arduino-libraries/MadgwickAHRS
// The upstream library specifies GNU GPL version 3.
//
// Date			Author          Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
//=====================================================================================================
#ifndef MadgwickAHRS_h
#define MadgwickAHRS_h

//----------------------------------------------------------------------------------------------------
// Variable declaration

// extern volatile float q0, q1, q2, q3;	// quaternion of sensor frame relative to auxiliary frame

//---------------------------------------------------------------------------------------------------
// Function declarations

void MadgwickAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az, float *pitch, float *roll, float *yaw);
void MadgwickAHRSetBeta(float beta);
#endif
//=====================================================================================================
// End of file
//=====================================================================================================
