/*****************************************************************************
 * wmm.h — lightweight World Magnetic Model (WMM2025), declination only
 *
 * Magnetic declination (deg, east-positive) at a geodetic position and date.
 * Add the result to a magnetic heading to get true heading.
 *
 *   latDeg  geodetic latitude, deg  [-90, 90]
 *   lonDeg  geodetic longitude, deg [-180, 180]
 *   altKm   altitude above the WGS84 ellipsoid, km (0 is fine at ground)
 *   decYear decimal year, e.g. 2026.63 (model valid 2025.0 - 2030.0)
 *****************************************************************************/
#ifndef _WMM_H_
#define _WMM_H_

float wmmDeclination(float latDeg, float lonDeg, float altKm, float decYear);

#endif
