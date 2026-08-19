/*****************************************************************************
 * wmm.cpp — lightweight World Magnetic Model (WMM2025), declination only
 *
 * Port of the NOAA/NCEI reference algorithm (Geomagnetism Library) trimmed
 * to what a compass needs: magnetic declination at a geodetic position and
 * date. Spherical-harmonic synthesis to degree/order 12 with the official
 * WMM2025 coefficients (see wmm_coeffs.inc, generated from WMM.COF).
 *
 * Accuracy: matches the official WMM2025 test values to < 0.01 deg; the
 * model itself is specified to ~0.5 deg (1-sigma) globally.
 *****************************************************************************/
#include "wmm.h"
#include <math.h>

#include "wmm_coeffs.inc"

/* Schmidt quasi-normalization factors, computed once. */
static double schmidtNorm[WMM_TERMS];
static bool normReady = false;

static void buildSchmidtNorm(void) {
  schmidtNorm[0] = 1.0;
  for (int n = 1; n <= WMM_NMAX; n++) {
    int base = n * (n + 1) / 2;
    int prev = (n - 1) * n / 2;
    schmidtNorm[base] = schmidtNorm[prev] * (double)(2 * n - 1) / n;
    for (int m = 1; m <= n; m++) {
      schmidtNorm[base + m] = schmidtNorm[base + m - 1]
          * sqrt((double)((n - m + 1) * (m == 1 ? 2 : 1)) / (n + m));
    }
  }
  normReady = true;
}

float wmmDeclination(float latDeg, float lonDeg, float altKm, float decYear) {
  if (!normReady) buildSchmidtNorm();

  const double DEG = M_PI / 180.0;
  const double a  = 6378.137;            /* WGS84 semi-major axis, km   */
  const double f  = 1.0 / 298.257223563; /* WGS84 flattening            */
  const double e2 = f * (2.0 - f);
  const double re = 6371.2;              /* geomagnetic reference radius */

  double lat = latDeg * DEG, lon = lonDeg * DEG;
  double slat = sin(lat), clat = cos(lat);

  /* Geodetic -> geocentric spherical */
  double Rc = a / sqrt(1.0 - e2 * slat * slat);
  double p  = (Rc + altKm) * clat;
  double zc = (Rc * (1.0 - e2) + altKm) * slat;
  double r  = sqrt(p * p + zc * zc);
  double sphi = zc / r;                  /* sin/cos of geocentric latitude */
  double cphi = sqrt(1.0 - sphi * sphi);

  /* Time-adjusted coefficients are applied inline: c = c0 + dt * cdot */
  double dt = (double)decYear - WMM_EPOCH;

  /* Associated Legendre P(n,m)(sphi) and dP/dphi, Schmidt semi-normalized
   * (NOAA MAG_PcupLow recursion) */
  double P[WMM_TERMS], dP[WMM_TERMS];
  P[0] = 1.0;
  dP[0] = 0.0;
  for (int n = 1; n <= WMM_NMAX; n++) {
    for (int m = 0; m <= n; m++) {
      int k = n * (n + 1) / 2 + m;
      if (n == m) {
        int k1 = (n - 1) * n / 2 + m - 1;
        P[k]  = cphi * P[k1];
        dP[k] = cphi * dP[k1] + sphi * P[k1];
      } else if (n == 1 && m == 0) {
        int k1 = (n - 1) * n / 2 + m;
        P[k]  = sphi * P[k1];
        dP[k] = sphi * dP[k1] - cphi * P[k1];
      } else {
        int k1 = (n - 2) * (n - 1) / 2 + m;   /* (n-2, m) */
        int k2 = (n - 1) * n / 2 + m;         /* (n-1, m) */
        if (m > n - 2) {
          P[k]  = sphi * P[k2];
          dP[k] = sphi * dP[k2] - cphi * P[k2];
        } else {
          double c = (double)((n - 1) * (n - 1) - m * m)
                   / ((double)(2 * n - 1) * (2 * n - 3));
          P[k]  = sphi * P[k2] - c * P[k1];
          dP[k] = sphi * dP[k2] - cphi * P[k2] - c * dP[k1];
        }
      }
    }
  }

  /* Spherical-harmonic summation: field in geocentric frame.
   * Bx' = north, By' = east, Bz' = down (only Bx, By needed for D,
   * but Bz is cheap and needed for the geodetic rotation of X). */
  double sm[WMM_NMAX + 1], cm[WMM_NMAX + 1];
  for (int m = 0; m <= WMM_NMAX; m++) {
    sm[m] = sin(m * lon);
    cm[m] = cos(m * lon);
  }
  double ar = re / r;
  double arn = ar * ar;                       /* (re/r)^(n+2), start n=1 */
  double Bx = 0, By = 0, Bz = 0;
  for (int n = 1; n <= WMM_NMAX; n++) {
    arn *= ar;
    for (int m = 0; m <= n; m++) {
      int k = n * (n + 1) / 2 + m;
      double gnm = (WMM_G0[k] + dt * WMM_GD[k]) * schmidtNorm[k];
      double hnm = (WMM_H0[k] + dt * WMM_HD[k]) * schmidtNorm[k];
      double gc = gnm * cm[m] + hnm * sm[m];
      Bx += arn * gc * dP[k];
      Bz -= arn * (n + 1) * gc * P[k];
      if (m > 0 && cphi > 1e-10)
        By += arn * m * (gnm * sm[m] - hnm * cm[m]) * P[k] / cphi;
    }
  }

  /* Rotate X,Z from geocentric to geodetic latitude frame */
  double psi = atan2(sphi, cphi) - lat;       /* phi' - phi */
  double Xg = Bx * cos(psi) - Bz * sin(psi);
  double Yg = By;

  return (float)(atan2(Yg, Xg) / DEG);
}
