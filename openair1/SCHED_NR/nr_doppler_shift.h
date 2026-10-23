/*
 * SPDX-License-Identifier: LicenseRef-CSSL-1.0
 */

/*
 * Satellite Doppler shift and propagation delay emulation for the gNB RU.
 *
 * The experiment clock is synchronized with a TCP time server, then a Doppler /
 * delay / ephemeris source is built for one of two satellite models
 * ("Doppler model" in config.txt):
 *
 *   EARTH_MOVING       analytic circular orbit over a non-rotating spherical
 *                      Earth; the UE sees the full Doppler at its position.
 *   QUASI_EARTH_FIXED  pass over a fixed beam center, played back from a
 *                      precomputed table (see load_qef_table()). The satellite
 *                      pre-compensates the beam-center Doppler, so the UE only
 *                      sees the residual  -f_c/c * (rr_device - rr_center).
 */

#ifndef NR_DOPPLER_SHIFT_H
#define NR_DOPPLER_SHIFT_H

#include <stddef.h>
#include <pthread.h>

typedef struct {
  double experiment_time; // experiment time received from the server (s)
  double start_time;      // local wall-clock when the server was queried (s)
} NR_ExperimentTimes;

/*
 * Propagation delay inserted in the RU, "Delay mode = N" in config.txt.
 * The delay is d(t) = slant_range(t) / c, from the same geometry as the Doppler.
 *
 * ABSOLUTE inserts d(t). RELATIVE inserts d(t) - min(d) over the window, i.e.
 * only the time-varying part the UE timing advance loop has to track.
 * The value is applied one way on each leg (tx_rf() downlink, rx_rf() uplink),
 * so the round trip is 2*d.
 *
 * In TDD the inserted delay must stay below the guard period, otherwise the
 * delayed DL burst overlaps the gNB UL reception window.
 */
typedef enum {
  NR_DOPPLER_DELAY_OFF      = 0,
  NR_DOPPLER_DELAY_RELATIVE = 1, // d(t) - min d, zero at closest approach
  NR_DOPPLER_DELAY_ABSOLUTE = 2, // d(t)
} NR_DopplerDelayMode;

typedef enum {
  NR_DOPPLER_MODEL_EARTH_MOVING      = 0,
  NR_DOPPLER_MODEL_QUASI_EARTH_FIXED = 1,
} NR_DopplerModel;

/// One row of the quasi-earth-fixed table (geometry only, no carrier).
typedef struct {
  double rr_center;    // range rate satellite -> beam center (m/s)
  double rr_device;    // range rate satellite -> device (m/s)
  double range_device; // slant range satellite -> device (m)
  double pos[3];       // satellite position, ECEF/ITRS (m)
  double vel[3];       // satellite velocity, ECEF/ITRS (m/s)
} NR_QefRow;

typedef struct {
  double            *table;           // precomputed Doppler values (Hz)
  size_t             table_size;      // number of entries
  double             t_start;         // experiment time of table[0]
  double             dt;              // time step between entries (s)

  NR_ExperimentTimes times;
  pthread_mutex_t    times_mutex;

  int    max_elevation_deg;           // maximum elevation of the pass (deg)
  double carrier_freq_hz;             // carrier frequency (Hz)
  char   server_ip[64];               // TCP clock server address

  double a1;                          // cos of the minimum central angle between
                                      // ground station and satellite ground track

  int    delay_mode;                  // NR_DopplerDelayMode
  double delay_ref_s;                 // smallest one-way delay in the window
  double delay_max_s;                 // largest one-way delay in the window
  int    sib19_period_frames;         // SIB19 ephemeris refresh period, 0 = off

  int        model;                   // NR_DopplerModel
  int        qef_window;              // 0 = full pass, 1 = elevation-masked rows only
  char       qef_table[256];          // table path, relative to the config file directory
  NR_QefRow *qef;                     // rows of the selected window, qef[i] at t_start + i*dt

  int    server_connected;            // 1 after successful TCP sync
} NR_DopplerContext;

extern NR_DopplerContext g_doppler_ctx;

/*
 * Parse config_path, synchronize with the TCP clock server on server_port and
 * build the Doppler table. Blocks until the clock sync is done (about 5 s).
 * Returns 0 on success.
 */
int nr_doppler_init(const char *config_path, int server_port);

/*
 * Doppler frequency offset (Hz) at the current experiment time.
 * Thread-safe; returns 0.0 if the table is not ready.
 */
double nr_doppler_get_current_hz(void);

/*
 * Satellite position (m) and velocity (m/s) at the current experiment time,
 * for the SIB19 ephemeris (positionVelocity-r17).
 *
 * EARTH_MOVING: frame of the analytic model, ground station at (0, 0, R_E).
 * The range |pos - (0, 0, R_E)| is the one used to build the Doppler table, so
 * a UE computing its own Doppler from this ephemeris matches the shift applied
 * in tx_rf().
 * QUASI_EARTH_FIXED: ECEF/ITRS state from the table.
 *
 * Thread-safe. Returns 0 on success, -1 if the table is not ready.
 */
int nr_doppler_get_sat_state(double pos[3], double vel[3]);

/*
 * One-way propagation delay (s) at the current experiment time, according to
 * the configured delay mode. Returns 0.0 when the mode is OFF or the table is
 * not ready. Thread-safe.
 */
double nr_doppler_get_delay_s(void);

/*
 * Largest value nr_doppler_get_delay_s() can return over the window, used to
 * size the uplink delay line. Returns 0.0 when the mode is OFF.
 */
double nr_doppler_get_max_delay_s(void);

/*
 * SIB19 ephemeris refresh period in frames ("SIB19 period = N" in config.txt,
 * default 100). 0 keeps the static ephemeris from the gNB configuration.
 */
int nr_doppler_get_sib19_period(void);

void nr_doppler_cleanup(void);

#endif /* NR_DOPPLER_SHIFT_H */
