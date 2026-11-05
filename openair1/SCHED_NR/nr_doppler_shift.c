/*
 * SPDX-License-Identifier: LicenseRef-CSSL-1.0
 */

/*
 * Satellite Doppler shift and propagation delay emulation (see nr_doppler_shift.h).
 */

#include "nr_doppler_shift.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <stdint.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>

// ---------------------------------------------------------------------------
// Physical constants
// ---------------------------------------------------------------------------
#define DS_H        600000.0            // satellite altitude (m)
#define DS_THETA_MIN 30.0               // minimum elevation angle (deg)
#define DS_K        3.986005e14         // Kepler constant (m^3/s^2)
#define DS_RE       6371000.0           // Earth radius (m)
#define DS_R        (DS_RE + DS_H)      // orbit radius (m)
#define DS_C        299792458.0         // speed of light (m/s)

// Angular velocity of the satellite (rad/s)
static double omega_s(void)
{
  static double val = 0.0;
  if (val == 0.0)
    val = sqrt(DS_K / (DS_R * DS_R * DS_R));
  return val;
}

NR_DopplerContext g_doppler_ctx = {0};

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

// Wall-clock in seconds with nanosecond resolution
static double now_seconds(void)
{
  struct timespec ts;
  clock_gettime(CLOCK_REALTIME, &ts);
  return (double)ts.tv_sec + (double)ts.tv_nsec * 1e-9;
}

// Monotonic clock in seconds, for round-trip measurements
static double mono_seconds(void)
{
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (double)ts.tv_sec + (double)ts.tv_nsec * 1e-9;
}

static void set_times(double experiment_time, double wall_time)
{
  pthread_mutex_lock(&g_doppler_ctx.times_mutex);
  g_doppler_ctx.times.experiment_time = experiment_time;
  g_doppler_ctx.times.start_time      = wall_time;
  pthread_mutex_unlock(&g_doppler_ctx.times_mutex);
}

// Current experiment time: the start of the pass until the first transmitted slot,
// then wrapped over the pass like the server clock
static double current_exp_time(void)
{
  if (!__atomic_load_n(&g_doppler_ctx.tx_started, __ATOMIC_ACQUIRE))
    return g_doppler_ctx.pass_start;
  double exp_t, start_t;
  pthread_mutex_lock(&g_doppler_ctx.times_mutex);
  exp_t   = g_doppler_ctx.times.experiment_time;
  start_t = g_doppler_ctx.times.start_time;
  pthread_mutex_unlock(&g_doppler_ctx.times_mutex);
  double t = now_seconds() - start_t + exp_t;
  if (g_doppler_ctx.vis_time > 0.0) {
    t = g_doppler_ctx.pass_start + fmod(t - g_doppler_ctx.pass_start, g_doppler_ctx.vis_time);
    if (t < g_doppler_ctx.pass_start)
      t += g_doppler_ctx.vis_time;
  }
  return t;
}

// One-way propagation delay (s) at experiment time t: slant range between
// ground station and satellite divided by c (same range as in build_table()).
static double ds_delay_at(double t)
{
  const double Os = omega_s();
  const double a1 = g_doppler_ctx.a1;
  return sqrt(DS_RE * DS_RE + DS_R * DS_R
              - 2.0 * DS_RE * DS_R * cos(Os * t) * a1) / DS_C;
}

// Visibility time for a given maximum elevation angle
static double ds_t_v(double theta_max_deg)
{
  double Os = omega_s();
  double p = cos(acos(DS_RE / DS_R * cos(DS_THETA_MIN * M_PI / 180.0))
                 - DS_THETA_MIN * M_PI / 180.0);
  double q = cos(acos(DS_RE / DS_R * cos(theta_max_deg * M_PI / 180.0))
                 - theta_max_deg * M_PI / 180.0);
  return (2.0 / Os) * acos(p / q);
}

// ---------------------------------------------------------------------------
// Config parser
// ---------------------------------------------------------------------------
static int parse_config(const char *path,
                        int    *max_elev_deg,
                        double *carrier_hz,
                        char   *server_ip,
                        size_t  server_ip_len,
                        int    *delay_mode,
                        int    *sib19_period,
                        int    *model,
                        int    *qef_window,
                        char   *qef_table,
                        size_t  qef_table_len)
{
  FILE *f = fopen(path, "r");
  if (!f) {
    fprintf(stderr, "[DOPPLER] Cannot open config: %s\n", path);
    return -1;
  }

  // Defaults
  *max_elev_deg = 70;
  *carrier_hz   = 3319680000.0;
  *delay_mode   = NR_DOPPLER_DELAY_OFF;
  *sib19_period = 100;
  *model        = NR_DOPPLER_MODEL_EARTH_MOVING;
  *qef_window   = 1;
  strncpy(server_ip, "127.0.0.1", server_ip_len - 1);
  strncpy(qef_table, "qef_table.csv", qef_table_len - 1);

  char line[512];
  while (fgets(line, sizeof(line), f)) {
    char *eq = NULL;
    if (line[0] == '#')
      continue; // comment lines may mention key names
    if ((eq = strstr(line, "Maximum elevation = ")))
      sscanf(eq + 20, "%d", max_elev_deg);
    else if ((eq = strstr(line, "Carrier frequency = ")))
      sscanf(eq + 20, "%lf", carrier_hz);
    else if ((eq = strstr(line, "REMOTE_IP = ")))
      sscanf(eq + 12, "%63s", server_ip);
    else if ((eq = strstr(line, "Delay mode = ")))
      sscanf(eq + 13, "%d", delay_mode);
    else if ((eq = strstr(line, "SIB19 period = ")))
      sscanf(eq + 15, "%d", sib19_period);
    else if ((eq = strstr(line, "Doppler model = ")))
      sscanf(eq + 16, "%d", model);
    else if ((eq = strstr(line, "QEF window = ")))
      sscanf(eq + 13, "%d", qef_window);
    else if ((eq = strstr(line, "QEF table = ")))
      sscanf(eq + 12, "%255s", qef_table);
  }

  fclose(f);
  if (*delay_mode < NR_DOPPLER_DELAY_OFF || *delay_mode > NR_DOPPLER_DELAY_ABSOLUTE) {
    fprintf(stderr, "[DOPPLER] invalid Delay mode %d, disabling delay\n", *delay_mode);
    *delay_mode = NR_DOPPLER_DELAY_OFF;
  }
  if (*sib19_period < 0) {
    fprintf(stderr, "[DOPPLER] invalid SIB19 period %d, disabling SIB19 refresh\n", *sib19_period);
    *sib19_period = 0;
  }
  printf("[DOPPLER] SIB19 ephemeris refresh every %d frames%s\n",
         *sib19_period, *sib19_period ? "" : " (off)");
  if (*model != NR_DOPPLER_MODEL_EARTH_MOVING && *model != NR_DOPPLER_MODEL_QUASI_EARTH_FIXED) {
    fprintf(stderr, "[DOPPLER] invalid Doppler model %d, using earth-moving\n", *model);
    *model = NR_DOPPLER_MODEL_EARTH_MOVING;
  }
  if (*model == NR_DOPPLER_MODEL_QUASI_EARTH_FIXED)
    printf("[DOPPLER] model: quasi-earth-fixed, table %s, %s window\n",
           qef_table, *qef_window ? "elevation-masked" : "full-pass");
  else
    printf("[DOPPLER] model: earth-moving (analytic)\n");
  static const char *delay_mode_str[] = {"off", "relative", "absolute"};
  printf("[DOPPLER] config: max_elev=%d deg  carrier=%.7e Hz  server=%s  delay=%s\n",
         *max_elev_deg, *carrier_hz, server_ip, delay_mode_str[*delay_mode]);
  return 0;
}

// ---------------------------------------------------------------------------
// TCP clock client
// ---------------------------------------------------------------------------
// Connect to the clock server and identify as "oai_gnb". Returns the socket, or -1.
static int clock_connect(const char *server_ip, int server_port)
{
  int sockfd = socket(AF_INET, SOCK_STREAM, 0);
  if (sockfd < 0) {
    perror("[DOPPLER] socket");
    return -1;
  }

  struct sockaddr_in addr = {0};
  addr.sin_family = AF_INET;
  addr.sin_port   = htons((uint16_t)server_port);
  if (inet_pton(AF_INET, server_ip, &addr.sin_addr) <= 0) {
    fprintf(stderr, "[DOPPLER] Invalid server IP: %s\n", server_ip);
    close(sockfd);
    return -1;
  }

  if (connect(sockfd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
    perror("[DOPPLER] connect");
    close(sockfd);
    return -1;
  }

  // Handshake: receive server prompt, send identifier, receive confirmation
  char buf[1024] = {0};
  recv(sockfd, buf, sizeof(buf) - 1, 0);
  printf("[DOPPLER] Server: %s\n", buf);
  const char *id = "oai_gnb";
  send(sockfd, id, strlen(id), 0);
  memset(buf, 0, sizeof(buf));
  recv(sockfd, buf, sizeof(buf) - 1, 0);
  printf("[DOPPLER] Server: %s\n", buf);
  return sockfd;
}

// Server pass time corrected for the network latency: N probes, keep the one with
// the smallest round trip and add half of it. Returns NAN on failure.
static double clock_query(int sockfd)
{
  const int n_probes = 5;
  int have_best = 0;
  double best_server = 0.0, best_half_rtt = 0.0, best_recv = 0.0;

  for (int p = 0; p < n_probes; p++) {
    const char *query = "time?";
    const double t0 = mono_seconds();
    if (send(sockfd, query, strlen(query), 0) < 0)
      break;
    // 8-byte big-endian double
    uint8_t raw[8];
    int got = 0;
    while (got < 8) {
      int n = recv(sockfd, raw + got, 8 - got, 0);
      if (n <= 0) {
        got = -1;
        break;
      }
      got += n;
    }
    const double t1 = mono_seconds();
    if (got != 8)
      break;
    uint8_t host[8];
    for (int k = 0; k < 8; k++)
      host[k] = raw[7 - k];
    double server_t;
    memcpy(&server_t, host, sizeof(server_t));

    const double rtt = t1 - t0;
    if (!have_best || rtt < 2.0 * best_half_rtt) {
      have_best     = 1;
      best_server   = server_t;
      best_half_rtt = rtt / 2.0;
      best_recv     = t1;
    }
  }

  if (!have_best)
    return NAN;
  return best_server + best_half_rtt + (mono_seconds() - best_recv);
}

// Waits for the first transmitted slot, asks the server to start the pass clock
// ("started"), then re-synchronizes the local pass clock every second. The server
// starts its clock only once, so a restarted gNB takes the current pass time and
// rejoins the pass in phase.
static void *server_clock_thread(void *arg)
{
  (void)arg;
  const int sockfd = g_doppler_ctx.sockfd;

  int waited_ms = 0;
  while (!__atomic_load_n(&g_doppler_ctx.tx_started, __ATOMIC_ACQUIRE)) {
    if (waited_ms >= 120000) {
      fprintf(stderr, "[DOPPLER] No transmitted slot within 120 s: pass clock not started\n");
      return NULL;
    }
    usleep(20000);
    waited_ms += 20;
  }

  const char *msg = "started";
  send(sockfd, msg, strlen(msg), 0);
  char ack[64] = {0};
  recv(sockfd, ack, sizeof(ack) - 1, 0);

  int first = 1;
  for (;;) {
    const double server_t = clock_query(sockfd);
    if (!isnan(server_t)) {
      set_times(server_t, now_seconds());
      if (first) {
        printf("[DOPPLER] Pass clock started: experiment_time=%.6f s\n", server_t);
        first = 0;
      }
    }
    sleep(1);
  }
  return NULL;
}

// ---------------------------------------------------------------------------
// Earth-moving Doppler table
// ---------------------------------------------------------------------------
static int build_table(double theta_max_deg, double carrier_hz,
                       double t_start, double t_end, double dt)
{
  if (t_end <= t_start) {
    fprintf(stderr, "[DOPPLER] t_end (%.2f) <= t_start (%.2f)\n", t_end, t_start);
    return -1;
  }
  size_t n = (size_t)((t_end - t_start) / dt) + 1;
  double *tbl = malloc(n * sizeof(double));
  if (!tbl) {
    perror("[DOPPLER] malloc");
    return -1;
  }

  double a1 = cos(acos(DS_RE / DS_R * cos(theta_max_deg * M_PI / 180.0))
                  - theta_max_deg * M_PI / 180.0);
  double Os = omega_s();

  for (size_t i = 0; i < n; i++) {
    double t    = t_start + (double)i * dt;
    double num  = DS_RE * DS_R * sin(Os * t) * a1 * Os;
    double denom = sqrt(DS_RE * DS_RE + DS_R * DS_R
                        - 2.0 * DS_RE * DS_R * cos(Os * t) * a1);
    tbl[i] = -(carrier_hz / DS_C) * num / denom;
  }

  g_doppler_ctx.table      = tbl;
  g_doppler_ctx.table_size = n;
  g_doppler_ctx.t_start    = t_start;
  g_doppler_ctx.dt         = dt;
  g_doppler_ctx.a1         = a1;
  g_doppler_ctx.delay_ref_s = ds_delay_at(0.0); // delay at maximum elevation
  // d(t) is symmetric about t = 0, so the largest delay is at one end of the window
  g_doppler_ctx.delay_max_s = fmax(ds_delay_at(t_start), ds_delay_at(t_end));

  printf("[DOPPLER] Table built: %zu entries  t=[%.2f, %.2f] s  dt=%.3f s\n",
         n, t_start, t_end, dt);
  printf("[DOPPLER] Delay curve: %.1f us at max elevation, %.1f us at t_start, "
         "%.1f us of variation\n",
         g_doppler_ctx.delay_ref_s * 1e6,
         ds_delay_at(t_start) * 1e6,
         (ds_delay_at(t_start) - g_doppler_ctx.delay_ref_s) * 1e6);
  return 0;
}

// ---------------------------------------------------------------------------
// Quasi-earth-fixed table
// ---------------------------------------------------------------------------
// CSV, '#' comment header, rows evenly spaced in time:
//   t_s, rr_center, rr_device, range_device, sat_x, sat_y, sat_z,
//   sat_vx, sat_vy, sat_vz, masked
// t_s is the experiment time (s), rr_* are range rates (m/s), range_device is
// the slant range to the device (m), sat_* the satellite ECEF state (m, m/s),
// masked is 1 when both beam center and device are above the elevation mask.
// The table holds geometry only: the Doppler is formed with the configured carrier.
static int load_qef_table(const char *config_path)
{
  // A relative "QEF table" path is resolved from the directory of config.txt
  char path[512];
  if (g_doppler_ctx.qef_table[0] == '/') {
    snprintf(path, sizeof(path), "%s", g_doppler_ctx.qef_table);
  } else {
    const char *slash = strrchr(config_path, '/');
    const int dirlen = slash ? (int)(slash - config_path) : 0;
    snprintf(path, sizeof(path), "%.*s%s%s", dirlen, config_path, dirlen ? "/" : "",
             g_doppler_ctx.qef_table);
  }
  FILE *f = fopen(path, "r");
  if (!f) {
    fprintf(stderr, "[DOPPLER] Cannot open QEF table: %s\n", path);
    return -1;
  }

  size_t cap = 8192, n = 0;
  double *t = malloc(cap * sizeof(*t));
  int *masked = malloc(cap * sizeof(*masked));
  NR_QefRow *rows = malloc(cap * sizeof(*rows));
  char line[512];
  while (t && masked && rows && fgets(line, sizeof(line), f)) {
    if (line[0] == '#' || line[0] == '\n')
      continue;
    if (n == cap) {
      cap *= 2;
      t = realloc(t, cap * sizeof(*t));
      masked = realloc(masked, cap * sizeof(*masked));
      rows = realloc(rows, cap * sizeof(*rows));
      if (!t || !masked || !rows)
        break;
    }
    NR_QefRow *r = &rows[n];
    if (sscanf(line, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%d",
               &t[n], &r->rr_center, &r->rr_device, &r->range_device,
               &r->pos[0], &r->pos[1], &r->pos[2], &r->vel[0], &r->vel[1], &r->vel[2],
               &masked[n]) == 11)
      n++;
  }
  fclose(f);
  if (!t || !masked || !rows || n < 2) {
    fprintf(stderr, "[DOPPLER] QEF table %s: unreadable or fewer than 2 rows\n", path);
    free(t); free(masked); free(rows);
    return -1;
  }

  // Window: the whole pass, or the rows where center and device clear the mask.
  size_t first = 0, last = n - 1;
  if (g_doppler_ctx.qef_window) {
    while (first < n && !masked[first]) first++;
    while (last > first && !masked[last]) last--;
    if (first >= n || !masked[last]) {
      fprintf(stderr, "[DOPPLER] QEF table %s: no masked rows\n", path);
      free(t); free(masked); free(rows);
      return -1;
    }
  }
  const size_t m = last - first + 1;
  const double dt = t[first + 1] - t[first];
  for (size_t i = first; i <= last; i++) {
    if (fabs(t[i] - t[first] - (double)(i - first) * dt) > 1e-3) {
      fprintf(stderr, "[DOPPLER] QEF table %s: rows are not evenly spaced (row %zu)\n", path, i);
      free(t); free(masked); free(rows);
      return -1;
    }
  }

  double *tbl = malloc(m * sizeof(*tbl));
  NR_QefRow *qef = malloc(m * sizeof(*qef));
  if (!tbl || !qef) {
    perror("[DOPPLER] malloc");
    free(t); free(masked); free(rows); free(tbl); free(qef);
    return -1;
  }
  double dmin = 1e30, dmax = 0.0, dop_lo = 1e30, dop_hi = -1e30;
  for (size_t i = 0; i < m; i++) {
    qef[i] = rows[first + i];
    // Beam-center Doppler is pre-compensated: the device sees the residual
    tbl[i] = -(g_doppler_ctx.carrier_freq_hz / DS_C) * (qef[i].rr_device - qef[i].rr_center);
    const double d = qef[i].range_device / DS_C;
    dmin = fmin(dmin, d); dmax = fmax(dmax, d);
    dop_lo = fmin(dop_lo, tbl[i]); dop_hi = fmax(dop_hi, tbl[i]);
  }

  g_doppler_ctx.table       = tbl;
  g_doppler_ctx.table_size  = m;
  g_doppler_ctx.t_start     = t[first];
  g_doppler_ctx.dt          = dt;
  g_doppler_ctx.qef         = qef;
  g_doppler_ctx.delay_ref_s = dmin;
  g_doppler_ctx.delay_max_s = dmax;

  printf("[DOPPLER] QEF table %s: %zu of %zu rows, t=[%.2f, %.2f] s, dt=%.3f s\n",
         path, m, n, t[first], t[last], dt);
  printf("[DOPPLER] QEF residual Doppler %.1f .. %.1f Hz at %.7e Hz, device delay %.1f .. %.1f us\n",
         dop_lo, dop_hi, g_doppler_ctx.carrier_freq_hz, dmin * 1e6, dmax * 1e6);
  free(t); free(masked); free(rows);
  return 0;
}

// Position of experiment time t in the table: row index i and the fraction
// towards row i+1. Outside the table the first / last row is held.
static void table_locate(double t, size_t *i, double *frac)
{
  const size_t n = g_doppler_ctx.table_size;
  const double x = (t - g_doppler_ctx.t_start) / g_doppler_ctx.dt;
  if (x <= 0.0 || n < 2) {
    *i = 0;
    *frac = 0.0;
  } else if (x >= (double)(n - 1)) {
    *i = n - 1;
    *frac = 0.0;
  } else {
    *i = (size_t)x;
    *frac = x - (double)*i;
  }
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

int nr_doppler_init(const char *config_path, int server_port)
{
  pthread_mutex_init(&g_doppler_ctx.times_mutex, NULL);

  if (parse_config(config_path,
                   &g_doppler_ctx.max_elevation_deg,
                   &g_doppler_ctx.carrier_freq_hz,
                   g_doppler_ctx.server_ip,
                   sizeof(g_doppler_ctx.server_ip),
                   &g_doppler_ctx.delay_mode,
                   &g_doppler_ctx.sib19_period_frames,
                   &g_doppler_ctx.model,
                   &g_doppler_ctx.qef_window,
                   g_doppler_ctx.qef_table,
                   sizeof(g_doppler_ctx.qef_table)) != 0)
    return -1;

  // The pass, as served by the clock server: [-vis/2, vis/2], looped
  g_doppler_ctx.vis_time   = ds_t_v(g_doppler_ctx.max_elevation_deg);
  g_doppler_ctx.pass_start = -g_doppler_ctx.vis_time / 2.0;
  set_times(g_doppler_ctx.pass_start, now_seconds());

  g_doppler_ctx.sockfd = clock_connect(g_doppler_ctx.server_ip, server_port);
  g_doppler_ctx.server_connected = g_doppler_ctx.sockfd >= 0;
  if (!g_doppler_ctx.server_connected) {
    fprintf(stderr, "[DOPPLER] Clock server not connected, Doppler emulation disabled\n");
    return -1;
  }

  int ret;
  if (g_doppler_ctx.model == NR_DOPPLER_MODEL_QUASI_EARTH_FIXED)
    ret = load_qef_table(config_path);
  else // earth-moving: table over the whole pass
    ret = build_table(g_doppler_ctx.max_elevation_deg,
                      g_doppler_ctx.carrier_freq_hz,
                      g_doppler_ctx.pass_start, -g_doppler_ctx.pass_start, 0.1);
  if (ret != 0)
    return ret;

  pthread_t clock_tid;
  pthread_create(&clock_tid, NULL, server_clock_thread, NULL);
  pthread_detach(clock_tid);
  return 0;
}

void nr_doppler_mark_tx(void)
{
  if (__atomic_load_n(&g_doppler_ctx.tx_started, __ATOMIC_ACQUIRE) || !g_doppler_ctx.table)
    return;
  // First transmitted slot: start the local pass clock at the beginning of the pass;
  // the clock thread then synchronizes it with the server
  set_times(g_doppler_ctx.pass_start, now_seconds());
  __atomic_store_n(&g_doppler_ctx.tx_started, 1, __ATOMIC_RELEASE);
}

double nr_doppler_get_current_hz(void)
{
  if (!g_doppler_ctx.table || !__atomic_load_n(&g_doppler_ctx.tx_started, __ATOMIC_ACQUIRE))
    return 0.0;

  size_t i;
  double frac;
  table_locate(current_exp_time(), &i, &frac);

  // Quasi-earth-fixed: interpolate between rows.
  if (g_doppler_ctx.model == NR_DOPPLER_MODEL_QUASI_EARTH_FIXED && frac > 0.0)
    return g_doppler_ctx.table[i] + frac * (g_doppler_ctx.table[i + 1] - g_doppler_ctx.table[i]);

  // Earth-moving: nearest row below
  return g_doppler_ctx.table[i];
}

int nr_doppler_get_sat_state(double pos[3], double vel[3])
{
  if (!g_doppler_ctx.table)
    return -1;

  const double t  = current_exp_time();

  if (g_doppler_ctx.model == NR_DOPPLER_MODEL_QUASI_EARTH_FIXED) {
    size_t i;
    double frac;
    table_locate(t, &i, &frac);
    const NR_QefRow *a = &g_doppler_ctx.qef[i];
    const NR_QefRow *b = (frac > 0.0) ? &g_doppler_ctx.qef[i + 1] : a;
    for (int k = 0; k < 3; k++) {
      pos[k] = a->pos[k] + a->vel[k] * frac * g_doppler_ctx.dt;
      vel[k] = a->vel[k] + frac * (b->vel[k] - a->vel[k]);
    }
    return 0;
  }

  const double Os = omega_s();

  const double ca = g_doppler_ctx.a1;                 // cos(alpha_min)
  const double sa = sqrt(1.0 - ca * ca);              // sin(alpha_min)
  const double c  = cos(Os * t);
  const double s  = sin(Os * t);

  pos[0] = DS_R * c * sa;
  pos[1] = DS_R * s;
  pos[2] = DS_R * c * ca;

  vel[0] = -Os * DS_R * s * sa;
  vel[1] =  Os * DS_R * c;
  vel[2] = -Os * DS_R * s * ca;

  return 0;
}

double nr_doppler_get_delay_s(void)
{
  if (!g_doppler_ctx.table || g_doppler_ctx.delay_mode == NR_DOPPLER_DELAY_OFF)
    return 0.0;

  double d;
  if (g_doppler_ctx.model == NR_DOPPLER_MODEL_QUASI_EARTH_FIXED) {
    size_t i;
    double frac;
    table_locate(current_exp_time(), &i, &frac);
    double r = g_doppler_ctx.qef[i].range_device;
    if (frac > 0.0)
      r += frac * (g_doppler_ctx.qef[i + 1].range_device - r);
    d = r / DS_C;
  } else {
    d = ds_delay_at(current_exp_time());
  }

  if (g_doppler_ctx.delay_mode == NR_DOPPLER_DELAY_RELATIVE)
    d -= g_doppler_ctx.delay_ref_s;

  // One way, applied on both legs: the round trip is 2*d, as a UE computes it
  // from the SIB19 ephemeris
  return d > 0.0 ? d : 0.0;
}

double nr_doppler_get_max_delay_s(void)
{
  if (!g_doppler_ctx.table || g_doppler_ctx.delay_mode == NR_DOPPLER_DELAY_OFF)
    return 0.0;
  const double d_max = g_doppler_ctx.delay_max_s;
  return g_doppler_ctx.delay_mode == NR_DOPPLER_DELAY_RELATIVE
             ? d_max - g_doppler_ctx.delay_ref_s
             : d_max;
}

int nr_doppler_get_sib19_period(void)
{
  return g_doppler_ctx.sib19_period_frames;
}

void nr_doppler_cleanup(void)
{
  free(g_doppler_ctx.table);
  free(g_doppler_ctx.qef);
  g_doppler_ctx.qef        = NULL;
  g_doppler_ctx.table      = NULL;
  g_doppler_ctx.table_size = 0;
  pthread_mutex_destroy(&g_doppler_ctx.times_mutex);
}
