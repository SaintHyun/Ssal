// main.c - RPi 3B+ + DHT22 + pigpio + IR (NEC, POWER TOGGLE only) + CSV logging
// Features:
// - Non-blocking stdin commands: "test", "nec", "car"
// - DHT hardening: 300ms timeout, 60us threshold, 3x retry @150ms
// - Requests SCHED_FIFO realtime (non-fatal if denied)
// - Pure hysteresis only: ON >= 26°C, OFF <= 24°C (no pending-arm)
//
// Build: make
// Run:   sudo ./tempctrl

#include <pigpio.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <signal.h>
#include <time.h>
#include <string.h>
#include <stdbool.h>
#include <unistd.h>
#include <sys/select.h>
#include <ctype.h>
#include <sched.h>
#include <errno.h>

// ================== Pins (BCM) ==================
#define DHT_PIN 4
#define IR_PIN 18 // BCM18 (physical pin 12), HW PWM

// ================== Control (hysteresis) ==================
#define COOL_ON_C 26.0f  // turn fan ON at or above this
#define COOL_OFF_C 24.0f // turn fan OFF at or below this

// Loop period (seconds)
#define LOOP_PERIOD_S 2

// ================== DHT22 timing ==================
#define TIMEOUT_US 300000   // wait-for-level timeout
#define DHT_THRESHOLD_US 60 // bit threshold (~28us=0, ~70us=1)
#define DHT_RETRIES 3
#define DHT_RETRY_DELAY_MS 150

// ================== NEC IR config ==================
#define IR_CARRIER_HZ 38000
#define IR_DUTY_PER_MILL 330000 // ~33% (0..1,000,000)
#define IR_REPEAT_COUNT 1       // single press for toggle safety
#define IR_REPEAT_GAP_US 40000
#define USE_NEC_REPEAT_FRAME 0

// ---- FILL THESE WITH YOUR CAPTURED CODES ----
#define NEC_ADDR 0x00       // <<< your remote's NEC address (8-bit)
#define NEC_CMD_TOGGLE 0x02 // <<< your remote's POWER (toggle) (8-bit)
// ---------------------------------------------

static volatile int keep_running = 1;

// ===== Utilities =====
static void handle_sigint(int sig)
{
  (void)sig;
  keep_running = 0;
}

static void iso_timestamp(char *buf, size_t len)
{
  time_t t = time(NULL);
  struct tm tmv;
  localtime_r(&t, &tmv);
  strftime(buf, len, "%Y-%m-%d %H:%M:%S", &tmv);
}

static void try_set_realtime(int prio)
{ // non-fatal
  struct sched_param sp;
  sp.sched_priority = prio;
  if (sched_setscheduler(0, SCHED_FIFO, &sp) != 0)
  {
    fprintf(stderr, "sched_setscheduler(SCHED_FIFO,%d) failed: %s\n",
            prio, strerror(errno));
  }
  else
  {
    printf("Scheduler set to SCHED_FIFO priority %d\n", prio);
  }
}

// ---------- stdin helpers ----------
static int stdin_readline(char *buf, size_t len)
{
  fd_set rfds;
  FD_ZERO(&rfds);
  FD_SET(STDIN_FILENO, &rfds);
  struct timeval tv = {0, 0}; // non-blocking
  int r = select(STDIN_FILENO + 1, &rfds, NULL, NULL, &tv);
  if (r > 0 && FD_ISSET(STDIN_FILENO, &rfds))
  {
    if (fgets(buf, (int)len, stdin))
    {
      size_t n = strlen(buf);
      while (n && (buf[n - 1] == '\n' || buf[n - 1] == '\r'))
        buf[--n] = '\0';
      return 1;
    }
  }
  return 0;
}
static int is_cmd_eq(const char *s, const char *t)
{
  size_t n = strlen(s), m = strlen(t);
  if (n != m)
    return 0;
  for (size_t i = 0; i < n; i++)
    if ((char)tolower((unsigned char)s[i]) != (char)tolower((unsigned char)t[i]))
      return 0;
  return 1;
}

// ===== DHT22 low-level helpers =====
static int wait_for_level(unsigned gpio, int level, uint32_t timeout_us)
{
  uint32_t start = gpioTick();
  while (gpioRead(gpio) != level)
  {
    if (gpioTick() - start > timeout_us)
      return -1;
  }
  return 0;
}

static int dht22_read_once(unsigned gpio, float *temp_c, float *rh)
{
  uint8_t data[5] = {0};

  // Start signal
  gpioSetMode(gpio, PI_OUTPUT);
  gpioWrite(gpio, 0);
  gpioDelay(18000); // 18 ms
  gpioWrite(gpio, 1);
  gpioDelay(40); // 40 us

  // Response
  gpioSetMode(gpio, PI_INPUT);
  gpioSetPullUpDown(gpio, PI_PUD_UP);
  if (wait_for_level(gpio, 0, TIMEOUT_US) < 0)
    return -1;
  if (wait_for_level(gpio, 1, TIMEOUT_US) < 0)
    return -2;
  if (wait_for_level(gpio, 0, TIMEOUT_US) < 0)
    return -3;

  // Read 40 bits
  for (int i = 0; i < 40; i++)
  {
    if (wait_for_level(gpio, 1, TIMEOUT_US) < 0)
      return -4; // HIGH start
    uint32_t start_high = gpioTick();
    if (wait_for_level(gpio, 0, TIMEOUT_US) < 0)
      return -5; // HIGH end
    uint32_t high_len = gpioTick() - start_high;

    int bit = (high_len > DHT_THRESHOLD_US) ? 1 : 0;
    data[i / 8] <<= 1;
    data[i / 8] |= bit;
  }

  // Checksum
  uint8_t sum = (uint8_t)(data[0] + data[1] + data[2] + data[3]);
  if (sum != data[4])
    return -6;

  int16_t raw_h = (int16_t)((data[0] << 8) | data[1]);
  int16_t raw_t = (int16_t)((data[2] << 8) | data[3]);
  *rh = raw_h / 10.0f;
  if (raw_t & 0x8000)
  {
    raw_t &= 0x7FFF;
    *temp_c = -(raw_t / 10.0f);
  }
  else
  {
    *temp_c = (raw_t / 10.0f);
  }
  return 0;
}

static int dht22_read_retry(unsigned gpio, float *t, float *h)
{
  int last = -99;
  for (int i = 0; i < DHT_RETRIES; i++)
  {
    int rc = dht22_read_once(gpio, t, h);
    if (rc == 0)
      return 0;
    last = rc;
    gpioDelay(DHT_RETRY_DELAY_MS * 1000);
  }
  return last;
}

// ===== NEC IR helpers =====
#define NEC_HDR_MARK 9000
#define NEC_HDR_SPACE 4500
#define NEC_BIT_MARK 562
#define NEC_ZERO_SPACE 562
#define NEC_ONE_SPACE 1687
#define NEC_END_MARK 562
#define NEC_RPT_MARK 9000
#define NEC_RPT_SPACE 2250
#define NEC_RPT_TAIL 562

static inline void ir_mark(unsigned usec)
{
  int rc = gpioHardwarePWM(IR_PIN, IR_CARRIER_HZ, IR_DUTY_PER_MILL);
  if (rc < 0)
    fprintf(stderr, "PWM mark error rc=%d on GPIO%d\n", rc, IR_PIN);
  gpioDelay(usec);
}
static inline void ir_space(unsigned usec)
{
  int rc = gpioHardwarePWM(IR_PIN, 0, 0);
  if (rc < 0)
    fprintf(stderr, "PWM space error rc=%d on GPIO%d\n", rc, IR_PIN);
  gpioDelay(usec);
}
static inline void carrier_burst_ms(int ms)
{
  int rc = gpioHardwarePWM(IR_PIN, IR_CARRIER_HZ, IR_DUTY_PER_MILL);
  if (rc < 0)
    fprintf(stderr, "PWM mark error rc=%d on GPIO%d\n", rc, IR_PIN);
  gpioDelay(ms * 1000);
  gpioHardwarePWM(IR_PIN, 0, 0);
}

static void ir_send_nec_frame(uint8_t addr, uint8_t cmd)
{
  ir_mark(NEC_HDR_MARK);
  ir_space(NEC_HDR_SPACE);
  uint8_t bytes[4] = {addr, (uint8_t)~addr, cmd, (uint8_t)~cmd};
  for (int b = 0; b < 4; b++)
  {
    uint8_t v = bytes[b];
    for (int i = 0; i < 8; i++)
    {
      ir_mark(NEC_BIT_MARK);
      if (v & 0x01)
        ir_space(NEC_ONE_SPACE);
      else
        ir_space(NEC_ZERO_SPACE);
      v >>= 1;
    }
  }
  ir_mark(NEC_END_MARK);
}
static void ir_send_nec_repeat(void)
{
  ir_mark(NEC_RPT_MARK);
  ir_space(NEC_RPT_SPACE);
  ir_mark(NEC_RPT_TAIL);
}
static void ir_send_nec(uint8_t addr, uint8_t cmd)
{
  ir_send_nec_frame(addr, cmd);
  ir_space(IR_REPEAT_GAP_US);
#if IR_REPEAT_COUNT > 1
  for (int r = 1; r < IR_REPEAT_COUNT; r++)
  {
#if USE_NEC_REPEAT_FRAME
    ir_send_nec_repeat();
#else
    ir_send_nec_frame(addr, cmd);
#endif
    ir_space(IR_REPEAT_GAP_US);
  }
#endif
}
static void fan_send_toggle(void) { ir_send_nec(NEC_ADDR, NEC_CMD_TOGGLE); }

// ============================= Main =====================================
int main(void)
{
  signal(SIGINT, handle_sigint);
  signal(SIGTERM, handle_sigint);

  if (gpioInitialise() < 0)
  {
    fprintf(stderr, "pigpio init failed\n");
    return 1;
  }
  try_set_realtime(50);           // may be denied; ok
  gpioSetMode(IR_PIN, PI_OUTPUT); // harmless with HW PWM

  printf("IR cfg: GPIO=%d, carrier=%u Hz, duty=%.1f%%, NEC addr=0x%02X, toggle=0x%02X\n",
         IR_PIN, IR_CARRIER_HZ, IR_DUTY_PER_MILL / 10000.0, NEC_ADDR, NEC_CMD_TOGGLE);
  printf("Hysteresis: ON >= %.1f°C, OFF <= %.1f°C (no arming)\n", COOL_ON_C, COOL_OFF_C);

  FILE *logf = fopen("log.csv", "a");
  if (!logf)
  {
    perror("log.csv open");
  }
  else
  {
    fprintf(logf, "# timestamp,temperature_c,humidity_pct,fan_on,status\n");
    fflush(logf);
  }

  bool fan_on = false; // assumed initial state

  while (keep_running)
  {
    // ----- manual commands -----
    char line[128];
    while (stdin_readline(line, sizeof line))
    {
      if (is_cmd_eq(line, "test") || is_cmd_eq(line, "nec"))
      {
        char ts[32];
        iso_timestamp(ts, sizeof ts);
        fan_send_toggle();
        fan_on = !fan_on; // assume success to keep local state
        if (logf)
        {
          fprintf(logf, "%s,,,%d,MANUAL_TOGGLE\n", ts, fan_on ? 1 : 0);
          fflush(logf);
        }
        printf("[%s] MANUAL: sent NEC TOGGLE  addr=0x%02X cmd=0x%02X  FAN=%d\n",
               ts, NEC_ADDR, NEC_CMD_TOGGLE, fan_on);
      }
      else if (is_cmd_eq(line, "car"))
      {
        carrier_burst_ms(300);
        printf("Manual: 38kHz carrier burst 300ms sent on GPIO%d\n", IR_PIN);
      }
      else
      {
        printf("Unknown cmd: %s  (use: test|nec|car)\n", line);
      }
    }

    // ----- sensor read with retry -----
    float t = 0.0f, h = 0.0f;
    int rc = dht22_read_retry(DHT_PIN, &t, &h);

    char ts[32];
    iso_timestamp(ts, sizeof ts);

    if (rc == 0)
    {
      // ----- pure hysteresis (no arming) -----
      if (!fan_on && t >= COOL_ON_C)
      {
        fan_send_toggle();
        fan_on = true;
      }
      else if (fan_on && t <= COOL_OFF_C)
      {
        fan_send_toggle();
        fan_on = false;
      }

      if (logf)
      {
        fprintf(logf, "%s,%.1f,%.1f,%d,OK\n", ts, t, h, fan_on ? 1 : 0);
        fflush(logf);
      }
      printf("[%s] T=%.1fC, RH=%.1f%% | FAN=%d | OK\n", ts, t, h, fan_on);
    }
    else
    {
      if (logf)
      {
        fprintf(logf, "%s,,,%d,READ_FAIL(%d)\n", ts, fan_on ? 1 : 0, rc);
        fflush(logf);
      }
      fprintf(stderr, "[%s] DHT22 read failed rc=%d\n", ts, rc);
    }

    // ----- sleep slices -----
    for (int i = 0; i < LOOP_PERIOD_S * 10 && keep_running; i++)
      gpioDelay(100000);
  }

  gpioHardwarePWM(IR_PIN, 0, 0);
  if (logf)
    fclose(logf);
  gpioTerminate();
  return 0;
}
