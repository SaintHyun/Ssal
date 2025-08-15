// main.c - RPi 3B+ + DHT22 + pigpio + IR (NEC, POWER TOGGLE only) + CSV logging
// Adds: type "test" + Enter in the same terminal to send a manual IR toggle.
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

// ================== Pins (BCM) ==================
#define DHT_PIN 4
#define IR_PIN 18

// ================== Control (hysteresis) ==================
#define COOL_ON_C 28.0f
#define COOL_OFF_C 26.0f

// Require arming pass (enter <= COOL_OFF_C once before auto IR)
#define REQUIRE_ARMING 1

// Loop period (seconds)
#define LOOP_PERIOD_S 2

// DHT22 timeout (microseconds)
#define TIMEOUT_US 200000

// ================== NEC IR config ==================
#define IR_CARRIER_HZ 38000
#define IR_DUTY_PER_MILL 330000 // ~33% (0..1,000,000)

// For toggle safety keep a single press by default
#define IR_REPEAT_COUNT 1
#define IR_REPEAT_GAP_US 40000
#define USE_NEC_REPEAT_FRAME 0

// ---- FILL THESE WITH YOUR CAPTURED CODES ----
#define NEC_ADDR 0x00       // <<< put your remote's NEC address
#define NEC_CMD_TOGGLE 0x45 // <<< put your remote's POWER (toggle)
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

static inline void carrier_burst_ms(int ms)
{
  gpioHardwarePWM(IR_PIN, IR_CARRIER_HZ, IR_DUTY_PER_MILL);
  gpioDelay(ms * 1000);
  gpioHardwarePWM(IR_PIN, 0, 0);
}

static int is_cmd_eq(const char *s, const char *t)
{
  size_t n = strlen(s), m = strlen(t);
  if (n != m)
    return 0;
  for (size_t i = 0; i < n; i++)
  {
    if ((char)tolower((unsigned char)s[i]) != (char)tolower((unsigned char)t[i]))
      return 0;
  }
  return 1;
}

// Non-blocking read of a full line from stdin if available.
// Returns 1 if a line was read into buf (stripped of trailing newline), 0 otherwise.
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
      {
        buf[--n] = '\0';
      }
      return 1;
    }
  }
  return 0;
}

// Case-insensitive equality to "test"
static int is_cmd_test(const char *s)
{
  const char *t = "test";
  size_t n = strlen(s);
  if (n != 4)
    return 0;
  for (size_t i = 0; i < 4; i++)
  {
    if ((char)tolower((unsigned char)s[i]) != t[i])
      return 0;
  }
  return 1;
}

// ===== DHT22 helpers =====
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

static int dht22_read(unsigned gpio, float *temp_c, float *rh)
{
  uint8_t data[5] = {0};

  // Start signal
  gpioSetMode(gpio, PI_OUTPUT);
  gpioWrite(gpio, 0);
  gpioDelay(18000);
  gpioWrite(gpio, 1);
  gpioDelay(40);

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
      return -4;
    uint32_t start_high = gpioTick();
    if (wait_for_level(gpio, 0, TIMEOUT_US) < 0)
      return -5;
    uint32_t high_len = gpioTick() - start_high;
    int bit = (high_len > 50) ? 1 : 0;
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
  gpioHardwarePWM(IR_PIN, IR_CARRIER_HZ, IR_DUTY_PER_MILL);
  gpioDelay(usec);
}

static inline void ir_space(unsigned usec)
{
  gpioHardwarePWM(IR_PIN, 0, 0);
  gpioDelay(usec);
}

static void ir_send_nec_frame(uint8_t addr, uint8_t cmd)
{
  // Header
  ir_mark(NEC_HDR_MARK);
  ir_space(NEC_HDR_SPACE);

  // Payload: addr, ~addr, cmd, ~cmd (LSB first)
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

static void fan_send_toggle(void)
{
  ir_send_nec(NEC_ADDR, NEC_CMD_TOGGLE);
}

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
  gpioSetMode(IR_PIN, PI_OUTPUT); // harmless with hardware PWM

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

  bool armed = (REQUIRE_ARMING == 0);
  bool fan_on = false;

  while (keep_running)
  {
    // ---------- 1) Handle manual input (non-blocking) ----------
    // Drain all pending lines to avoid backlog
    char line[128];
    while (stdin_readline(line, sizeof line))
    {
      if (is_cmd_eq(line, "test") || is_cmd_eq(line, "nec"))
      {
        char ts[32];
        iso_timestamp(ts, sizeof ts);
        // NEC 한 프레임 전송 (토글)
        fan_send_toggle();
        // 로컬 상태 반전
        fan_on = !fan_on;
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
        // 38kHz 캐리어만 300ms 쏘기 (IR 데이터 없이) -> 카메라로 발광 확인용
        carrier_burst_ms(300);
        printf("Manual: 38kHz carrier burst 300ms sent on GPIO%d\n", IR_PIN);
      }
      else
      {
        printf("Unknown cmd: %s  (use: test|nec|car)\n", line);
      }
    }

    // ---------- 2) Sensor read ----------
    float t = 0.0f, h = 0.0f;
    int rc = dht22_read(DHT_PIN, &t, &h);

    char ts[32];
    iso_timestamp(ts, sizeof ts);

    if (rc == 0)
    {
      // ---------- 3) Auto control ----------
      if (!armed)
      {
        if (t <= COOL_OFF_C)
          armed = true;
      }
      else
      {
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
      }

      if (logf)
      {
        fprintf(logf, "%s,%.1f,%.1f,%d,%s\n",
                ts, t, h, fan_on ? 1 : 0, armed ? "OK" : "PENDING_ARM");
        fflush(logf);
      }
      printf("[%s] T=%.1fC, RH=%.1f%% | FAN=%d | %s\n",
             ts, t, h, fan_on, armed ? "ARMED" : "PENDING_ARM");
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

    // ---------- 4) Sleep (in slices) ----------
    for (int i = 0; i < LOOP_PERIOD_S * 10 && keep_running; i++)
      gpioDelay(100000);
  }

  gpioHardwarePWM(IR_PIN, 0, 0);
  // flush stdin (not necessary)
  if (logf)
    fclose(logf);
  gpioTerminate();
  return 0;
}