// main.c - RPi 3B+ + DHT22 + pigpio + IR (NEC, POWER TOGGLE only) fan control + CSV logging
// BCM numbering. Requires sudo.
// Build: make
// Run:   sudo ./tempctrl
//
// Notes:
// - NEC remote with only POWER TOGGLE available.
// - We use an "arming" strategy to avoid desync on boot: no IR is sent until temperature first
//   drops into the "OFF band" (<= COOL_OFF_C). After armed, we send toggle only on threshold crossings.
// - You MUST fill NEC_ADDR and NEC_CMD_TOGGLE with your remote's captured values.
//
// Hardware:
// - IR LED on BCM18 (hardware PWM). Drive via NPN transistor + series resistor. Do NOT drive LED directly.
// - DHT22 on BCM4 with 10k pull-up to 3.3V.

#include <pigpio.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <signal.h>
#include <time.h>
#include <string.h>
#include <stdbool.h>

// ================== Pins (BCM) ==================
#define DHT_PIN 4
#define IR_PIN 18

// ================== Control (hysteresis) ==================
#define COOL_ON_C 28.0f
#define COOL_OFF_C 26.0f

// Require arming pass (enter <= COOL_OFF_C once before sending any IR)
#define REQUIRE_ARMING 1

// Loop period (seconds)
#define LOOP_PERIOD_S 2

// DHT22 timeout (microseconds)
#define TIMEOUT_US 200000

// ================== NEC IR config ==================
#define IR_CARRIER_HZ 38000
#define IR_DUTY_PER_MILL 330000 // ~33% (0..1,000,000)

// Send exactly one full NEC frame per action to avoid double toggles
#define IR_REPEAT_COUNT 1
#define IR_REPEAT_GAP_US 40000
#define USE_NEC_REPEAT_FRAME 0 // keep 0 for power toggle

// ---- FILL THESE WITH YOUR CAPTURED CODES ----
#define NEC_ADDR 0x00       // <<< replace with your fan's NEC address (8-bit)
#define NEC_CMD_TOGGLE 0x02 // <<< replace with your fan's POWER (toggle) command (8-bit)
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

  // End
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
  // Send exactly one press; repeat disabled by default for toggle safety
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

  bool armed = (REQUIRE_ARMING == 0); // if arming disabled, start armed
  bool fan_on = false;                // our believed state (may differ from reality until armed)

  while (keep_running)
  {
    float t = 0.0f, h = 0.0f;
    int rc = dht22_read(DHT_PIN, &t, &h);

    char ts[32];
    iso_timestamp(ts, sizeof ts);

    if (rc == 0)
    {
      if (!armed)
      {
        if (t <= COOL_OFF_C)
          armed = true; // armed after entering OFF band once
      }
      else
      {
        // Hysteresis with POWER TOGGLE
        if (!fan_on && t >= COOL_ON_C)
        {
          fan_send_toggle();
          fan_on = true; // assume success
        }
        else if (fan_on && t <= COOL_OFF_C)
        {
          fan_send_toggle();
          fan_on = false; // assume success
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
      // Sensor read failed -> do not send IR (can't infer safe action)
      if (logf)
      {
        fprintf(logf, "%s,,,%d,READ_FAIL(%d)\n", ts, fan_on ? 1 : 0, rc);
        fflush(logf);
      }
      fprintf(stderr, "[%s] DHT22 read failed rc=%d\n", ts, rc);
    }

    // Sleep in slices to respond to signals
    for (int i = 0; i < LOOP_PERIOD_S * 10 && keep_running; i++)
      gpioDelay(100000);
  }

  gpioHardwarePWM(IR_PIN, 0, 0);
  if (logf)
    fclose(logf);
  gpioTerminate();
  return 0;
}