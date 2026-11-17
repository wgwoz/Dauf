/*
 * dauf.c - Mini-sumo firmware (single file), LINE-SENSOR-ONLY strategy.
 *
 * Hardware: RP2350 (Pico 2) + FreeRTOS, two QRE1113E line sensors, P1R RC-5
 * start module input, FM24CL16B FRAM (I2C), differential drive.
 * (The VL53L0X opponent sensors are not used by this strategy and were removed.)
 *
 * IR receiver (GPIO IR_RX_PIN), selected by the mode switch on GPIO 17:
 *   - HIGH = COMPETITION mode: behaves like a P1R start module
 *            (RC-5, address 0x07 start/stop, address 0x0B programming)
 *   - LOW  = TEST mode: NEC commands (see table below)
 *
 * TEST-mode NEC commands:
 *   0x43  toggle line-sensor strategy on/off (LED on = running)
 *   0x09  manual straight reverse pulse (START_REVERSE_MS)
 *   0x40  manual straight reverse pulse (EDGE_ESCAPE_MS)
 *   0x44  CALIBRATION: capture reading with both sensors over the DOHYO
 *   0x45  CALIBRATION: capture reading with both sensors over the LINE
 *   0x46  reset QRE thresholds to compile-time defaults (and save)
 *
 * QRE threshold calibration:
 *   1. Switch to TEST mode, open the serial console.
 *   2. Put BOTH line sensors over the dohyo (black)  -> send 0x44
 *   3. Put BOTH line sensors over the white line     -> send 0x45
 *      (order of 2 and 3 does not matter)
 *   After both captures: threshold = midpoint per sensor, "line is lower/higher
 *   than dohyo" is inferred, and the result is saved to FRAM. It is loaded at
 *   every boot. If FRAM has no valid record, the compile-time defaults are used.
 *   Calibration is rejected if dohyo and line readings are < 200 us apart.
 *
 * STRATEGY (QRE only):
 *   FORWARD : drive forward with slow alternating gentle left/right arcs.
 *   BACKOFF : line seen -> reverse straight until both sensors are clear.
 *   TURN    : turn away from the side that saw the line, then FORWARD again.
 *               left sensor only  -> turn right (clockwise)
 *               right sensor only -> turn left
 *               both sensors      -> longer turn (~180 deg) to the right
 *   Competition mode: runs only while the P1R state is STARTED.
 *
 * Competition state is controlled only by the P1R RC-5 start/stop commands.
 * The state and programmed dohyo command are restored from FRAM.
 */
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"

/* FreeRTOS Headers */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "hardware/sync.h"

#define LED_PIN          25   /* Pico 2 built-in LED (NOT valid on Pico 2 W) */

/* Mode switch + IR receiver */
#define MODE_SWITCH_PIN    17   /* physical switch: competition / test */
#define COMPETITION_LEVEL  1    /* pin HIGH = competition mode */
#define IR_RX_PIN          13   /* <<< CHANGE to the GPIO your IR receiver output is on */

/* Non-volatile storage (FRAM on the I2C bus) */
#define FRAM_I2C_ADDR      0x50 /* FM24CL16B base address; A10:A8 are in the slave address */
#define FRAM_NV_ADDR       0x0000  /* P1R state record (4 bytes) */
#define FRAM_CAPACITY      0x0800  /* FM24CL16B: 2 KiB = 0x000..0x7FF */

/* I2C */
#define I2C_PORT         i2c0
#define I2C_SDA_PIN      4
#define I2C_SCL_PIN      5
#define I2C_BAUD_RATE    (400 * 1000)
#define I2C_TIMEOUT_US   5000

/* Motor PWM: native Pico SDK compare levels, 20 kHz carrier from actual clk_sys.
 * Commands are permille duty (0..1000). */
#define MOTOR_PWM_FREQUENCY_HZ 20000u
#define MOTOR_MIN_DUTY_PERMILLE 200u   /* deadband, needs bench calibration */
#define MOTOR_FULL_DUTY_PERMILLE 1000u
#define MOTOR_LEFT_FWD_PIN  21u
#define MOTOR_LEFT_REV_PIN  22u
#define MOTOR_RIGHT_FWD_PIN 20u
#define MOTOR_RIGHT_REV_PIN 19u
#define MOTOR_FORWARD_PWM   INT16_C(1000)
#define MOTOR_REVERSE_PWM   INT16_C(1000)
#define START_REVERSE_MS    500u   /* manual test pulse 0x09 (and optional start reverse) */
#define EDGE_ESCAPE_MS      500u   /* manual test pulse 0x40 */

/* ---- Strategy tunables (times in ms, PWM in permille 0..1000) ---- */
#define DAUF_FWD_PWM            INT16_C(1000)
#define DAUF_ARC_INNER_PWM      INT16_C(700)    /* slower wheel while arcing */
#define DAUF_ARC_PERIOD_MS      1200u           /* arc direction flips this often */
#define DAUF_BACKOFF_PWM        INT16_C(1000)
#define DAUF_BACKOFF_MIN_MS     120u
#define DAUF_BACKOFF_MAX_MS     500u
#define DAUF_TURN_PWM           INT16_C(800)
#define DAUF_TURN_SIDE_MS       220u            /* tune: about 90 deg */
#define DAUF_TURN_BOTH_MS       420u            /* tune: about 180 deg */
#define DAUF_USE_START_REVERSE  1               /* 1 = reverse START_REVERSE_MS before driving */

/* QRE1113E hardware */
#define QRE_LINE_SENSOR_COUNT 2
#define QRE_LINE_PIN_0        6     /* sensor 0 = LEFT  */
#define QRE_LINE_PIN_1        15    /* sensor 1 = RIGHT */
#define QRE_CHARGE_US         15
#define QRE_TIMEOUT_US        10000
#define QRE_TASK_PERIOD_MS    5
#define QRE_TEST_PRINT        1     /* 1 = print readings at 2 Hz in TEST mode */

/* QRE defaults, used when FRAM holds no valid calibration.
 * In capacitor-discharge mode, a brighter surface normally discharges faster. */
#define QRE_EDGE_THRESHOLD_US 1000u
#define QRE_EDGE_IS_LOWER     1
#define QRE_EDGE_CLEAR_MARGIN_US 100u

/* QRE calibration (stored in FRAM at QRE_NV_ADDR) */
#define QRE_NV_ADDR              0x0010u  /* 8-byte record, inside one 256-byte page */
#define QRE_NV_MAGIC             0xC7u
#define QRE_CAL_SAMPLES          16
#define QRE_CAL_SAMPLE_GAP_MS    10
#define QRE_CAL_MIN_SEPARATION_US 200u    /* reject calibration if surfaces look alike */

/* NEC test commands */
#define TEST_RUN_TOGGLE_CMD    0x43u
#define TEST_REVERSE_CMD       0x09u
#define TEST_EDGE_ESCAPE_CMD   0x40u
#define TEST_QRE_DOHYO_CMD     0x44u
#define TEST_QRE_LINE_CMD      0x45u
#define TEST_QRE_RESET_CMD     0x46u
#define TEST_TOGGLE_GUARD_MS   1000u

/* ---------------------------------------------------------------- */
/* Basics + I2C / FRAM access                                       */
/* ---------------------------------------------------------------- */

static inline bool competition_mode(void) {
    return gpio_get(MODE_SWITCH_PIN) == COMPETITION_LEVEL;
}

/* The I2C bus is shared with other users, so every transaction is locked */
static SemaphoreHandle_t i2c_mutex = NULL;
static inline void i2c_lock(void)   { if (i2c_mutex) xSemaphoreTake(i2c_mutex, portMAX_DELAY); }
static inline void i2c_unlock(void) { if (i2c_mutex) xSemaphoreGive(i2c_mutex); }

/* FM24CL16B does NOT take a 16-bit memory address. Its 11-bit address is
 * split between the I2C slave address (A10:A8 -> address bits 3:1) and one
 * word-address byte (A7:A0). Keep each transaction within one 256-byte page.
 */
static bool fram_address(uint16_t addr, size_t n, uint8_t *dev_addr, uint8_t *word_addr) {
    if (n == 0 || addr >= FRAM_CAPACITY || n > (size_t)(FRAM_CAPACITY - addr)) return false;
    if (n > (size_t)(0x100u - (addr & 0x00FFu))) return false;

    *dev_addr  = (uint8_t)(FRAM_I2C_ADDR | ((addr >> 8) & 0x07u));
    *word_addr = (uint8_t)(addr & 0x00FFu);
    return true;
}

static bool fram_write(uint16_t addr, const uint8_t *d, size_t n) {
    uint8_t buf[1 + 8];
    uint8_t dev_addr, word_addr;
    if (d == NULL || n > 8 || !fram_address(addr, n, &dev_addr, &word_addr)) return false;

    buf[0] = word_addr;
    for (size_t i = 0; i < n; i++) buf[1 + i] = d[i];

    i2c_lock();
    bool ok = i2c_write_timeout_us(I2C_PORT, dev_addr, buf, 1 + n, false, I2C_TIMEOUT_US) == (int)(1 + n);
    i2c_unlock();
    return ok;
}

static bool fram_read(uint16_t addr, uint8_t *d, size_t n) {
    uint8_t dev_addr, word_addr;
    if (d == NULL || !fram_address(addr, n, &dev_addr, &word_addr)) return false;

    i2c_lock();
    bool ok = (i2c_write_timeout_us(I2C_PORT, dev_addr, &word_addr, 1, true, I2C_TIMEOUT_US) == 1) &&
              (i2c_read_timeout_us(I2C_PORT, dev_addr, d, n, false, I2C_TIMEOUT_US) == (int)n);
    i2c_unlock();
    return ok;
}

/* ---------------------------------------------------------------- */
/* P1R state record in FRAM                                         */
/* ---------------------------------------------------------------- */

typedef enum { SM_POWER_ON = 0, SM_STARTED = 1, SM_STOPPED = 2 } sm_state_t;

static volatile sm_state_t g_sm_state   = SM_POWER_ON;
static volatile bool       g_programmed = false;
static volatile uint8_t    g_stop_cmd   = 0;      /* start command = stop command + 1 */

static volatile bool       prog_blink_active = false;
static volatile TickType_t prog_blink_start = 0;

/* The strategy only drives the motors when this is SM_STARTED (competition). */
sm_state_t start_module_state(void) { return g_sm_state; }

/* Record: [magic][state][stopCmd | programmed(bit0)][~(sum of previous 3)] */
#define NV_MAGIC 0xB2

/* Write the record and read it back to verify (up to 3 attempts) */
static bool nv_record_write(const uint8_t r[4]) {
    uint8_t chk[4];
    for (int attempt = 0; attempt < 3; attempt++) {
        if (fram_write(FRAM_NV_ADDR, r, 4) &&
            fram_read(FRAM_NV_ADDR, chk, 4) &&
            memcmp(chk, r, 4) == 0) return true;
    }
    return false;
}

static void nv_save(void) {
    uint8_t r[4];
    r[0] = NV_MAGIC;
    r[1] = (uint8_t)g_sm_state;
    r[2] = (uint8_t)((g_stop_cmd & 0xFE) | (g_programmed ? 1 : 0));
    r[3] = (uint8_t)~(r[0] + r[1] + r[2]);
    if (!nv_record_write(r)) printf("[NV] FRAM write/verify failed (state not persistent)\n");
}

/* Boot behaviour from the P1R spec: restore the saved state; a saved
 * "Stopped" becomes "Power-ON". Invalid/missing record = not programmed. */
static void nv_load(void) {
    uint8_t r[4];
    g_sm_state = SM_POWER_ON;
    g_programmed = false;
    g_stop_cmd = 0;
    if (!fram_read(FRAM_NV_ADDR, r, 4)) {
        printf("[NV] FRAM read failed (state not persistent), starting unprogrammed\n");
        return;
    }
    if (r[0] == NV_MAGIC && r[3] == (uint8_t)~(r[0] + r[1] + r[2]) && r[1] <= SM_STOPPED) {
        g_programmed = (r[2] & 1) != 0;
        g_stop_cmd   = r[2] & 0xFE;
        g_sm_state   = (r[1] == SM_STARTED) ? SM_STARTED : SM_POWER_ON;
        if (r[1] == SM_STOPPED) nv_save();     /* saved STOPPED -> POWER-ON, persist that */
        printf("[NV] Restored: programmed=%d stopCmd=0x%02X state=%s\n",
               g_programmed, g_stop_cmd,
               g_sm_state == SM_STARTED ? "STARTED" : "POWER-ON");
    } else {
        printf("[NV] No valid saved record: unprogrammed, POWER-ON\n");
    }
}

/* ---------------------------------------------------------------- */
/* QRE1113E line sensors: thresholds in FRAM + calibration          */
/* ---------------------------------------------------------------- */

/* Latest discharge times in microseconds. Timeout/no discharge is reported
 * as QRE_TIMEOUT_US. */
volatile uint32_t qre_line_sensor_us[QRE_LINE_SENSOR_COUNT] = {0, 0};

static const uint8_t qre_line_pins[QRE_LINE_SENSOR_COUNT] = {
    QRE_LINE_PIN_0, QRE_LINE_PIN_1
};

/* Active thresholds, one per sensor (index matches qre_line_sensor_us[]) */
static volatile uint16_t qre_thr_us[QRE_LINE_SENSOR_COUNT] = {
    QRE_EDGE_THRESHOLD_US, QRE_EDGE_THRESHOLD_US
};
static volatile bool qre_edge_lower[QRE_LINE_SENSOR_COUNT] = {
    QRE_EDGE_IS_LOWER, QRE_EDGE_IS_LOWER
};

/* FRAM record (8 bytes at QRE_NV_ADDR):
 *   [0] magic
 *   [1] flags: bit0 = sensor 0 line reads LOWER than dohyo, bit1 = sensor 1
 *   [2][3] threshold sensor 0 (us, little endian)
 *   [4][5] threshold sensor 1 (us, little endian)
 *   [6] reserved (0)
 *   [7] ~(sum of bytes 0..6)
 */
static uint8_t qre_nv_checksum(const uint8_t r[8]) {
    uint8_t sum = 0;
    for (int i = 0; i < 7; i++) sum = (uint8_t)(sum + r[i]);
    return (uint8_t)~sum;
}

static bool qre_nv_save(void) {
    uint8_t r[8] = {0};
    uint8_t chk[8];
    r[0] = QRE_NV_MAGIC;
    r[1] = (uint8_t)((qre_edge_lower[0] ? 1u : 0u) | (qre_edge_lower[1] ? 2u : 0u));
    r[2] = (uint8_t)(qre_thr_us[0] & 0xFFu);
    r[3] = (uint8_t)(qre_thr_us[0] >> 8);
    r[4] = (uint8_t)(qre_thr_us[1] & 0xFFu);
    r[5] = (uint8_t)(qre_thr_us[1] >> 8);
    r[6] = 0;
    r[7] = qre_nv_checksum(r);

    for (int attempt = 0; attempt < 3; attempt++) {
        if (fram_write(QRE_NV_ADDR, r, 8) &&
            fram_read(QRE_NV_ADDR, chk, 8) &&
            memcmp(chk, r, 8) == 0) return true;
    }
    printf("[QRE NV] FRAM write/verify failed (thresholds not persistent)\n");
    return false;
}

/* Call once at boot, after the I2C bus + mutex are ready. */
static void qre_nv_load(void) {
    uint8_t r[8];
    if (!fram_read(QRE_NV_ADDR, r, 8)) {
        printf("[QRE NV] FRAM read failed, using default thresholds\n");
    } else if (r[0] == QRE_NV_MAGIC && r[7] == qre_nv_checksum(r)) {
        uint16_t t0 = (uint16_t)(r[2] | ((uint16_t)r[3] << 8));
        uint16_t t1 = (uint16_t)(r[4] | ((uint16_t)r[5] << 8));
        if (t0 > 0 && t0 < QRE_TIMEOUT_US && t1 > 0 && t1 < QRE_TIMEOUT_US) {
            qre_thr_us[0] = t0;
            qre_thr_us[1] = t1;
            qre_edge_lower[0] = (r[1] & 1u) != 0;
            qre_edge_lower[1] = (r[1] & 2u) != 0;
            printf("[QRE NV] Loaded: S0 thr=%u us (%s)  S1 thr=%u us (%s)\n",
                   (unsigned)t0, qre_edge_lower[0] ? "line lower" : "line higher",
                   (unsigned)t1, qre_edge_lower[1] ? "line lower" : "line higher");
            return;
        }
        printf("[QRE NV] Stored thresholds out of range, using defaults\n");
    } else {
        printf("[QRE NV] No valid record, using default thresholds\n");
    }
    printf("[QRE NV] Defaults: thr=%u us (%s)\n", (unsigned)QRE_EDGE_THRESHOLD_US,
           QRE_EDGE_IS_LOWER ? "line lower" : "line higher");
}

/* Per-sensor edge tests, i = index into qre_line_sensor_us[] */
static bool qre_edge_detected_i(size_t i, uint32_t reading) {
    return qre_edge_lower[i] ? (reading <= qre_thr_us[i])
                             : (reading >= qre_thr_us[i]);
}

static bool qre_edge_cleared_i(size_t i, uint32_t reading) {
    return qre_edge_lower[i] ? (reading > (uint32_t)qre_thr_us[i] + QRE_EDGE_CLEAR_MARGIN_US)
                             : (reading + QRE_EDGE_CLEAR_MARGIN_US < (uint32_t)qre_thr_us[i]);
}

/* ---- Calibration (runs in the IR task, blocks ~160 ms per capture) ---- */
static uint32_t qre_cal_dohyo[QRE_LINE_SENSOR_COUNT];
static uint32_t qre_cal_line[QRE_LINE_SENSOR_COUNT];
static bool qre_have_dohyo = false;
static bool qre_have_line = false;

static void qre_sample_average(uint32_t out[QRE_LINE_SENSOR_COUNT]) {
    uint32_t sum[QRE_LINE_SENSOR_COUNT] = {0};
    for (int s = 0; s < QRE_CAL_SAMPLES; s++) {
        for (size_t i = 0; i < QRE_LINE_SENSOR_COUNT; i++) sum[i] += qre_line_sensor_us[i];
        vTaskDelay(pdMS_TO_TICKS(QRE_CAL_SAMPLE_GAP_MS));
    }
    for (size_t i = 0; i < QRE_LINE_SENSOR_COUNT; i++) out[i] = sum[i] / QRE_CAL_SAMPLES;
}

/* is_line = false: sensors are over the dohyo; true: sensors are over the line */
static void qre_nv_calibrate_capture(bool is_line) {
    uint32_t avg[QRE_LINE_SENSOR_COUNT];
    qre_sample_average(avg);

    for (size_t i = 0; i < QRE_LINE_SENSOR_COUNT; i++) {
        if (is_line) qre_cal_line[i] = avg[i];
        else         qre_cal_dohyo[i] = avg[i];
    }
    if (is_line) qre_have_line = true; else qre_have_dohyo = true;
    printf("[QRE CAL] %s captured: S0=%lu us S1=%lu us\n",
           is_line ? "LINE" : "DOHYO", (unsigned long)avg[0], (unsigned long)avg[1]);

    if (!(qre_have_line && qre_have_dohyo)) {
        printf("[QRE CAL] now capture the %s\n", is_line ? "DOHYO (0x44)" : "LINE (0x45)");
        return;
    }

    uint16_t new_thr[QRE_LINE_SENSOR_COUNT];
    bool new_lower[QRE_LINE_SENSOR_COUNT];
    for (size_t i = 0; i < QRE_LINE_SENSOR_COUNT; i++) {
        uint32_t a = qre_cal_dohyo[i], b = qre_cal_line[i];
        uint32_t diff = a > b ? a - b : b - a;
        if (diff < QRE_CAL_MIN_SEPARATION_US) {
            printf("[QRE CAL] FAILED: sensor %u dohyo/line too similar (%lu vs %lu us). Redo both captures.\n",
                   (unsigned)i, (unsigned long)a, (unsigned long)b);
            qre_have_line = qre_have_dohyo = false;
            return;
        }
        new_thr[i] = (uint16_t)((a + b) / 2u);
        new_lower[i] = b < a;
    }

    for (size_t i = 0; i < QRE_LINE_SENSOR_COUNT; i++) {
        qre_thr_us[i] = new_thr[i];
        qre_edge_lower[i] = new_lower[i];
    }
    qre_have_line = qre_have_dohyo = false;
    bool saved = qre_nv_save();
    printf("[QRE CAL] New thresholds: S0=%u us (%s) S1=%u us (%s) -> %s\n",
           (unsigned)new_thr[0], new_lower[0] ? "line lower" : "line higher",
           (unsigned)new_thr[1], new_lower[1] ? "line lower" : "line higher",
           saved ? "saved to FRAM" : "NOT saved");
}

static void qre_nv_reset_defaults(void) {
    for (size_t i = 0; i < QRE_LINE_SENSOR_COUNT; i++) {
        qre_thr_us[i] = QRE_EDGE_THRESHOLD_US;
        qre_edge_lower[i] = QRE_EDGE_IS_LOWER;
    }
    qre_have_line = qre_have_dohyo = false;
    bool saved = qre_nv_save();
    printf("[QRE CAL] Reset to defaults: thr=%u us (%s) -> %s\n",
           (unsigned)QRE_EDGE_THRESHOLD_US, QRE_EDGE_IS_LOWER ? "line lower" : "line higher",
           saved ? "saved to FRAM" : "NOT saved");
}

/* ---- Capacitor-discharge measurement + task ---- */
static uint32_t qre_read_discharge_us(uint pin) {
    /* Charge the sensor's capacitance through the GPIO output. */
    gpio_set_dir(pin, GPIO_OUT);
    gpio_put(pin, 1);
    busy_wait_us_32(QRE_CHARGE_US);

    /* Float the pin and time the capacitor's discharge through the sensor. */
    gpio_set_dir(pin, GPIO_IN);
    gpio_disable_pulls(pin);

    const uint32_t start_us = time_us_32();
    while (gpio_get(pin)) {
        const uint32_t elapsed_us = time_us_32() - start_us;
        if (elapsed_us >= QRE_TIMEOUT_US) {
            return QRE_TIMEOUT_US;
        }
    }

    return time_us_32() - start_us;
}

void vQRELineSensorTask(void *pvParameters) {
    (void)pvParameters;

    for (size_t i = 0; i < QRE_LINE_SENSOR_COUNT; ++i) {
        gpio_init(qre_line_pins[i]);
        gpio_disable_pulls(qre_line_pins[i]);
        gpio_set_dir(qre_line_pins[i], GPIO_IN);
    }

    uint32_t last_print_ms = 0;

    for (;;) {
        for (size_t i = 0; i < QRE_LINE_SENSOR_COUNT; ++i) {
            qre_line_sensor_us[i] = qre_read_discharge_us(qre_line_pins[i]);
        }

#if QRE_TEST_PRINT
        /* In TEST mode print both readings and the active thresholds at 2 Hz. */
        const uint32_t now_ms = to_ms_since_boot(get_absolute_time());
        if (!competition_mode() && (now_ms - last_print_ms >= 500)) {
            printf("[QRE TEST] S0: %lu us (thr %u, %s, %s) | S1: %lu us (thr %u, %s, %s)\n",
                   (unsigned long)qre_line_sensor_us[0], (unsigned)qre_thr_us[0],
                   qre_edge_lower[0] ? "low=line" : "high=line",
                   qre_edge_detected_i(0, qre_line_sensor_us[0]) ? "LINE" : "dohyo",
                   (unsigned long)qre_line_sensor_us[1], (unsigned)qre_thr_us[1],
                   qre_edge_lower[1] ? "low=line" : "high=line",
                   qre_edge_detected_i(1, qre_line_sensor_us[1]) ? "LINE" : "dohyo");
            last_print_ms = now_ms;
        }
#else
        (void)last_print_ms;
#endif

        vTaskDelay(pdMS_TO_TICKS(QRE_TASK_PERIOD_MS));
    }
}

/* ---------------------------------------------------------------- */
/* GPIO ISR: IR receiver edge capture                               */
/* ---------------------------------------------------------------- */

/* IR edge capture (decoded later by vIRCommandTask) */
#define IR_MAX_EDGES   80
static volatile uint32_t ir_edge_t[IR_MAX_EDGES];
static volatile uint16_t ir_edge_cnt = 0;
static volatile uint32_t ir_last_edge_us = 0;
static volatile bool     ir_first_is_fall = false;
static volatile bool     ir_edge_overflow = false;

/* The Pico SDK allows one shared GPIO callback per core; this callback
 * captures the IR receiver edges. */
static void gpio_isr(uint gpio, uint32_t events) {
    if (gpio == IR_RX_PIN) {
        uint32_t now = time_us_32();
        if (ir_edge_cnt == 0) ir_first_is_fall = (events & GPIO_IRQ_EDGE_FALL) != 0;
        if (ir_edge_cnt < IR_MAX_EDGES) ir_edge_t[ir_edge_cnt++] = now;
        else ir_edge_overflow = true;
        ir_last_edge_us = now;
    }
}

/* ---------------------------------------------------------------- */
/* IR command task                                                  */
/*   competition mode: P1R start module (RC-5, p1r.se spec)         */
/*   test mode       : standard NEC                                 */
/* ---------------------------------------------------------------- */

/* P1R protocol */
#define P1R_ADDR_CMD       0x07   /* start / stop (last command bit: 1 = start, 0 = stop) */
#define P1R_ADDR_PROGRAM   0x0B   /* programming: selects which remote / dohyo to obey */

#define IR_POLL_MS         5
#define IR_FRAME_GAP_US    7000   /* silence that ends a frame (NEC leading space is 4.5 ms) */

/* ---- P1R state machine ---- */
/* ---------------------------------------------------------------- */
/* P1R state machine                                                */
/* ---------------------------------------------------------------- */

static void p1r_handle(uint8_t addr, uint8_t cmd) {
    if (addr == P1R_ADDR_PROGRAM) {
        if (g_sm_state == SM_STOPPED) return;            /* stopped: ignore until power cycle */
        g_stop_cmd   = cmd & 0xFE;
        g_programmed = true;
        g_sm_state   = SM_POWER_ON;                      /* motors must stop: state is not STARTED */
        nv_save();
        prog_blink_start  = xTaskGetTickCount();
        prog_blink_active = true;
        printf("[P1R] Programmed: stopCmd=0x%02X startCmd=0x%02X -> POWER-ON\n",
               g_stop_cmd, (uint8_t)(g_stop_cmd + 1));
    } else if (addr == P1R_ADDR_CMD) {
        if (!g_programmed || g_sm_state == SM_STOPPED) return;
        
        /* Mask out the lowest bit to compare the base command, 
           matching how the remote packs ID into even (stop) and odd (start) commands */
        uint8_t base_cmd = cmd & 0xFE;
        uint8_t saved_base_cmd = g_stop_cmd & 0xFE;

        if (base_cmd == saved_base_cmd) {
            if ((cmd & 1) == 0) {
                /* Even command = STOP */
                g_sm_state = SM_STOPPED;
                nv_save();
                printf("[P1R] STOP received -> STOPPED (power cycle to re-arm)\n");
            } else {
                /* Odd command = START */
                if (g_sm_state == SM_POWER_ON) {
                    g_sm_state = SM_STARTED;
                    nv_save();
                    printf("[P1R] START received -> STARTED\n");
                }
            }
        }
    }
}

/* LED: POWER-ON off, STARTED on, STOPPED slow blink, programmed = 2 quick blinks */
static bool led_pattern(TickType_t now) {
    uint32_t now_ms = (uint32_t)now * portTICK_PERIOD_MS;
    if (prog_blink_active) {
        uint32_t t = now_ms - (uint32_t)prog_blink_start * portTICK_PERIOD_MS;
        if (t < 100) return true;
        if (t < 600) return false;
        if (t < 700) return true;
        if (t < 1200) return false;
        prog_blink_active = false;
    }
    switch (g_sm_state) {
        case SM_STARTED: return true;
        case SM_STOPPED: return (now_ms / 500) & 1;
        default:         return false;
    }
}

/* Competition start/stop is controlled exclusively by the P1R RC-5 commands.
 * Do not let a local button or test-mode input modify the persistent
 * competition state: that could bypass the referee's start command. */

/* ---- P1R RC-5 decoder (14 bits on the wire: S1 S2 T A4..A0 C5..C0) ---- */
#define RC5_SHORT_MIN   350u
#define RC5_SHORT_MAX   1400u
#define RC5_LONG_MAX    2500u
#define RC5_BITS 14

/*
 * The first captured edge must be the falling mid-bit edge of S1.  Each
 * subsequent edge is separated by one or two nominal 889 us half-bits.
 * Reject implausibly short intervals instead of silently skipping them:
 * skipping an isolated glitch changes the Manchester phase and can turn a
 * corrupted frame into a different command.
 */
static bool rc5_decode(const uint32_t *t, int n, uint8_t *addr,
                       uint8_t *cmd, uint8_t *toggle) {
    if (n < 12 || n > 30 || t == NULL || addr == NULL || cmd == NULL || toggle == NULL)
        return false;

    uint8_t h[2 * RC5_BITS];
    int hn = 0;
    h[hn++] = 1;                      /* S1 starts with idle-high */
    uint8_t level = 0;                /* first falling edge starts low half */

    for (int i = 0; i < n - 1; i++) {
        uint32_t d = t[i + 1] - t[i];
        int halves;
        if (d < RC5_SHORT_MIN) return false;
        if (d <= RC5_SHORT_MAX)      halves = 1;
        else if (d <= RC5_LONG_MAX)  halves = 2;
        else return false;

        if (hn + halves > (int)(sizeof h)) return false;
        for (int k = 0; k < halves; k++) h[hn++] = level;
        level ^= 1;
    }

    /* The final transition can be absent when the last data half ends high. */
    if (hn == 2 * RC5_BITS - 1 && level == 1) h[hn++] = 1;
    if (hn != 2 * RC5_BITS) return false;

    uint8_t bits[RC5_BITS];
    for (int k = 0; k < RC5_BITS; k++) {
        if (h[2 * k] == 1 && h[2 * k + 1] == 0)      bits[k] = 1;
        else if (h[2 * k] == 0 && h[2 * k + 1] == 1) bits[k] = 0;
        else return false;             /* invalid Manchester pair */
    }
    if (bits[0] != 1 || bits[1] != 1) return false;

    uint8_t a = 0, c = 0;
    *toggle = bits[2];
    for (int k = 3; k <= 7; k++)  a = (uint8_t)((a << 1) | bits[k]);
    for (int k = 8; k <= 13; k++) c = (uint8_t)((c << 1) | bits[k]);

    *addr = a;
    *cmd = c;
    return true;
}

/* ---- NEC decoder (test mode) ---- */
static bool nec_decode(const uint32_t *t, int n, uint8_t *addr, uint8_t *cmd, bool *repeat) {
    if (n < 3) return false;
    uint32_t mark  = t[1] - t[0];
    uint32_t space = t[2] - t[1];
    if (mark < 7500 || mark > 10500) return false;

    if (space >= 1500 && space <= 3000) { *repeat = true; return true; }   /* repeat code */
    if (space < 3500 || space > 5500 || n < 67) return false;

    uint32_t v = 0;
    for (int k = 0; k < 32; k++) {
        uint32_t m = t[3 + 2 * k] - t[2 + 2 * k];
        uint32_t sp = t[4 + 2 * k] - t[3 + 2 * k];
        if (m < 300 || m > 1000) return false;
        if (sp < 300 || sp > 2200) return false;
        if (sp > 1100) v |= (1u << k);
    }
    uint8_t c  = (v >> 16) & 0xFF;
    uint8_t ci = (v >> 24) & 0xFF;
    if ((uint8_t)(c ^ ci) != 0xFF) return false;
    *addr = v & 0xFF;
    *cmd = c;
    *repeat = false;
    return true;
}

/* Test-mode requests, consumed by the strategy task. */
static volatile bool g_test_run_enabled = false;
static volatile bool g_test_reverse_request = false;
static volatile bool g_test_edge_escape_request = false;
static uint32_t g_test_last_toggle_ms = 0;
static bool g_test_toggle_seen = false;

/* NEC command handler, used only while the mode switch selects TEST mode.
 * Runs in the IR task, so the blocking calibration capture is safe here. */
void ir_nec_command_handler(uint8_t addr, uint8_t cmd, bool repeat) {
    if (repeat) {
        return; /* NEC hold repeats must not retrigger/toggle actions. */
    }
    printf("[IR/NEC] addr=0x%02X cmd=0x%02X\n", addr, cmd);
    if (competition_mode()) return;

    uint32_t now_ms = to_ms_since_boot(get_absolute_time());
    switch (cmd) {
        case TEST_RUN_TOGGLE_CMD:
            if (!g_test_toggle_seen || (uint32_t)(now_ms - g_test_last_toggle_ms) >= TEST_TOGGLE_GUARD_MS) {
                g_test_toggle_seen = true;
                g_test_last_toggle_ms = now_ms;
                g_test_run_enabled = !g_test_run_enabled;
                printf("[TEST] line strategy %s\n", g_test_run_enabled ? "ON" : "OFF");
            } else {
                printf("[TEST] 0x43 ignored (toggle guard: 1 second)\n");
            }
            break;
        case TEST_REVERSE_CMD:
            g_test_reverse_request = true;
            printf("[TEST] reverse manoeuvre requested\n");
            break;
        case TEST_EDGE_ESCAPE_CMD:
            g_test_edge_escape_request = true;
            printf("[TEST] straight reverse (edge-escape) requested\n");
            break;
        case TEST_QRE_DOHYO_CMD:
            qre_nv_calibrate_capture(false);
            break;
        case TEST_QRE_LINE_CMD:
            qre_nv_calibrate_capture(true);
            break;
        case TEST_QRE_RESET_CMD:
            qre_nv_reset_defaults();
            break;
        default:
            break;
    }
}

void vIRCommandTask(void *pvParameters) {
    (void)pvParameters;

    gpio_set_irq_enabled_with_callback(IR_RX_PIN, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true, gpio_isr);

    bool last_comp = competition_mode();
    printf("[MODE] %s\n", last_comp ? "COMPETITION (P1R start module)" : "TEST MODE (NEC commands)");

    TickType_t period = pdMS_TO_TICKS(IR_POLL_MS);
    if (period == 0) period = 1;

    while (1) {
        vTaskDelay(period);

        bool comp = competition_mode();
        if (comp != last_comp) {
            last_comp = comp;
            uint32_t irq = save_and_disable_interrupts();
            ir_edge_cnt = 0;
            ir_edge_overflow = false;
            restore_interrupts(irq);
            printf("[MODE] %s\n", comp ? "COMPETITION (P1R start module)" : "TEST MODE (NEC commands)");
        }

        if (comp) gpio_put(LED_PIN, led_pattern(xTaskGetTickCount()));
        else      gpio_put(LED_PIN, g_test_run_enabled);   /* test mode: LED on = strategy running */

        /* Recheck the silence gap with interrupts disabled, so an edge
         * arriving between the initial test and the copy cannot split a frame. */
        uint32_t t[IR_MAX_EDGES];
        int n;
        bool first_fall, overflow;
        uint32_t irq = save_and_disable_interrupts();
        if (ir_edge_cnt == 0 || (time_us_32() - ir_last_edge_us) <= IR_FRAME_GAP_US) {
            restore_interrupts(irq);
            continue;
        }
        n = ir_edge_cnt;
        first_fall = ir_first_is_fall;
        overflow = ir_edge_overflow;
        for (int i = 0; i < n; i++) t[i] = ir_edge_t[i];
        ir_edge_cnt = 0;
        ir_edge_overflow = false;
        restore_interrupts(irq);

        if (comp) {
            uint8_t addr, cmd, toggle;
            if (overflow) {
                printf("[IR/RC5] Rejected: edge buffer overflow (%d captured)\n", n);
                continue;
            }
            if (!first_fall) {
                printf("[IR/RC5] Rejected: first edge was not falling (%d edges)\n", n);
                continue;
            }
            if (rc5_decode(t, n, &addr, &cmd, &toggle)) {
                printf("[IR/RC5] addr=0x%02X cmd=0x%02X toggle=%u\n",
                       addr, cmd, toggle);
                p1r_handle(addr, cmd);
            } else {
                printf("[IR/RC5] Rejected: %d edges. Intervals: ", n);
                for (int i = 0; i < n - 1 && i < 28; i++)
                    printf("%lu ", (unsigned long)(t[i + 1] - t[i]));
                printf("\n");
            }
        } else {
            if (overflow) {
                printf("[IR/NEC] Rejected: edge buffer overflow (%d captured)\n", n);
                continue;
            }
            if (!first_fall || n < 3) continue;
            uint8_t addr, cmd;
            bool rep;
            if (nec_decode(t, n, &addr, &cmd, &rep)) ir_nec_command_handler(addr, cmd, rep);
            else printf("[IR/NEC] unrecognized frame (%d edges)\n", n);
        }
    }
}

/* ---------------------------------------------------------------- */
/* Motor control                                                    */
/* ---------------------------------------------------------------- */

typedef struct {
    uint pin;
} motor_pwm_t;

static motor_pwm_t motor_right_fwd, motor_right_rev;
static motor_pwm_t motor_left_fwd, motor_left_rev;
static uint16_t motor_pwm_top;
static uint32_t motor_pwm_period_counts;

static void motor_pwm_configure_slice(uint slice) {
    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_clkdiv(&cfg, 1.0f);
    pwm_config_set_wrap(&cfg, motor_pwm_top);
    pwm_init(slice, &cfg, false);
    pwm_set_enabled(slice, true);
}

static void motor_pwm_init(void) {
    const uint32_t sys_hz = clock_get_hz(clk_sys);
    motor_pwm_period_counts = (sys_hz + MOTOR_PWM_FREQUENCY_HZ / 2u)
                            / MOTOR_PWM_FREQUENCY_HZ;
    if (motor_pwm_period_counts < 2u || motor_pwm_period_counts > 65536u) {
        panic("Motor PWM: clk_sys cannot produce requested frequency with divider 1");
    }
    motor_pwm_top = (uint16_t)(motor_pwm_period_counts - 1u);

    const uint pins[] = {
        MOTOR_RIGHT_FWD_PIN, MOTOR_RIGHT_REV_PIN,
        MOTOR_LEFT_FWD_PIN, MOTOR_LEFT_REV_PIN
    };
    uint configured_slices[4];
    size_t configured_count = 0;

    for (size_t i = 0; i < (sizeof(pins) / sizeof(pins[0])); ++i) {
        gpio_set_function(pins[i], GPIO_FUNC_PWM);
        const uint slice = pwm_gpio_to_slice_num(pins[i]);
        bool already_configured = false;
        for (size_t j = 0; j < configured_count; ++j) {
            if (configured_slices[j] == slice) {
                already_configured = true;
                break;
            }
        }
        if (!already_configured) {
            configured_slices[configured_count++] = slice;
            motor_pwm_configure_slice(slice);
        }
    }

    motor_right_fwd.pin = MOTOR_RIGHT_FWD_PIN;
    motor_right_rev.pin = MOTOR_RIGHT_REV_PIN;
    motor_left_fwd.pin = MOTOR_LEFT_FWD_PIN;
    motor_left_rev.pin = MOTOR_LEFT_REV_PIN;

    for (size_t i = 0; i < (sizeof(pins) / sizeof(pins[0])); ++i) {
        pwm_set_gpio_level(pins[i], 0);
    }
}

static uint16_t motor_duty_to_level(uint16_t duty_permille) {
    if (duty_permille == 0u) return 0u;
    if (duty_permille < MOTOR_MIN_DUTY_PERMILLE)
        duty_permille = MOTOR_MIN_DUTY_PERMILLE;
    if (duty_permille > MOTOR_FULL_DUTY_PERMILLE)
        duty_permille = MOTOR_FULL_DUTY_PERMILLE;

    /* TOP+1 gives a true 100% duty (compare never matches the counter). */
    return (uint16_t)(((uint32_t)duty_permille * motor_pwm_period_counts + 500u)
                      / MOTOR_FULL_DUTY_PERMILLE);
}

static void motor_pwm_write(motor_pwm_t output, uint16_t duty_permille) {
    pwm_set_gpio_level(output.pin, motor_duty_to_level(duty_permille));
}

static void motors_init(void) {
    motor_pwm_init();
}

static void motor_set_one(motor_pwm_t forward, motor_pwm_t reverse,
                          int16_t command) {
    uint16_t magnitude = (uint16_t)(command < 0 ? -command : command);
    if (command >= 0) {
        motor_pwm_write(reverse, 0u);
        motor_pwm_write(forward, magnitude);
    } else {
        motor_pwm_write(forward, 0u);
        motor_pwm_write(reverse, magnitude);
    }
}

/* Positive = forward, negative = backward, zero = coast. */
static void motors_set(int16_t left, int16_t right) {
    motor_set_one(motor_left_fwd, motor_left_rev, left);
    motor_set_one(motor_right_fwd, motor_right_rev, right);
}

static void motors_stop(void) {
    motors_set(0, 0);
}

/* Drive BOTH wheels backward using their explicitly assigned reverse pins:
 * left = GPIO22, right = GPIO19. */
static void motors_reverse_both(uint16_t duty_permille) {
    motor_pwm_write(motor_left_fwd, 0u);
    motor_pwm_write(motor_right_fwd, 0u);
    motor_pwm_write(motor_left_rev, duty_permille);
    motor_pwm_write(motor_right_rev, duty_permille);
}

/* ---------------------------------------------------------------- */
/* Line-sensor-only strategy                                        */
/* ---------------------------------------------------------------- */

typedef enum {
    DAUF_IDLE = 0,
    DAUF_START_REVERSE,
    DAUF_FORWARD,
    DAUF_BACKOFF,
    DAUF_TURN
} dauf_state_t;

void vStrategyTask(void *pvParameters) {
    (void)pvParameters;
    motors_init();
    motors_stop();
    printf("[DAUF] PWM: clk_sys=%lu Hz, frequency=%lu Hz, TOP=%u, min duty=%u/1000\n",
           (unsigned long)clock_get_hz(clk_sys),
           (unsigned long)(clock_get_hz(clk_sys) / motor_pwm_period_counts),
           (unsigned)motor_pwm_top, (unsigned)MOTOR_MIN_DUTY_PERMILLE);

    dauf_state_t state = DAUF_IDLE;
    uint32_t state_start_ms = 0;
    uint32_t forward_start_ms = 0;
    int turn_dir = 1;               /* +1 = clockwise, -1 = counter-clockwise */
    uint32_t turn_ms = DAUF_TURN_SIDE_MS;

    enum { MANUAL_NONE = 0, MANUAL_REVERSE, MANUAL_ESCAPE };
    int manual_action = MANUAL_NONE;
    uint32_t manual_start_ms = 0;

    for (;;) {
        const uint32_t now_ms = to_ms_since_boot(get_absolute_time());
        const bool comp_mode   = competition_mode();
        const bool comp_active = comp_mode && (start_module_state() == SM_STARTED);
        const bool test_active = !comp_mode && g_test_run_enabled;
        const bool run         = comp_active || test_active;

        /* ---- manual test pulses (TEST mode only) ---- */
        if (!comp_mode) {
            if (g_test_reverse_request) {
                g_test_reverse_request = false;
                manual_action = MANUAL_REVERSE;
                manual_start_ms = now_ms;
                printf("[TEST] 0x09 reverse started\n");
            } else if (g_test_edge_escape_request) {
                g_test_edge_escape_request = false;
                manual_action = MANUAL_ESCAPE;
                manual_start_ms = now_ms;
                printf("[TEST] 0x40 reverse started\n");
            }
        } else {
            g_test_reverse_request = false;
            g_test_edge_escape_request = false;
            manual_action = MANUAL_NONE;
        }

        if (manual_action != MANUAL_NONE) {
            uint32_t dur = (manual_action == MANUAL_REVERSE) ? START_REVERSE_MS
                                                             : EDGE_ESCAPE_MS;
            if ((uint32_t)(now_ms - manual_start_ms) < dur) {
                motors_reverse_both((uint16_t)MOTOR_REVERSE_PWM);
                vTaskDelay(pdMS_TO_TICKS(2));
                continue;
            }
            manual_action = MANUAL_NONE;
            motors_stop();
            printf("[TEST] manual manoeuvre complete\n");
        }

        /* ---- not running: stay stopped and re-arm the state machine ---- */
        if (!run) {
            if (state != DAUF_IDLE) {
                state = DAUF_IDLE;
                printf("[DAUF] stopped\n");
            }
            motors_stop();
            vTaskDelay(pdMS_TO_TICKS(2));
            continue;
        }

        /* ---- just started ---- */
        if (state == DAUF_IDLE) {
            state_start_ms = now_ms;
            forward_start_ms = now_ms;
#if DAUF_USE_START_REVERSE
            state = DAUF_START_REVERSE;
            printf("[DAUF] started: initial reverse\n");
#else
            state = DAUF_FORWARD;
            printf("[DAUF] started: forward\n");
#endif
        }

        const uint32_t qre_left  = qre_line_sensor_us[0];
        const uint32_t qre_right = qre_line_sensor_us[1];
        const bool left_edge  = qre_edge_detected_i(0, qre_left);
        const bool right_edge = qre_edge_detected_i(1, qre_right);
        const uint32_t in_state_ms = (uint32_t)(now_ms - state_start_ms);

        switch (state) {
        case DAUF_START_REVERSE:
            if (in_state_ms < START_REVERSE_MS) {
                motors_reverse_both((uint16_t)MOTOR_REVERSE_PWM);
            } else {
                state = DAUF_FORWARD;
                state_start_ms = forward_start_ms = now_ms;
            }
            break;

        case DAUF_FORWARD:
            if (left_edge || right_edge) {
                if (left_edge && right_edge) {
                    turn_dir = 1;
                    turn_ms = DAUF_TURN_BOTH_MS;
                } else if (left_edge) {
                    turn_dir = 1;               /* line on left -> turn right */
                    turn_ms = DAUF_TURN_SIDE_MS;
                } else {
                    turn_dir = -1;              /* line on right -> turn left */
                    turn_ms = DAUF_TURN_SIDE_MS;
                }
                printf("[DAUF] line L=%lu R=%lu us -> backoff, turn %s\n",
                       (unsigned long)qre_left, (unsigned long)qre_right,
                       turn_dir > 0 ? "right" : "left");
                state = DAUF_BACKOFF;
                state_start_ms = now_ms;
                motors_set(-DAUF_BACKOFF_PWM, -DAUF_BACKOFF_PWM);
            } else {
                /* sweep: alternate gentle arcs left / right */
                bool arc_left = (((now_ms - forward_start_ms) / DAUF_ARC_PERIOD_MS) & 1u) == 0u;
                if (arc_left) motors_set(DAUF_ARC_INNER_PWM, DAUF_FWD_PWM);
                else          motors_set(DAUF_FWD_PWM, DAUF_ARC_INNER_PWM);
            }
            break;

        case DAUF_BACKOFF: {
            bool clear = qre_edge_cleared_i(0, qre_left) && qre_edge_cleared_i(1, qre_right);
            if ((in_state_ms >= DAUF_BACKOFF_MIN_MS && clear) ||
                in_state_ms >= DAUF_BACKOFF_MAX_MS) {
                state = DAUF_TURN;
                state_start_ms = now_ms;
            } else {
                motors_set(-DAUF_BACKOFF_PWM, -DAUF_BACKOFF_PWM);
            }
            break;
        }

        case DAUF_TURN:
            if (in_state_ms < turn_ms) {
                if (turn_dir > 0) motors_set(DAUF_TURN_PWM, -DAUF_TURN_PWM);   /* clockwise */
                else              motors_set(-DAUF_TURN_PWM, DAUF_TURN_PWM);   /* counter-clockwise */
            } else {
                state = DAUF_FORWARD;
                state_start_ms = forward_start_ms = now_ms;
            }
            break;

        default:
            state = DAUF_IDLE;
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

/* ---------------------------------------------------------------- */
/* FreeRTOS hooks                                                   */
/* ---------------------------------------------------------------- */

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) {
    (void)xTask;
    printf("CRITICAL: Stack overflow in task: %s\n", pcTaskName);
    portDISABLE_INTERRUPTS();
    while (1);
}

void vApplicationMallocFailedHook(void) {
    printf("CRITICAL: FreeRTOS malloc failed!\n");
    portDISABLE_INTERRUPTS();
    while (1);
}

/* ---------------------------------------------------------------- */
/* Main                                                             */
/* ---------------------------------------------------------------- */

static void motor_pins_safe_state(void) {
    const uint pins[] = {
        MOTOR_LEFT_FWD_PIN, MOTOR_LEFT_REV_PIN,
        MOTOR_RIGHT_FWD_PIN, MOTOR_RIGHT_REV_PIN
    };
    for (size_t i = 0; i < sizeof(pins) / sizeof(pins[0]); ++i) {
        gpio_init(pins[i]);
        gpio_set_dir(pins[i], GPIO_OUT);
        gpio_put(pins[i], 0);
    }
}

int main() {
    motor_pins_safe_state();
    stdio_init_all();
    sleep_ms(2000);

    i2c_init(I2C_PORT, I2C_BAUD_RATE);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 0);

    /* Mode switch: HIGH = competition, LOW/open = test */
    gpio_init(MODE_SWITCH_PIN);
    gpio_set_dir(MODE_SWITCH_PIN, GPIO_IN);
    gpio_pull_down(MODE_SWITCH_PIN);

    /* IR receiver output (idles high, pulses low) */
    gpio_init(IR_RX_PIN);
    gpio_set_dir(IR_RX_PIN, GPIO_IN);
    gpio_pull_up(IR_RX_PIN);

    i2c_mutex = xSemaphoreCreateMutex();
    if (i2c_mutex == NULL) {
        printf("[NV] ERROR: could not create I2C mutex; refusing to start tasks\n");
        while (1) tight_loop_contents();
    }

    /* Restore the P1R state and the QRE thresholds before any task can use them. */
    nv_load();
    qre_nv_load();

    printf("\n--- Mini-sumo line-sensor strategy ---\n");

    xTaskCreate(vQRELineSensorTask, "QRETask",  1024, NULL, 3, NULL);
    xTaskCreate(vIRCommandTask,     "IRTask",   1536, NULL, 4, NULL);
    xTaskCreate(vStrategyTask,      "Strategy", 1024, NULL, 5, NULL);
    vTaskStartScheduler();

    while (1) tight_loop_contents();
}