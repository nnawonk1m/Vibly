/* ===========/Users/nawonkim/Downloads/vibly_localised_haptics/audio_model.h=================================================================
 * vibly_localised_haptics.ino
 * XIAO ESP32-S3 · 2x INMP441 · 2x ERM · BLE
 * ----------------------------------------------------------------------------
 * Merge of xiao_dual_inmp441_inference.ino and haptic_pattern_engine.ino.
 *
 * SIGNAL PATH — three stages, strictly in this order:
 *
 *   STAGE 1  CLASSIFY   Each microphone is classified on its own, untouched
 *                       audio. Nothing in this stage knows or cares where the
 *                       sound is. This is deliberate: the model was trained on
 *                       single-mic recordings, so any channel arithmetic here
 *                       (averaging, panning, gain matching) would move the MFE
 *                       features away from the training distribution.
 *
 *   STAGE 2  LOCALISE   Only after am_predict() has returned and the vote
 *                       smoother has produced a stable class do we look at the
 *                       interaural level difference (ILD) to decide WHERE the
 *                       sound is. This stage reads the ring buffers but never
 *                       writes them, so it cannot perturb Stage 1.
 *
 *   STAGE 3  RENDER     Class -> rhythm, ILD -> left/right duty ratio,
 *                       absolute level -> overall duty.
 *
 * WHY THE STAGES ARE SPLIT
 *   Localisation and classification want opposite things from the audio.
 *   Classification wants one clean channel that resembles the training data.
 *   Localisation wants both channels, unmodified, compared against each other.
 *   Running them in sequence rather than in parallel means neither compromises
 *   for the other, and it costs nothing: the ILD is computed from RMS values,
 *   which is ~4000 multiply-accumulates against the ~1.2 M of the CNN.
 *
 * ---------------------------------------------------------------------------
 * WIRING
 * ---------------------------------------------------------------------------
 *      signal        MIC A (left)      MIC B (right)     XIAO ESP32-S3
 *      ---------------------------------------------------------------
 *      VDD           VDD               VDD               3V3
 *      GND           GND               GND               GND
 *      L/R           GND   <-- low     3V3  <-- high     (not a XIAO pin)
 *      SCK  (BCLK)   SCK               SCK               D2 / GPIO3
 *      WS   (LRCLK)  WS                WS                D3 / GPIO4
 *      SD   (DOUT)   SD ---------------SD -------------- D4 / GPIO5
 *
 *      LEFT  ERM  -> D10   via MOSFET/ULN2003A + flyback diode
 *      RIGHT ERM  -> D9    via MOSFET/ULN2003A + flyback diode
 *
 *   MIC A must sit on the user's LEFT and MIC B on the user's RIGHT, or the
 *   bearing is mirrored. If they are swapped, set SWAP_CHANNELS to 1 — that
 *   flips both the classifier's channel identity and the bearing together.
 *
 * ---------------------------------------------------------------------------
 * FIRST-RUN CALIBRATION  (do this before trusting any bearing reading)
 * ---------------------------------------------------------------------------
 *   1. Mount both mics in their final positions on the headband.
 *   2. Put a steady sound source (speech, a tone on a phone) directly in
 *      front of the wearer, ~1 m away, equidistant from both mics.
 *   3. Press 'k'. It averages the raw ILD over 2 s and prints a number.
 *   4. Paste that number into MIC_BALANCE_DB below and re-flash.
 *
 *   Skipping this is the single most likely reason a bearing sits off-centre.
 *   Two INMP441s have a sensitivity tolerance, and cable length, mounting
 *   compliance and how close each sits to the skull all add more.
 *
 * ---------------------------------------------------------------------------
 * ARDUINO IDE
 * ---------------------------------------------------------------------------
 *   Board XIAO_ESP32S3 · PSRAM = OPI PSRAM · CPU = 240 MHz
 *   USB CDC On Boot = Enabled · Partition Scheme = 8M with spiffs
 *   ESP32 core 3.x (needed for ledcAttach) · Serial Monitor @ 115200
 *   Serial Monitor line ending = "No line ending"
 * ==========================================================================*/

#include <driver/i2s.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

/* esp_ble_gatts_cb_param_t lives here; BLEDevice.h does not pull it in.
 * __has_include so a core that moves the header costs one skipped
 * optimisation rather than the whole build. */
#if __has_include(<esp_gatts_api.h>)
  #include <esp_gatts_api.h>
  #define HAVE_GATTS_PARAM 1
#else
  #define HAVE_GATTS_PARAM 0
#endif

#include "audio_model.h"
/* SoundClass, Segment, Pattern, HapticState. These MUST live in a header
 * rather than in this file: the Arduino IDE injects generated prototypes
 * above the sketch body, so a type declared in the .ino is invisible to the
 * prototype of any function that takes it. See vibly_types.h. */
#include "vibly_types.h"

/* ===========================================================================
 * SECTION 1 — AUDIO FRONT END CONFIG   (unchanged from the inference sketch)
 * ========================================================================= */
#define I2S_SCK          3      /* D2 - BCLK, shared by both mics */
#define I2S_WS           4      /* D3 - WS,   shared by both mics */
#define I2S_SD           5      /* D4 - SD,   both mics tied together */
#define I2S_PORT         I2S_NUM_0

#define DMA_FRAMES       256
#define DMA_BUF_COUNT    8

/* MUST match the value used to record the training data. */
#define SHIFT_BITS       14

#define ENABLE_DC_BLOCK  1
#define DC_BLOCK_R       0.995f

/* Set to 1 if mic A is physically on the right. Flips classifier channel
 * identity AND bearing sign together, so they can never disagree. */
#define SWAP_CHANNELS    0

#define MODE_BOTH   0   /* classify BOTH mics, fuse the results  (default)   */
#define MODE_MIX    1   /* average the two mics, one inference               */
#define MODE_BEST   2   /* per window, use whichever mic is louder           */
#define MODE_LEFT   3   /* mic A only  (debug / fallback)                    */
#define MODE_RIGHT  4   /* mic B only  (debug / fallback)                    */

/* MODE_MIX is a poor choice here for two separate reasons: it comb-filters
 * two widely spaced mics into features the model never saw, and it discards
 * the very level difference Stage 2 needs. Localisation still works in
 * MODE_MIX because Stage 2 reads the rings directly, but the classification
 * feeding it will be the weakest of the five modes. */
#define FUSION           MODE_BOTH

#define HOP_SAMPLES      4000   /* 250 ms between inferences                 */
#define CONF_THRESHOLD   0.70f
#define SMOOTH_M         5
#define SMOOTH_N         3
#define PRINT_EVERY      1
#define CHANNEL_REPORT   1
#define DEAD_MIC_DB      25.0f

/* ===========================================================================
 * SECTION 2 — LOCALISATION CONFIG   (new)
 * ========================================================================= */

/* Bearing is measured over the most RECENT slice of audio, not the whole 1 s
 * classifier window. The class is a property of the last second; the bearing
 * should be a property of right now, or it lags behind a source that moves. */
#define LOC_WIN_SAMPLES   4000    /* 250 ms */

/* Fixed offset between the two mics with a centred source, in dB.
 * Measured value = (mic B dBFS) - (mic A dBFS). Press 'k' to measure. */
#define MIC_BALANCE_DB    0.0f

/* ILD below this is treated as dead centre. Room reflections, mic tolerance
 * and the DC blocker all wobble by around a dB, and without a deadband the
 * motors would trade emphasis back and forth on a stationary source. */
#define ILD_DEADBAND_DB   1.5f

/* ILD at which bearing saturates to hard left/right. With mics either side of
 * a head, head shadow gives roughly 6-15 dB depending on frequency; 9 dB is a
 * reasonable starting point. Lower it if bearings feel sluggish, raise it if
 * they slam to the extremes on slightly off-centre sources. */
#define ILD_FULL_DB       9.0f

/* Below this level on the louder mic the ILD is just noise ratio, so the last
 * good bearing is held instead of being replaced with garbage. */
#define LOC_GATE_DBFS    -55.0f

/* Exponential smoothing on bearing. 0 = frozen, 1 = no smoothing. */
#define BEARING_ALPHA     0.40f

/* Level of the louder mic mapped to proximity 0..1. Calibrate against the
 * distances that actually matter for the wearer. */
#define PROX_DB_FAR      -50.0f
#define PROX_DB_NEAR     -20.0f

/* How long the pattern keeps running after the classifier stops reporting a
 * stable class. Stops a pattern being chopped off mid-phrase by one bad hop. */
#define HAPTIC_RELEASE_MS 1200

#define CAL_HOPS          8       /* 8 x 250 ms = 2 s of averaging for 'k' */

/* ===========================================================================
 * SECTION 3 — HAPTIC CONFIG   (unchanged from the pattern engine)
 * ========================================================================= */
#define RIGHT_MOTOR_PIN  D9
#define LEFT_MOTOR_PIN   D10

#define PWM_FREQ_HZ   20000
#define PWM_RES_BITS  8
#define PWM_MAX       255

#define DUTY_FLOOR   70
#define DUTY_CEIL    130

#define KICK_MS       25
#define KICK_GAIN     1.6f

#define PAN_DEPTH     0.85f

/* The haptic timeline runs in its own task so pattern timing survives the
 * ~100 ms the CNN spends holding the Arduino loop task. 5 ms is well under
 * the ERM's ~30-50 ms perceptual rise time, so the edges land where the
 * pattern table says they do. */
#define HAPTIC_TICK_MS    5

#define SERVICE_UUID        "780bf242-ff64-47f4-9627-cc76108e2a27"
#define CHARACTERISTIC_UUID "38ff9161-1378-4414-a3c2-cb5ffc17ff92"

/* ---------------------------------------------------------------------------
 * BLE LOG STREAMING  (from ble_adv.ino)
 *
 * ble_adv.ino notified a hardcoded string to prove the transport worked. This
 * sends the real status line instead, so the Serial Monitor and the phone show
 * the same text and the device can be tested off the cable.
 *
 * THREE THINGS THAT SKETCH GOT AWAY WITH AND THIS ONE CANNOT:
 *
 *  - A notification carries only (MTU - 3) bytes. The default MTU is 23, so
 *    twenty characters arrive and the rest is discarded with no error at
 *    either end. The status line here is ~150 characters. setMTU() is a
 *    REQUEST; the central decides, so the real value is read back and the
 *    line is split to fit.
 *
 *  - setMTU() must be called AFTER BLEDevice::init(), because it configures
 *    the running GATT stack. Called first it is quietly ignored.
 *
 *  - One notification goes out per connection interval. A central left to
 *    itself often picks 100-500 ms, so eight packets is four seconds. The
 *    device now asks for the fastest interval Apple permits, AND queues
 *    lines to a separate task so a slow radio can never stall inference.
 * ------------------------------------------------------------------------ */
#define BLE_REQUEST_MTU   247
/* Connection interval, in 1.25 ms units.
 *
 * THESE MUST OBEY APPLE'S RULES, not merely the Bluetooth spec. Chrome and
 * Safari on macOS and iOS both go through CoreBluetooth, and Apple does not
 * simply ignore a non-compliant connection-parameter request — it drops the
 * link a few seconds later:
 *
 *     interval min        >= 15 ms   (12 units)
 *     interval max        >= min + 15 ms
 *     slave latency       <= 30
 *     supervision timeout  2 s to 6 s
 *     interval max x (latency + 1) <= 2 s
 *
 * Requesting 7.5 ms is legal per the spec and is what most ESP32 examples
 * show, and it is what was killing connections about three seconds in.
 * Complying costs almost nothing: 15 ms still allows ~66 notifications per
 * second, and four log lines a second needs about eight. */
#define BLE_CONN_MIN      12     /* x1.25 ms = 15 ms - Apple's floor */
#define BLE_CONN_MAX      24     /* x1.25 ms = 30 ms = min + 15 ms   */
#define BLE_CONN_LATENCY  0
#define BLE_CONN_TIMEOUT  400    /* x10 ms = 4 s, inside Apple's 2-6 s */
#define BLE_QUEUE_LEN     8
#define BLE_LINE_MAX      256   /* the line grew: haptic duties + OVD state */

/* 1 = stream status lines. 0 = events only. */
#define BLE_SEND_STATUS   1

/* Send only every Nth status line. 1 = all four a second, 4 = one a second.
 *
 * A 250-character line is 2 packets at a negotiated MTU of 247, but THIRTEEN
 * at the 23-byte default. Four lines a second then means ~52 notifications a
 * second, which is enough to fill the controller's transmit queue and get the
 * link dropped. Sending one line a second removes radio load as a variable
 * entirely; events are still sent the instant they happen, so nothing
 * important is delayed. Put this back to 1 once the link is proven stable. */
#define BLE_STATUS_EVERY_N 1

/* ===========================================================================
 * SECTION 3c — OWN-VOICE DETECTION  (XIAO nRF52840 Sense over UART)
 * ---------------------------------------------------------------------------
 * WHAT IT IS FOR
 *   Speech is the class the wearer triggers most often, and most of it is
 *   their own. Buzzing someone every time they open their mouth trains them
 *   to ignore the device, which defeats the whole point of an alert. The
 *   nRF52840's IMU picks up bone-conducted vibration from the wearer's own
 *   voice, which an external microphone cannot distinguish from anyone
 *   else's, and sends a 1 or 0 down the UART.
 *
 * WHAT IT IS *NOT* FOR
 *   It gates HAPTICS ONLY, and only the speech pattern. A car horn is still
 *   worth feeling while you are mid-sentence, and arguably more so. It never
 *   touches the classifier: the model keeps seeing untouched audio, exactly
 *   as with localisation, so an OVD fault can never corrupt classification.
 *
 * THE FEEDBACK LOOP THIS CLOSES
 *   An ERM spins at 100-250 Hz, right inside the speech band, so the IMU
 *   cannot tell the wearer's voice from the device's own motors. Left
 *   unhandled, a haptic pulse reads as own-voice, which suppresses the next
 *   pulse, which un-suppresses it. HAPTIC_BLANK_PIN is driven HIGH whenever
 *   either motor is running, and the OVD sketch freezes its decision while it
 *   is high. That wire is what stops the device detecting itself.
 *
 * WIRING  (both boards are 3.3 V logic - no level shifting needed)
 *   nRF52840 D6 (TX)         ->  ESP32-S3 D7 / GPIO44  (OVD_RX_PIN)
 *   ESP32-S3 D8 / GPIO7      ->  nRF52840 HAPTIC_BLANK_PIN
 *   GND                      <-> GND        <-- REQUIRED, not optional
 *
 *   GPIO44/43 are UART0's default pins on this board. They are free here
 *   because USB CDC On Boot puts the Serial Monitor on the USB peripheral
 *   instead, and these pins are the ones already proven working in
 *   ovd_uart_receiver_esp32s3.ino.
 *
 *   In ovd_uart_nrf52840.ino, set:
 *       const int HAPTIC_BLANK_PIN = D3;   // or whichever pin you wire to
 * ========================================================================= */
#define OVD_ENABLE        1
#define OVD_RX_PIN        44     /* D7 - receives from nRF52840 D6 (TX) */
#define OVD_TX_PIN        43     /* D6 - unused one-way, but begin() needs it */
#define OVD_BAUD          115200

/* The OVD board heartbeats every 50 ms. If nothing arrives for this long the
 * link is treated as DOWN and suppression stops — failing open, so a pulled
 * wire costs the wearer a feature rather than silencing their alerts. */
#define OVD_TIMEOUT_MS    500

/* Hold suppression this long after speech stops, so the gaps between words
 * do not let a pattern stutter in and out mid-sentence. */
#define OVD_RELEASE_MS    350

/* Drives HIGH whenever either motor is running. -1 disables the output. */
#define HAPTIC_BLANK_PIN  7      /* D8 */

/* 1 = print a line whenever own-voice flips. The standalone receiver printed
 * on every byte, which is 20 lines a second at the nRF's 50 ms heartbeat and
 * would bury the classifier output. Only transitions carry information. */
#define OVD_VERBOSE       1

/* 1 = print every byte arriving on the OVD UART, as hex and character.
 * Noisy, but it is the difference between "no bytes" and "bytes I cannot
 * parse", which are two completely different faults. */
#define OVD_DEBUG_RAW     0

/* ===========================================================================
 * SECTION 4 — SHARED STATE
 * ========================================================================= */
static int16_t  ringL[AM_WIN], ringR[AM_WIN];     /* 32 KB each */
static volatile uint32_t ring_head = 0;
static volatile uint32_t since_hop = 0;
static volatile uint32_t overruns  = 0;

static int16_t  linbuf[AM_WIN];
static float    feat[AM_FRAMES * AM_MELS];
static float    probA[AM_CLASSES], probB[AM_CLASSES], prob[AM_CLASSES];

static int32_t  i2s_raw[DMA_FRAMES * 2];
static TaskHandle_t cap_task  = NULL;
static TaskHandle_t hap_task  = NULL;
static SemaphoreHandle_t hop_sem;
static SemaphoreHandle_t hap_mux;                 /* guards HapticState */

static int hist[SMOOTH_M];
static int hist_idx = 0, last_reported = -1;

#if SWAP_CHANNELS
  #define IDX_A 1
  #define IDX_B 0
#else
  #define IDX_A 0
  #define IDX_B 1
#endif

BLEServer         *server;
BLEService        *service;
BLECharacteristic *characteristic;
BLEAdvertising    *advertising;
static volatile bool phoneConnected = false;

/* Outgoing log lines. The inference loop drops a line here and returns at
 * once; the radio drains it in its own task. Lines are DROPPED when the queue
 * fills rather than queued forever — a log that silently falls further and
 * further behind reality is worse than one with a counted gap. */
struct BleLine { char txt[BLE_LINE_MAX]; };
static QueueHandle_t     ble_q      = NULL;
static volatile uint32_t bleDropped = 0;
static volatile uint16_t blePeerMTU = 23;
static volatile uint32_t loopPeriodMs = 0;   /* measured, not assumed */

/* ---- own-voice detection state ---- */
static volatile bool     ovdSpeaking   = false;  /* last raw bit from the nRF */
static volatile uint32_t ovdLastRxMs   = 0;      /* heartbeat watchdog        */
static volatile uint32_t ovdReleaseAt  = 0;      /* hangover expiry           */
static volatile uint32_t ovdFrames     = 0;      /* bits received, for the log */
static volatile uint32_t ovdSuppressed = 0;      /* speech alerts withheld     */
static volatile uint32_t ovdRawBytes   = 0;      /* EVERY byte seen on the wire */
static volatile uint32_t ovdBadBytes   = 0;      /* bytes that were not 0/1/CR/LF */
static volatile uint8_t  ovdLastByte   = 0;

/* AUTO  = the classifier drives the motors.
 * MANUAL = a serial key or a BLE write drives them, and Stage 3 stands down
 *          so the two sources never fight over the same motors. */
static volatile bool manualMode = false;

/* ===========================================================================
 * SECTION 5 — I2S CAPTURE   (unchanged)
 * ========================================================================= */
static bool i2s_init(void)
{
    i2s_config_t cfg = {
        .mode                 = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
        .sample_rate          = AM_SR,
        .bits_per_sample      = I2S_BITS_PER_SAMPLE_32BIT,
        .channel_format       = I2S_CHANNEL_FMT_RIGHT_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags     = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count        = DMA_BUF_COUNT,
        .dma_buf_len          = DMA_FRAMES,
        .use_apll             = true,
        .tx_desc_auto_clear   = false,
        .fixed_mclk           = 0
    };
    i2s_pin_config_t pins = {
        .bck_io_num   = I2S_SCK,
        .ws_io_num    = I2S_WS,
        .data_out_num = I2S_PIN_NO_CHANGE,
        .data_in_num  = I2S_SD
    };
    if (i2s_driver_install(I2S_PORT, &cfg, 0, NULL) != ESP_OK) return false;
    if (i2s_set_pin(I2S_PORT, &pins) != ESP_OK)                return false;
    i2s_zero_dma_buffer(I2S_PORT);

    size_t br; uint32_t t0 = millis();
    while (millis() - t0 < 200)
        i2s_read(I2S_PORT, i2s_raw, sizeof(i2s_raw), &br, portMAX_DELAY);
    return true;
}

static void capture_task(void *arg)
{
    size_t br;
    float ax = 0.0f, ay = 0.0f, bx = 0.0f, by = 0.0f;

    for (;;) {
        if (i2s_read(I2S_PORT, i2s_raw, sizeof(i2s_raw), &br, portMAX_DELAY) != ESP_OK)
            continue;
        int frames = br / (sizeof(int32_t) * 2);

        for (int i = 0; i < frames; i++) {
            int32_t a = i2s_raw[2*i + IDX_A] >> SHIFT_BITS;
            int32_t b = i2s_raw[2*i + IDX_B] >> SHIFT_BITS;
#if ENABLE_DC_BLOCK
            float xa = (float)a, ya = xa - ax + DC_BLOCK_R * ay; ax = xa; ay = ya; a = (int32_t)ya;
            float xb = (float)b, yb = xb - bx + DC_BLOCK_R * by; bx = xb; by = yb; b = (int32_t)yb;
#endif
            if (a >  32767) a =  32767;
            if (a < -32768) a = -32768;
            if (b >  32767) b =  32767;
            if (b < -32768) b = -32768;

            ringL[ring_head] = (int16_t)a;
            ringR[ring_head] = (int16_t)b;
            ring_head = (ring_head + 1) % AM_WIN;

            if (++since_hop >= HOP_SAMPLES) {
                since_hop = 0;
                if (xSemaphoreGive(hop_sem) != pdTRUE) overruns++;
            }
        }
    }
}

static void snapshot(const int16_t *ring, uint32_t head)
{
    uint32_t tail = AM_WIN - head;
    memcpy(linbuf,        ring + head, tail * sizeof(int16_t));
    memcpy(linbuf + tail, ring,        head * sizeof(int16_t));
}

static void snapshot_mix(uint32_t head)
{
    for (uint32_t i = 0; i < AM_WIN; i++) {
        uint32_t j = (head + i) % AM_WIN;
        linbuf[i] = (int16_t)(((int32_t)ringL[j] + (int32_t)ringR[j]) >> 1);
    }
}

static float ring_rms_dbfs(const int16_t *ring)
{
    double s = 0.0;
    for (uint32_t i = 0; i < AM_WIN; i += 4) {
        double v = (double)ring[i];
        s += v * v;
    }
    double r = sqrt(s / (AM_WIN / 4.0)) / 32768.0;
    return 20.0f * log10f((float)r + 1e-9f);
}

static void classify(const float *f, float *out) { am_predict(f, out); }

/* ===========================================================================
 * SECTION 6 — HAPTIC ENGINE
 * ========================================================================= */
/* SoundClass, Segment, Pattern and HapticState come from vibly_types.h */

const Segment SEG_SPEECH[] = {
  { true,   80 }, { false, 130 },
  { true,   80 }, { false, 130 },
  { true,   80 }, { false, 600 }
};

const Segment SEG_CAR_HORN[] = {
  { true, 1000 }
};

const Segment SEG_BABY_CRY[] = {
  { true,  350 }, { false, 150 },
  { true,   80 }, { false, 620 }
};

const Pattern PATTERNS[CLASS_COUNT] = {
  { nullptr,      0,                                      "none"      },
  { SEG_SPEECH,   sizeof(SEG_SPEECH)   / sizeof(Segment), "speech"    },
  { SEG_CAR_HORN, sizeof(SEG_CAR_HORN) / sizeof(Segment), "car horn"  },
  { SEG_BABY_CRY, sizeof(SEG_BABY_CRY) / sizeof(Segment), "baby cry"  }
};

HapticState hap = { CLASS_NONE, 0, 0, false, 0, 0, 0, 255, 255 };

void writeMotors(uint8_t dl, uint8_t dr) {
  if (dl != hap.liveL) { ledcWrite(LEFT_MOTOR_PIN,  dl); hap.liveL = dl; }
  if (dr != hap.liveR) { ledcWrite(RIGHT_MOTOR_PIN, dr); hap.liveR = dr; }

#if HAPTIC_BLANK_PIN >= 0
  /* Tell the OVD board the motors are live. Set from dl/dr rather than from
   * hap.liveL/R so the line is correct even on a call that changed nothing,
   * and set it HERE rather than in the pattern engine so it can never
   * disagree with what is physically on the pins. */
  digitalWrite(HAPTIC_BLANK_PIN, (dl || dr) ? HIGH : LOW);
#endif
}

/* Duties as actually written, read under the mutex. */
void hapticDuties(uint8_t &dl, uint8_t &dr) {
  xSemaphoreTake(hap_mux, portMAX_DELAY);
  dl = hap.dutyL; dr = hap.dutyR;
  xSemaphoreGive(hap_mux);
}

uint8_t kickOf(uint8_t duty) {
  if (duty == 0) return 0;
  uint16_t k = (uint16_t)(duty * KICK_GAIN);
  return (k > PWM_MAX) ? PWM_MAX : (uint8_t)k;
}

/* bearing:   -1.0 = hard left ... 0.0 = centre ... +1.0 = hard right
 * proximity:  0.0 = far ... 1.0 = very close */
void encodeDuties(float bearing, float proximity, uint8_t &outL, uint8_t &outR) {
  if (bearing   < -1.0f) bearing   = -1.0f;
  if (bearing   >  1.0f) bearing   =  1.0f;
  if (proximity <  0.0f) proximity =  0.0f;
  if (proximity >  1.0f) proximity =  1.0f;

  float base = DUTY_FLOOR + proximity * (float)(DUTY_CEIL - DUTY_FLOOR);

  float gainL = (bearing <= 0.0f) ? 1.0f : (1.0f - bearing * PAN_DEPTH);
  float gainR = (bearing >= 0.0f) ? 1.0f : (1.0f + bearing * PAN_DEPTH);

  float dl = base * gainL;
  float dr = base * gainR;

  outL = (dl < DUTY_FLOOR) ? 0 : (uint8_t)(dl > 255.0f ? 255.0f : dl);
  outR = (dr < DUTY_FLOOR) ? 0 : (uint8_t)(dr > 255.0f ? 255.0f : dr);
}

/* Unlocked internals. The public wrappers own the mutex, so hapticSetSource
 * can stop the motors without trying to take a lock it already holds. */
static void hapticStopLocked() {
  hap.cls     = CLASS_NONE;
  hap.kicking = false;
  writeMotors(0, 0);
}

static void hapticSetSourceLocked(SoundClass cls, float bearing, float proximity) {
  if (cls == CLASS_NONE || cls >= CLASS_COUNT) { hapticStopLocked(); return; }

  encodeDuties(bearing, proximity, hap.dutyL, hap.dutyR);

  /* Same class re-applied = amplitude update only. The rhythm keeps its
   * phase, so a source can pan across the head without the pattern
   * stuttering back to its first tap every 250 ms. */
  if (cls != hap.cls) {
    hap.cls       = cls;
    hap.segIdx    = 0;
    hap.segStart  = millis();
    hap.kicking   = PATTERNS[cls].seg[0].on;
    hap.kickStart = millis();
  }
}

void hapticSetSource(SoundClass cls, float bearing, float proximity) {
  xSemaphoreTake(hap_mux, portMAX_DELAY);
  hapticSetSourceLocked(cls, bearing, proximity);
  xSemaphoreGive(hap_mux);
}

void hapticStop() {
  xSemaphoreTake(hap_mux, portMAX_DELAY);
  hapticStopLocked();
  xSemaphoreGive(hap_mux);
}

bool hapticActive() {
  xSemaphoreTake(hap_mux, portMAX_DELAY);
  bool a = (hap.cls != CLASS_NONE);
  xSemaphoreGive(hap_mux);
  return a;
}

void hapticUpdate() {
  xSemaphoreTake(hap_mux, portMAX_DELAY);

  if (hap.cls == CLASS_NONE) {
#if HAPTIC_BLANK_PIN >= 0
    /* Before the first writeMotors call, so the line is never floating while
     * the OVD board is already reading it. */
    pinMode(HAPTIC_BLANK_PIN, OUTPUT);
    digitalWrite(HAPTIC_BLANK_PIN, LOW);
#endif
    writeMotors(0, 0);
    xSemaphoreGive(hap_mux);
    return;
  }

  const Pattern &p = PATTERNS[hap.cls];
  uint32_t now     = millis();
  const Segment *s = &p.seg[hap.segIdx];

  if (now - hap.segStart >= s->ms) {
    bool wasOn   = s->on;
    hap.segIdx   = (hap.segIdx + 1) % p.count;
    hap.segStart = now;
    s            = &p.seg[hap.segIdx];

    if (s->on && !wasOn) {
      hap.kicking   = true;
      hap.kickStart = now;
    }
  }

  if (!s->on) {
    hap.kicking = false;
    writeMotors(0, 0);
    xSemaphoreGive(hap_mux);
    return;
  }

  if (hap.kicking && (now - hap.kickStart) < KICK_MS) {
    writeMotors(kickOf(hap.dutyL), kickOf(hap.dutyR));
  } else {
    hap.kicking = false;
    writeMotors(hap.dutyL, hap.dutyR);
  }

  xSemaphoreGive(hap_mux);
}

static void haptic_task(void *arg)
{
  TickType_t last = xTaskGetTickCount();
  for (;;) {
    hapticUpdate();
    vTaskDelayUntil(&last, pdMS_TO_TICKS(HAPTIC_TICK_MS));
  }
}

/* ===========================================================================
 * SECTION 7 — STAGE 2: LOCALISATION
 * ========================================================================= */

/* RMS in dBFS over the most recent n samples of a ring. Both channels are
 * sampled at identical indices with an identical decimation, so any bias in
 * the estimator cancels in the difference — which is all we use it for. */
static float win_rms_dbfs(const int16_t *ring, uint32_t head, uint32_t n)
{
    if (n > AM_WIN) n = AM_WIN;
    uint32_t start = (head + AM_WIN - n) % AM_WIN;
    double   s = 0.0;
    uint32_t cnt = 0;
    for (uint32_t i = 0; i < n; i += 2) {
        double v = (double)ring[(start + i) % AM_WIN];
        s += v * v;
        cnt++;
    }
    double r = sqrt(s / (double)cnt) / 32768.0;
    return 20.0f * log10f((float)r + 1e-9f);
}

static float loc_bearing = 0.0f;   /* smoothed, -1 left .. +1 right */
static float loc_prox    = 0.0f;
static float loc_dbL     = -90.0f;
static float loc_dbR     = -90.0f;
static float loc_ild     = 0.0f;   /* raw, balance-corrected */
static bool  loc_gated   = true;

/* Called AFTER am_predict(). Reads the rings; never writes them. */
static void localise(void)
{
    uint32_t head = ring_head;          /* freshest audio, not the classifier's */

    loc_dbL = win_rms_dbfs(ringL, head, LOC_WIN_SAMPLES);
    loc_dbR = win_rms_dbfs(ringR, head, LOC_WIN_SAMPLES) - MIC_BALANCE_DB;

    float loud = (loc_dbL > loc_dbR) ? loc_dbL : loc_dbR;

    /* proximity from the near ear only: using the mean would make a sound
     * hard to one side read as further away than the same sound centred. */
    float p = (loud - PROX_DB_FAR) / (PROX_DB_NEAR - PROX_DB_FAR);
    if (p < 0.0f) p = 0.0f;
    if (p > 1.0f) p = 1.0f;
    loc_prox = p;

    if (loud < LOC_GATE_DBFS) {         /* too quiet to trust — hold bearing */
        loc_gated = true;
        return;
    }
    loc_gated = false;

    /* positive ILD = right mic louder = source on the right */
    float ild = loc_dbR - loc_dbL;
    loc_ild = ild;

    float mag = fabsf(ild);
    float b;
    if (mag <= ILD_DEADBAND_DB) {
        b = 0.0f;
    } else {
        b = (mag - ILD_DEADBAND_DB) / (ILD_FULL_DB - ILD_DEADBAND_DB);
        if (b > 1.0f) b = 1.0f;
        if (ild < 0.0f) b = -b;
    }

    loc_bearing += BEARING_ALPHA * (b - loc_bearing);
    if (loc_bearing < -1.0f) loc_bearing = -1.0f;
    if (loc_bearing >  1.0f) loc_bearing =  1.0f;
}

/* ---- 'k' calibration: average raw ILD with a centred source ---- */
static int    cal_left   = 0;
static double cal_accum  = 0.0;

static void calibration_hop(void)
{
    if (cal_left <= 0) return;

    uint32_t head = ring_head;
    float dL = win_rms_dbfs(ringL, head, LOC_WIN_SAMPLES);
    float dR = win_rms_dbfs(ringR, head, LOC_WIN_SAMPLES);
    cal_accum += (double)(dR - dL);

    if (--cal_left == 0) {
        float off = (float)(cal_accum / CAL_HOPS);
        Serial.println();
        Serial.printf("[cal] mean ILD with centred source = %+.2f dB\n", off);
        Serial.printf("[cal] set  #define MIC_BALANCE_DB  %+.2ff   and re-flash\n", off);
        if (fabsf(off) > 10.0f)
            Serial.println("[cal] that is a very large offset - check the quiet mic's "
                           "SD, L/R and VDD before accepting it as a balance figure");
        Serial.println();
    }
}

/* ===========================================================================
 * SECTION 8 — STAGE 3: CLASS MAP AND EVENT DISPATCH
 * ========================================================================= */

/* Model label index -> haptic class. Order follows AM_LABELS:
 *   0 baby_crying · 1 human_voice · 2 noise_none · 3 traffic          */
static const SoundClass CLASS_MAP[AM_CLASSES] = {
    CLASS_BABY_CRY,   /* baby_crying */
    CLASS_SPEECH,     /* human_voice */
    CLASS_NONE,       /* noise_none  -> nothing to report */
    CLASS_CAR_HORN    /* traffic     */
};

static uint32_t last_active_ms = 0;

/* ---------------------------------------------------------------------------
 * Own-voice link
 * ------------------------------------------------------------------------ */
#if OVD_ENABLE
/* Reads the 1/0 stream from the nRF52840. Its own task rather than the main
 * loop because the loop can sit inside the CNN for ~200 ms, and suppression
 * that arrives 200 ms late has already let the buzz through. */
static void ovd_task(void *arg)
{
    for (;;) {
        while (Serial1.available()) {
            char c = (char)Serial1.read();
            ovdRawBytes++;
            ovdLastByte = (uint8_t)c;

#if OVD_DEBUG_RAW
            Serial.printf("[ovd raw] 0x%02X '%c'\n", (uint8_t)c,
                          (c >= 32 && c < 127) ? c : '.');
#endif
            /* Anything that is not a digit or a line ending means bytes ARE
             * arriving but cannot be parsed — almost always a baud mismatch,
             * which looks identical to a dead wire in the status line unless
             * it is counted separately. */
            if (c != '0' && c != '1') {
                if (c != '\r' && c != '\n') ovdBadBytes++;
                continue;
            }

            bool now = (c == '1');
#if OVD_VERBOSE
            if (now != ovdSpeaking)
                Serial.printf("RX: %c  -> OVD = %s\n", c,
                              now ? "true (USER SPEAKING - suppress haptics)"
                                  : "false (not speaking)");
#endif
            ovdSpeaking = now;
            if (ovdLastRxMs == 0)
                Serial.println(F("[ovd] first frame received - link is up"));
            ovdLastRxMs = millis();
            ovdFrames++;
            if (ovdSpeaking) ovdReleaseAt = millis() + OVD_RELEASE_MS;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}
#endif

/* Is the link alive? Used to fail open rather than silently suppressing
 * alerts because a wire fell out. */
static bool ovdLinkUp(void)
{
#if OVD_ENABLE
    return ovdLastRxMs != 0 && (millis() - ovdLastRxMs) < OVD_TIMEOUT_MS;
#else
    return false;
#endif
}

static bool ownVoiceActive(void)
{
    if (!ovdLinkUp()) return false;            /* no link -> no suppression */
    if (ovdSpeaking)  return true;
    return (int32_t)(ovdReleaseAt - millis()) > 0;   /* hangover */
}

static void driveHaptics(int stable)
{
    if (manualMode) return;             /* phone or serial has the motors */

    SoundClass want = (stable >= 0) ? CLASS_MAP[stable] : CLASS_NONE;

    /* Own voice suppresses the SPEECH pattern only. A car horn during your
     * own sentence is still worth feeling, so the other classes pass through
     * untouched. */
    if (want == CLASS_SPEECH && ownVoiceActive()) {
        want = CLASS_NONE;
        ovdSuppressed++;
    }

    if (want != CLASS_NONE) {
        hapticSetSource(want, loc_bearing, loc_prox);
        last_active_ms = millis();
    } else if (hapticActive() &&
               millis() - last_active_ms > HAPTIC_RELEASE_MS) {
        hapticStop();
    }
}

/* THE one place a status line is formatted. Serial and BLE both call this, so
 * the text on the phone is the text on the cable — there is no second format
 * to keep in step. */
static int fmtStatusLine(char *out, size_t cap,
                         uint32_t t_mfe, uint32_t t_nn,
                         float dbA, float dbB, int used,
                         uint8_t hapticL, uint8_t hapticR, bool ownVoice)
{
    int n = snprintf(out, cap, "[MFE %3lu ms | NN %3lu ms]",
                     (unsigned long)(t_mfe/1000), (unsigned long)(t_nn/1000));
#if CHANNEL_REPORT
    n += snprintf(out + n, cap - n, " A %5.1f dB  B %5.1f dB  src:%s |",
                  dbA, dbB, used == 0 ? "A" : (used == 1 ? "B" : "mix"));
#endif
    for (int i = 0; i < AM_CLASSES; i++)
        n += snprintf(out + n, cap - n, "  %s %.2f", AM_LABELS[i], prob[i]);

    n += snprintf(out + n, cap - n, " | ILD %+5.1f dB  bear %+.2f  prox %.2f%s",
                  loc_ild, loc_bearing, loc_prox, loc_gated ? " (held)" : "");

    /* The duties actually written to the pins. Printed as raw 0-255 PWM
     * because that is the number you compare against DUTY_FLOOR when a motor
     * feels dead — a normalised value would hide exactly that. */
    n += snprintf(out + n, cap - n, " | hapticL %3u  hapticR %3u", hapticL, hapticR);

    /* OVD as a plain true/false, plus a link marker. A pulled wire also reads
     * false, so without "[no link]" a dead cable would be indistinguishable
     * from a quiet wearer — and the whole feature would look like it was
     * working while doing nothing at all. */
#if OVD_ENABLE
    n += snprintf(out + n, cap - n, "  OVD=%s", ownVoice ? "true" : "false");
    if (!ovdLinkUp()) n += snprintf(out + n, cap - n, " [no link]");
    if (ovdSuppressed)
        n += snprintf(out + n, cap - n, " (supp %lu)", (unsigned long)ovdSuppressed);
#endif

    if (manualMode) n += snprintf(out + n, cap - n, "  [MANUAL]");

    /* Loop period answers "why is this slow" without guesswork. ~250 ms here
     * with lines arriving seconds apart on the phone means the classifier is
     * fine and the radio is the bottleneck. */
    /* Loop period, then DEVICE UPTIME. Uptime is the one field that separates
     * "the radio link dropped" from "the board reset": a link drop leaves
     * uptime climbing, a brownout or crash sends it back to zero. Without it
     * the two are indistinguishable from the receiving end. */
    n += snprintf(out + n, cap - n, " | %lu ms  up %lus",
                  (unsigned long)loopPeriodMs, (unsigned long)(millis() / 1000));

    if (bleDropped)
        n += snprintf(out + n, cap - n, "  [ble dropped %lu]", (unsigned long)bleDropped);
    if (overruns)
        n += snprintf(out + n, cap - n, "   [!] overruns=%lu", (unsigned long)overruns);
    return n;
}

/* Queue a line for the radio. Returns immediately. */
static void bleNotifyLine(const char *line)
{
    if (!phoneConnected || ble_q == NULL) return;

    BleLine item;
    strncpy(item.txt, line, BLE_LINE_MAX - 2);
    item.txt[BLE_LINE_MAX - 2] = '\0';
    strcat(item.txt, "\n");      /* terminator rides WITH the line, so it
                                  * costs no extra packet */

    if (xQueueSend(ble_q, &item, 0) != pdTRUE) bleDropped++;
}

/* ===========================================================================
 * SECTION 9 — BLE
 * ========================================================================= */
/* Drains the queue. Splits each line to whatever MTU was actually negotiated,
 * because notify() sends at most (MTU - 3) bytes and discards the rest in
 * silence. */
static void bleSendNow(const char *line)
{
    uint16_t mtu = 23;
    if (server) {
        uint16_t peer = server->getPeerMTU(server->getConnId());
        if (peer >= 23) mtu = peer;
    }
    if (mtu != blePeerMTU) {
        blePeerMTU = mtu;
        Serial.printf("[ble] negotiated MTU %u -> %u bytes/packet, %u packets/line\n",
                      (unsigned)mtu, (unsigned)(mtu - 3),
                      (unsigned)((strlen(line) + (mtu - 4)) / (mtu - 3)));
    }

    size_t chunk = mtu - 3;
    if (chunk < 1) chunk = 20;

    size_t len = strlen(line);
    for (size_t off = 0; off < len; off += chunk) {
        size_t n = len - off;
        if (n > chunk) n = chunk;
        characteristic->setValue((uint8_t *)(line + off), n);
        characteristic->notify();
    }
}

static void ble_task(void *arg)
{
    BleLine item;
    for (;;)
        if (xQueueReceive(ble_q, &item, portMAX_DELAY) == pdTRUE)
            if (phoneConnected && characteristic) bleSendNow(item.txt);
}

class ServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* s) override {
        phoneConnected = true;
        Serial.println("Phone connected");
    }

#if HAVE_GATTS_PARAM
    /* The two-argument form carries the peer address, which updateConnParams
     * needs. Deliberately NOT marked override: if a core drops this overload,
     * an unused method beats a build that will not compile. The Serial line is
     * how you tell whether it is actually being called. */
    void onConnect(BLEServer* s, esp_ble_gatts_cb_param_t *param) {
        phoneConnected = true;
        Serial.println("Phone connected");
        s->updateConnParams(param->connect.remote_bda,
                            BLE_CONN_MIN, BLE_CONN_MAX,
                            BLE_CONN_LATENCY, BLE_CONN_TIMEOUT);
        Serial.printf("[ble] requested %.1f-%.1f ms interval\n",
                      BLE_CONN_MIN * 1.25f, BLE_CONN_MAX * 1.25f);
    }
#endif
    void onDisconnect(BLEServer* s) override {
        phoneConnected = false;
        blePeerMTU     = 23;
        Serial.println("Phone disconnected");
        delay(500);
        advertising->start();
        Serial.println("Advertising restarted");
    }
};

class CharacteristicCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *c) override {
        String value = c->getValue();
        Serial.print("Received pattern id: ");
        Serial.println(value);

        /* Any pattern write hands the motors to the phone so the classifier
         * cannot overwrite what the user is deliberately previewing. */
        if      (value == "1") { manualMode = true;  hapticSetSource(CLASS_SPEECH,   0.0f, 0.8f); }
        else if (value == "2") { manualMode = true;  hapticSetSource(CLASS_CAR_HORN, 0.0f, 0.8f); }
        else if (value == "3") { manualMode = true;  hapticSetSource(CLASS_BABY_CRY, 0.0f, 0.8f); }
        else if (value == "0") { manualMode = false; hapticStop(); }
        else if (value == "a") { manualMode = false; }
    }
};

/* ===========================================================================
 * SECTION 10 — SERIAL TEST HARNESS
 * ========================================================================= */
float testBearing   = 0.0f;
float testProximity = 0.6f;

void printHelp() {
  Serial.println();
  Serial.println("Vibly -- key controls");
  Serial.println("  a   AUTO   (classifier drives the motors)");
  Serial.println("  s   speech      b  baby cry      c  car horn     x  stop");
  Serial.println("  [   pan left    ]  pan right     0  centre");
  Serial.println("  -   farther     =  nearer");
  Serial.println("  k   calibrate mic balance (centred source, 2 s)");
  Serial.println("  u   own-voice link diagnostics + UART loopback test");
  Serial.println("  h   show this help again");
  Serial.println("Any of s/b/c/x/[/]/0/-/= switches to MANUAL. 'a' returns to AUTO.");
  Serial.println("Set Serial Monitor line ending to 'No line ending'.");
#if OVD_ENABLE
  Serial.println("OVD in the log: OVD=true means own voice, so speech alerts are held.");
  Serial.println("'[no link]' means no UART data at all - press 'u' to diagnose.");
#endif
  Serial.println();
}

void reportState(const char* action) {
  Serial.print(action);
  if (hap.cls != CLASS_NONE) {
    Serial.print("  [");        Serial.print(PATTERNS[hap.cls].name); Serial.print("]");
  }
  Serial.print("  bearing ");   Serial.print(testBearing, 2);
  Serial.print("  prox ");      Serial.print(testProximity, 2);
  Serial.print("  dutyL ");     Serial.print(hap.dutyL);
  Serial.print("  dutyR ");     Serial.println(hap.dutyR);
}

void refreshCurrent() {
  if (hap.cls != CLASS_NONE) hapticSetSource(hap.cls, testBearing, testProximity);
}

void handleSerialCommand() {
  while (Serial.available()) {
    char k = Serial.read();
    if (k == '\n' || k == '\r' || k == ' ') continue;

    switch (k) {
      case 'a': case 'A':
        manualMode = false;
        hapticStop();
        Serial.println("AUTO - classifier drives the motors");
        break;

      case 's': case 'S':
        manualMode = true;
        hapticSetSource(CLASS_SPEECH,   testBearing, testProximity);
        reportState("speech");
        break;

      case 'b': case 'B':
        manualMode = true;
        hapticSetSource(CLASS_BABY_CRY, testBearing, testProximity);
        reportState("baby cry");
        break;

      case 'c': case 'C':
        manualMode = true;
        hapticSetSource(CLASS_CAR_HORN, testBearing, testProximity);
        reportState("car horn");
        break;

      case 'x': case 'X':
        manualMode = true;
        hapticStop();
        Serial.println("stopped");
        break;

      case '[':
        manualMode = true;
        testBearing -= 0.25f;
        if (testBearing < -1.0f) testBearing = -1.0f;
        refreshCurrent(); reportState("pan left");
        break;

      case ']':
        manualMode = true;
        testBearing += 0.25f;
        if (testBearing > 1.0f) testBearing = 1.0f;
        refreshCurrent(); reportState("pan right");
        break;

      case '0':
        manualMode = true;
        testBearing = 0.0f;
        refreshCurrent(); reportState("centre");
        break;

      case '-': case '_':
        manualMode = true;
        testProximity -= 0.2f;
        if (testProximity < 0.0f) testProximity = 0.0f;
        refreshCurrent(); reportState("farther");
        break;

      case '=': case '+':
        manualMode = true;
        testProximity += 0.2f;
        if (testProximity > 1.0f) testProximity = 1.0f;
        refreshCurrent(); reportState("nearer");
        break;

      case 'u': case 'U': {
#if OVD_ENABLE
        uint32_t age = ovdLastRxMs ? (millis() - ovdLastRxMs) : 0;
        Serial.println();
        Serial.println(F("--- OVD link diagnostics ---"));
        Serial.printf("  UART        : RX GPIO%d, TX GPIO%d, %d baud\n",
                      OVD_RX_PIN, OVD_TX_PIN, OVD_BAUD);
        Serial.printf("  raw bytes   : %lu\n", (unsigned long)ovdRawBytes);
        Serial.printf("  valid 0/1   : %lu\n", (unsigned long)ovdFrames);
        Serial.printf("  unparseable : %lu\n", (unsigned long)ovdBadBytes);
        if (ovdRawBytes)
            Serial.printf("  last byte   : 0x%02X '%c'\n", ovdLastByte,
                          (ovdLastByte >= 32 && ovdLastByte < 127) ? ovdLastByte : '.');
        if (ovdLastRxMs) Serial.printf("  last frame  : %lu ms ago\n", (unsigned long)age);
        Serial.printf("  link        : %s\n", ovdLinkUp() ? "UP" : "DOWN");

        /* The decisive test. Jumper GPIO43 to GPIO44 and press 'u': if the
         * ESP32 hears its own byte, this board's UART is fine and the fault
         * is the nRF52840 or the wire between them. If it hears nothing even
         * looped back, the fault is on this side and no amount of rewiring
         * the other board will help. */
        uint32_t before = ovdRawBytes;
        Serial1.write('1');
        Serial1.write('\n');
        delay(60);
        if (ovdRawBytes > before)
            Serial.println(F("  loopback    : HEARD - this board's UART works. "
                             "If you are NOT jumpering TX to RX, the nRF52840 "
                             "is sending and something else is wrong."));
        else
            Serial.println(F("  loopback    : silent. Jumper GPIO43 to GPIO44 and "
                             "press 'u' again. Still silent = this board's UART "
                             "or pin choice is the fault, not the nRF52840."));

        if (ovdRawBytes && !ovdFrames)
            Serial.println(F("  [!] bytes arrive but none parse - almost certainly "
                             "a BAUD MISMATCH. Both ends must be 115200."));
        if (!ovdRawBytes)
            Serial.println(F("  [!] nothing at all - check nRF D6 -> ESP32 D7 (GPIO44), "
                             "and that the two boards SHARE A GROUND."));
        Serial.println();
#else
        Serial.println(F("OVD is disabled at compile time (OVD_ENABLE 0)"));
#endif
        break;
      }

      case 'k': case 'K':
        cal_left  = CAL_HOPS;
        cal_accum = 0.0;
        Serial.println("[cal] hold a steady sound directly in front, 1 m away, "
                       "equidistant from both mics ... measuring 2 s");
        break;

      case '?': case 'h': case 'H':
        printHelp();
        break;

      default:
        Serial.print("unknown key '"); Serial.print(k);
        Serial.println("'  -- press h for help");
        break;
    }
  }
}

/* ===========================================================================
 * SECTION 11 — SETUP
 * ========================================================================= */
void setup()
{
    Serial.begin(115200);
    uint32_t t0 = millis();
    while (!Serial && millis() - t0 < 3000) delay(10);
    delay(200);

    /* ---- motors first: get them into a known-off state before anything
     * else can take time or fail ---- */
    if (!ledcAttach(LEFT_MOTOR_PIN,  PWM_FREQ_HZ, PWM_RES_BITS) ||
        !ledcAttach(RIGHT_MOTOR_PIN, PWM_FREQ_HZ, PWM_RES_BITS)) {
        Serial.println("ERROR: ledcAttach failed. Needs ESP32 core 3.x.");
        while (1) delay(500);
    }
    writeMotors(0, 0);

    static const char *MODE_NAME[] = { "BOTH (fuse)", "MIX (average)",
                                       "BEST (louder)", "LEFT only", "RIGHT only" };
    Serial.println();
    Serial.println(F("Vibly - dual INMP441 classifier + localised haptics"));
    Serial.printf ("window %d @ %d Hz | MFE %dx%d | %d classes\n",
                   AM_WIN, AM_SR, AM_FRAMES, AM_MELS, AM_CLASSES);
    Serial.printf ("fusion %s | hop %.0f ms | SHIFT_BITS %d | DC %s | swap %d\n",
                   MODE_NAME[FUSION], 1000.0f*HOP_SAMPLES/AM_SR,
                   SHIFT_BITS, ENABLE_DC_BLOCK ? "on" : "off", SWAP_CHANNELS);
    Serial.printf ("localiser: %.0f ms window | balance %+.2f dB | deadband %.1f dB"
                   " | full scale %.1f dB\n",
                   1000.0f*LOC_WIN_SAMPLES/AM_SR, (float)MIC_BALANCE_DB,
                   (float)ILD_DEADBAND_DB, (float)ILD_FULL_DB);
    Serial.printf ("mic A = L/R->GND (left slot), mic B = L/R->3V3 (right slot)\n");

    am_init();
    memset(ringL, 0, sizeof(ringL));
    memset(ringR, 0, sizeof(ringR));
    for (int i = 0; i < SMOOTH_M; i++) hist[i] = -1;

#if OVD_ENABLE
    Serial1.begin(OVD_BAUD, SERIAL_8N1, OVD_RX_PIN, OVD_TX_PIN);
    Serial.printf("[ovd] listening on GPIO%d (D7) @ %d baud", OVD_RX_PIN, OVD_BAUD);
  #if HAPTIC_BLANK_PIN >= 0
    Serial.printf(", blanking out on GPIO%d", HAPTIC_BLANK_PIN);
  #endif
    Serial.println();
    Serial.println(F("[ovd] no data yet - own-voice suppression stays OFF until "
                     "the nRF52840 starts sending"));
#else
    Serial.println(F("[ovd] disabled at compile time"));
#endif

    hap_mux = xSemaphoreCreateMutex();
    hop_sem = xSemaphoreCreateBinary();
    if (!hap_mux || !hop_sem) { Serial.println(F("FATAL: no RTOS objects")); while (1) delay(1000); }

    if (!i2s_init()) { Serial.println(F("FATAL: I2S init failed")); while (1) delay(1000); }

    xTaskCreatePinnedToCore(capture_task, "i2s", 4096, NULL,
                            configMAX_PRIORITIES - 2, &cap_task, 0);

    /* Priority 2 on core 1 puts the haptic timeline above the Arduino loop
     * task (priority 1), so it preempts the CNN rather than queueing behind
     * it. Each tick is a handful of microseconds, so the cost to inference
     * throughput is negligible. */
    xTaskCreatePinnedToCore(haptic_task, "haptic", 3072, NULL,
                            2, &hap_task, 1);

#if OVD_ENABLE
    xTaskCreatePinnedToCore(ovd_task, "ovd", 2048, NULL, 2, NULL, 0);
#endif

    BLEDevice::init("Vibly");
    /* AFTER init: this configures the running GATT stack. Called before, there
     * is nothing to configure and the request is silently dropped, which is
     * the quiet way to stay stuck at 23 bytes per packet. */
    BLEDevice::setMTU(BLE_REQUEST_MTU);

    ble_q = xQueueCreate(BLE_QUEUE_LEN, sizeof(BleLine));
    if (!ble_q) { Serial.println(F("FATAL: no BLE queue")); while (1) delay(1000); }
    xTaskCreatePinnedToCore(ble_task, "ble_tx", 4096, NULL, 1, NULL, 1);

    server = BLEDevice::createServer();
    server->setCallbacks(new ServerCallbacks());
    service = server->createService(SERVICE_UUID);
    characteristic = service->createCharacteristic(
        CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_READ |
        BLECharacteristic::PROPERTY_WRITE |
        BLECharacteristic::PROPERTY_NOTIFY);
    characteristic->setCallbacks(new CharacteristicCallbacks());
    characteristic->addDescriptor(new BLE2902());
    service->start();
    advertising = BLEDevice::getAdvertising();
    advertising->addServiceUUID(SERVICE_UUID);
    advertising->setScanResponse(true);
    /* Advertised preference, same Apple-compatible window as the request
     * above. 0x06 (7.5 ms) here is the value most ESP32 examples carry, and it
     * breaks the same rule. */
    advertising->setMinPreferred(0x0C);   /* 15 ms */
    advertising->setMaxPreferred(0x18);   /* 30 ms */
    BLEDevice::startAdvertising();
    Serial.println("Advertising as Vibly...");

    Serial.printf("free heap %u B\n", (unsigned)ESP.getFreeHeap());

    delay(1100);                       /* let both rings fill once */
    printHelp();
    Serial.println(F("Listening...\n"));
}

/* ===========================================================================
 * SECTION 12 — MAIN LOOP
 * ========================================================================= */
void loop()
{
    /* Short timeout rather than portMAX_DELAY: the hop only arrives every
     * 250 ms and serial keys should not have to wait for it. */
    if (xSemaphoreTake(hop_sem, pdMS_TO_TICKS(20)) != pdTRUE) {
        handleSerialCommand();
        return;
    }
    handleSerialCommand();

    {   static uint32_t prev = 0;
        uint32_t now = millis();
        if (prev) loopPeriodMs = now - prev;
        prev = now;
    }

    const uint32_t head = ring_head;

    /* ===================================================================
     * STAGE 1 — CLASSIFY. Nothing below this comment until the marked end
     * knows anything about direction.
     * =================================================================== */
    float dbA = ring_rms_dbfs(ringL);
    float dbB = ring_rms_dbfs(ringR);

    uint32_t t_mfe = 0, t_nn = 0, m0, m1, m2;
    int used = -1;

#if FUSION == MODE_BOTH
    snapshot(ringL, head);
    m0 = micros(); am_mfe(linbuf, feat); m1 = micros(); classify(feat, probA); m2 = micros();
    t_mfe += m1 - m0; t_nn += m2 - m1;

    snapshot(ringR, head);
    m0 = micros(); am_mfe(linbuf, feat); m1 = micros(); classify(feat, probB); m2 = micros();
    t_mfe += m1 - m0; t_nn += m2 - m1;

    float sum = 0.0f;
    for (int i = 0; i < AM_CLASSES; i++) {
        prob[i] = probA[i] > probB[i] ? probA[i] : probB[i];
        sum += prob[i];
    }
    for (int i = 0; i < AM_CLASSES; i++) prob[i] /= sum;

    {   int ta = 0, tb = 0;
        for (int i = 1; i < AM_CLASSES; i++) {
            if (probA[i] > probA[ta]) ta = i;
            if (probB[i] > probB[tb]) tb = i;
        }
        used = (probA[ta] >= probB[tb]) ? 0 : 1;
    }

#elif FUSION == MODE_MIX
    snapshot_mix(head);
    m0 = micros(); am_mfe(linbuf, feat); m1 = micros(); classify(feat, prob); m2 = micros();
    t_mfe = m1 - m0; t_nn = m2 - m1;
    used = 2;

#elif FUSION == MODE_BEST
    used = (dbA >= dbB) ? 0 : 1;
    snapshot(used == 0 ? ringL : ringR, head);
    m0 = micros(); am_mfe(linbuf, feat); m1 = micros(); classify(feat, prob); m2 = micros();
    t_mfe = m1 - m0; t_nn = m2 - m1;

#elif FUSION == MODE_LEFT
    snapshot(ringL, head);
    m0 = micros(); am_mfe(linbuf, feat); m1 = micros(); classify(feat, prob); m2 = micros();
    t_mfe = m1 - m0; t_nn = m2 - m1;
    used = 0;

#else /* MODE_RIGHT */
    snapshot(ringR, head);
    m0 = micros(); am_mfe(linbuf, feat); m1 = micros(); classify(feat, prob); m2 = micros();
    t_mfe = m1 - m0; t_nn = m2 - m1;
    used = 1;
#endif

    int   top  = 0;
    float best = prob[0];
    for (int i = 1; i < AM_CLASSES; i++) if (prob[i] > best) { best = prob[i]; top = i; }

    hist[hist_idx] = (best >= CONF_THRESHOLD) ? top : -1;
    hist_idx = (hist_idx + 1) % SMOOTH_M;

    int votes[AM_CLASSES] = {0};
    for (int i = 0; i < SMOOTH_M; i++) if (hist[i] >= 0) votes[hist[i]]++;
    int stable = -1;
    for (int i = 0; i < AM_CLASSES; i++) if (votes[i] >= SMOOTH_N) { stable = i; break; }
    /* ================= end of STAGE 1 ================================== */

    /* ===================================================================
     * STAGE 2 — LOCALISE. The class is already decided and immutable.
     * =================================================================== */
    localise();
    calibration_hop();

    /* ===================================================================
     * STAGE 3 — RENDER
     * =================================================================== */
    driveHaptics(stable);

    {   /* Heap is watched on Serial, not BLE, so it survives a link that is
         * dropping. A steady fall here means a leak, and a leak ends in a
         * reboot that looks exactly like a Bluetooth fault. */
        static uint32_t lastHeap = 0;
        if (millis() - lastHeap > 10000) {
            lastHeap = millis();
            Serial.printf("[health] up %lus  heap %u B  overruns %lu  "
                          "ble dropped %lu  ovd frames %lu\n",
                          (unsigned long)(millis()/1000), (unsigned)ESP.getFreeHeap(),
                          (unsigned long)overruns, (unsigned long)bleDropped,
                          (unsigned long)ovdFrames);
        }
    }

    /* One line, built once, sent to both destinations. Duties are read AFTER
     * driveHaptics() so the log shows what the motors are doing now, not what
     * they were doing before this hop's decision. */
    uint8_t hapticL, hapticR;
    hapticDuties(hapticL, hapticR);
    bool ownVoice = ownVoiceActive();

    char line[BLE_LINE_MAX];
    fmtStatusLine(line, sizeof(line), t_mfe, t_nn, dbA, dbB, used,
                  hapticL, hapticR, ownVoice);

#if PRINT_EVERY
    Serial.println(line);

  #if CHANNEL_REPORT
    {   static uint32_t bad = 0;
        float d = dbA - dbB;
        if (d < 0) d = -d;
        if (d > DEAD_MIC_DB) {
            if (++bad == 20) {
                char w[160];
                snprintf(w, sizeof(w),
                         "[!] mic %s is %.0f dB below the other for 5 s - "
                         "check its wiring, its L/R pin, and that it is not covered",
                         dbA < dbB ? "A" : "B", d);
                Serial.println(w);
                bleNotifyLine(w);
            }
        } else bad = 0;
    }
  #endif
#endif

#if BLE_SEND_STATUS
    {   static uint32_t nth = 0;
        if ((nth++ % BLE_STATUS_EVERY_N) == 0) bleNotifyLine(line);
    }
#endif

    if (stable >= 0 && stable != last_reported) {
        const char *side = loc_bearing < -0.15f ? "LEFT"
                         : loc_bearing >  0.15f ? "RIGHT" : "CENTRE";
        char ev[160];
        snprintf(ev, sizeof(ev),
                 ">>> EVENT: %-12s  %s (bearing %+.2f, prox %.2f)  -> pattern: %s"
                 "  hapticL %u  hapticR %u%s",
                 AM_LABELS[stable], side, loc_bearing, loc_prox,
                 PATTERNS[CLASS_MAP[stable]].name, hapticL, hapticR,
                 ownVoice ? "  OVD=true" : "");
        Serial.println(ev);
        bleNotifyLine(ev);
        last_reported = stable;
    } else if (stable < 0) {
        bool any = false;
        for (int i = 0; i < SMOOTH_M; i++) if (hist[i] >= 0) any = true;
        if (!any) last_reported = -1;
    }
}
