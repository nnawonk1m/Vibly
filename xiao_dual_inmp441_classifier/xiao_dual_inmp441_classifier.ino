/* ============================================================================
 * XIAO ESP32-S3 + TWO INMP441 microphones — 4-class audio classifier
 * ----------------------------------------------------------------------------
 * Same model as the single-mic build. audio_model.h is byte-identical; only the
 * audio front-end changed. Put audio_model.h in this folder.
 *
 * Classes: baby_crying · human_voice · noise_none · traffic
 *
 * ---------------------------------------------------------------------------
 * WIRING — both microphones share ONE I2S bus (3 GPIOs total)
 * ---------------------------------------------------------------------------
 * The INMP441 puts its data in the left or the right half of the I2S frame
 * depending on its L/R pin (datasheet: L/R low = left channel, high = right),
 * and tri-states its output during the other half. So two mics can share a
 * single SD line — that is the intended stereo pairing, not a hack.
 *
 *      signal        MIC A (left)      MIC B (right)     XIAO ESP32-S3
 *      ---------------------------------------------------------------
 *      VDD           VDD               VDD               3V3
 *      GND           GND               GND               GND
 *      L/R           GND   <-- low     3V3  <-- high     (not a XIAO pin)
 *      SCK  (BCLK)   SCK               SCK               D2 / GPIO3
 *      WS   (LRCLK)  WS                WS                D3 / GPIO4
 *      SD   (DOUT)   SD ---------------SD -------------- D4 / GPIO5
 *
 *   * The ONLY difference between the two modules is the L/R pin.
 *     A: L/R -> GND.   B: L/R -> 3V3.   Getting both the same = one silent
 *     channel and two mics fighting over the same time slot.
 *   * Tie the two SD pins together and run ONE wire to D4.
 *   * 100 nF + 10 uF from VDD to GND at EACH module, as close as you can.
 *   * Keep SCK / WS / SD under ~15 cm. They are 1.024 MHz signals.
 *   * Spacing between the two mics:
 *       MODE_BOTH / MODE_BEST : any spacing, even opposite sides of a box.
 *       MODE_MIX              : keep them 3-6 cm apart. Wider spacing causes
 *                               comb filtering above ~3 kHz for off-axis
 *                               sources, which the model was not trained on.
 *
 * ---------------------------------------------------------------------------
 * ARDUINO IDE
 * ---------------------------------------------------------------------------
 *   Board XIAO_ESP32S3 · PSRAM = OPI PSRAM · CPU = 240 MHz
 *   USB CDC On Boot = Enabled · Partition Scheme = 8M with spiffs
 *   Serial Monitor @ 115200
 *
 *   FIRST RUN: leave FUSION = MODE_BOTH and CHANNEL_REPORT = 1. Tap mic A,
 *   then mic B, and check the "L" and "R" levels move independently and in
 *   the order you expect. If they are swapped, set SWAP_CHANNELS to 1.
 * ==========================================================================*/

#include <driver/i2s.h>
#include "audio_model.h"

/* ---------------------------------------------------------------- hardware */
#define I2S_SCK          3      /* D2 - BCLK, shared by both mics */
#define I2S_WS           4      /* D3 - WS,   shared by both mics */
#define I2S_SD           5      /* D4 - SD,   both mics tied together */
#define I2S_PORT         I2S_NUM_0

#define DMA_FRAMES       256    /* frames per DMA buffer (1 frame = L + R) */
#define DMA_BUF_COUNT    8      /* 8 x 256 frames = 128 ms of slack */

/* Digital gain. 16 = unity 24->16 bit, each step of 1 is 6 dB.
 * MUST be the same value used to record the training data. */
#define SHIFT_BITS       14

#define ENABLE_DC_BLOCK  1
#define DC_BLOCK_R       0.995f

/* Set to 1 if the level meter shows the two mics the wrong way round. */
#define SWAP_CHANNELS    0

/* --------------------------------------------------------------- behaviour */
#define MODE_BOTH   0   /* classify BOTH mics, fuse the results  (default)   */
#define MODE_MIX    1   /* average the two mics, one inference               */
#define MODE_BEST   2   /* per window, use whichever mic is louder           */
#define MODE_LEFT   3   /* mic A only  (debug / fallback)                    */
#define MODE_RIGHT  4   /* mic B only  (debug / fallback)                    */

#define FUSION           MODE_BOTH

#define HOP_SAMPLES      4000   /* 250 ms between inferences                 */
#define CONF_THRESHOLD   0.70f
#define SMOOTH_M         5
#define SMOOTH_N         3
#define PRINT_EVERY      1
#define CHANNEL_REPORT   1      /* print per-mic level + a dead-mic warning  */
#define DEAD_MIC_DB      25.0f  /* warn if one mic is this far below the other */

/* ------------------------------------------------------------------ memory */
static int16_t  ringL[AM_WIN], ringR[AM_WIN];     /* 32 KB each */
static volatile uint32_t ring_head = 0;
static volatile uint32_t since_hop = 0;
static volatile uint32_t overruns  = 0;

static int16_t  linbuf[AM_WIN];                   /* contiguous work copy */
static float    feat[AM_FRAMES * AM_MELS];
static float    probA[AM_CLASSES], probB[AM_CLASSES], prob[AM_CLASSES];

static int32_t  i2s_raw[DMA_FRAMES * 2];          /* interleaved L,R */
static TaskHandle_t cap_task = NULL;
static SemaphoreHandle_t hop_sem;

static int hist[SMOOTH_M];
static int hist_idx = 0, last_reported = -1;

#if SWAP_CHANNELS
  #define IDX_A 1
  #define IDX_B 0
#else
  #define IDX_A 0
  #define IDX_B 1
#endif

/* ========================================================================= */
/*  I2S — stereo, 32-bit, both slots                                          */
/* ========================================================================= */
static bool i2s_init(void)
{
    i2s_config_t cfg = {
        .mode                 = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
        .sample_rate          = AM_SR,                     /* per channel */
        .bits_per_sample      = I2S_BITS_PER_SAMPLE_32BIT,
        /* the one real change vs the single-mic sketch: take BOTH slots */
        .channel_format       = I2S_CHANNEL_FMT_RIGHT_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags     = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count        = DMA_BUF_COUNT,
        .dma_buf_len          = DMA_FRAMES,
        .use_apll             = true,   /* no-op on the S3; kept for parity  */
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
    while (millis() - t0 < 200)          /* discard the settling click */
        i2s_read(I2S_PORT, i2s_raw, sizeof(i2s_raw), &br, portMAX_DELAY);
    return true;
}

static void capture_task(void *arg)
{
    size_t br;
    float ax = 0.0f, ay = 0.0f, bx = 0.0f, by = 0.0f;   /* DC blocker state */

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

/* copy one ring into linbuf, oldest sample first */
static void snapshot(const int16_t *ring, uint32_t head)
{
    uint32_t tail = AM_WIN - head;
    memcpy(linbuf,        ring + head, tail * sizeof(int16_t));
    memcpy(linbuf + tail, ring,        head * sizeof(int16_t));
}

/* average both rings into linbuf */
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
    for (uint32_t i = 0; i < AM_WIN; i += 4) {      /* every 4th sample is plenty */
        double v = (double)ring[i];
        s += v * v;
    }
    double r = sqrt(s / (AM_WIN / 4.0)) / 32768.0;
    return 20.0f * log10f((float)r + 1e-9f);
}

static void classify(const float *f, float *out) { am_predict(f, out); }

/* ========================================================================= */
void setup()
{
    Serial.begin(115200);
    uint32_t t0 = millis();
    while (!Serial && millis() - t0 < 3000) delay(10);
    delay(200);

    static const char *MODE_NAME[] = { "BOTH (fuse)", "MIX (average)",
                                       "BEST (louder)", "LEFT only", "RIGHT only" };
    Serial.println();
    Serial.println(F("XIAO ESP32-S3 + 2x INMP441 - audio classifier"));
    Serial.printf ("window %d @ %d Hz | MFE %dx%d | %d classes\n",
                   AM_WIN, AM_SR, AM_FRAMES, AM_MELS, AM_CLASSES);
    Serial.printf ("fusion %s | hop %.0f ms | SHIFT_BITS %d | DC %s | swap %d\n",
                   MODE_NAME[FUSION], 1000.0f*HOP_SAMPLES/AM_SR,
                   SHIFT_BITS, ENABLE_DC_BLOCK ? "on" : "off", SWAP_CHANNELS);
    Serial.printf ("mic A = L/R->GND (left slot), mic B = L/R->3V3 (right slot)\n");
    Serial.printf ("free heap %u B\n", (unsigned)ESP.getFreeHeap());

    am_init();
    memset(ringL, 0, sizeof(ringL));
    memset(ringR, 0, sizeof(ringR));
    for (int i = 0; i < SMOOTH_M; i++) hist[i] = -1;

    hop_sem = xSemaphoreCreateBinary();
    if (!i2s_init()) { Serial.println(F("FATAL: I2S init failed")); while (1) delay(1000); }
    xTaskCreatePinnedToCore(capture_task, "i2s", 4096, NULL,
                            configMAX_PRIORITIES - 2, &cap_task, 0);

    delay(1100);                       /* let both rings fill once */
    Serial.println(F("Listening...\n"));
}

/* ========================================================================= */
void loop()
{
    if (xSemaphoreTake(hop_sem, portMAX_DELAY) != pdTRUE) return;

    const uint32_t head = ring_head;
    float dbA = ring_rms_dbfs(ringL);
    float dbB = ring_rms_dbfs(ringR);

    uint32_t t_mfe = 0, t_nn = 0, m0, m1, m2;
    int used = -1;                     /* which mic produced the verdict */

#if FUSION == MODE_BOTH
    snapshot(ringL, head);
    m0 = micros(); am_mfe(linbuf, feat); m1 = micros(); classify(feat, probA); m2 = micros();
    t_mfe += m1 - m0; t_nn += m2 - m1;

    snapshot(ringR, head);
    m0 = micros(); am_mfe(linbuf, feat); m1 = micros(); classify(feat, probB); m2 = micros();
    t_mfe += m1 - m0; t_nn += m2 - m1;

    /* fuse: per class take the more confident microphone, then renormalise.
     * This is deliberately optimistic — a mic that is blocked, pointing the
     * wrong way, or wind-loaded simply loses the vote instead of dragging the
     * good mic down, which is the whole point of the second microphone. */
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

#if PRINT_EVERY
    Serial.printf("[MFE %3lu ms | NN %3lu ms]",
                  (unsigned long)(t_mfe/1000), (unsigned long)(t_nn/1000));
  #if CHANNEL_REPORT
    Serial.printf(" A %5.1f dB  B %5.1f dB  src:%s |",
                  dbA, dbB, used == 0 ? "A" : (used == 1 ? "B" : "mix"));
  #endif
    for (int i = 0; i < AM_CLASSES; i++)
        Serial.printf("  %s %.2f", AM_LABELS[i], prob[i]);
    if (overruns) Serial.printf("   [!] overruns=%lu", (unsigned long)overruns);
    Serial.println();

  #if CHANNEL_REPORT
    {   static uint32_t bad = 0;
        float d = dbA - dbB;
        if (d < 0) d = -d;
        if (d > DEAD_MIC_DB) {
            if (++bad == 20)
                Serial.printf("[!] mic %s is %.0f dB below the other for 5 s - "
                              "check its wiring, its L/R pin, and that it is not covered\n",
                              dbA < dbB ? "A" : "B", d);
        } else bad = 0;
    }
  #endif
#endif

    if (stable >= 0 && stable != last_reported) {
        Serial.printf(">>> EVENT: %s", AM_LABELS[stable]);
        if (used == 0 || used == 1) Serial.printf("   (loudest on mic %c)", used == 0 ? 'A' : 'B');
        Serial.println();
        last_reported = stable;
        /* hook your action here: GPIO, BLE, MQTT ... */
    } else if (stable < 0) {
        bool any = false;
        for (int i = 0; i < SMOOTH_M; i++) if (hist[i] >= 0) any = true;
        if (!any) last_reported = -1;
    }
}
