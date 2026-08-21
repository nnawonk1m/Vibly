/* ============================================================================
 * XIAO ESP32-S3 + TWO INMP441 — data capture / bring-up firmware
 * ----------------------------------------------------------------------------
 * Use this FIRST, before the inference sketch. It does two jobs:
 *
 *   LEVEL_METER 1  ->  per-microphone level meter. Confirms both mics are
 *                      alive, tells you which one is A and which is B, and
 *                      lets you set SHIFT_BITS. No streaming.
 *
 *   LEVEL_METER 0  ->  streams both channels as interleaved int16 over USB.
 *                      Pair with capture_wav_dual.py to record WAVs for
 *                      training / testing.
 *
 * Wiring is identical to the inference sketch — see its header. In short:
 *   both mics: VDD->3V3, GND->GND, SCK->D2/GPIO3, WS->D3/GPIO4, SD->D4/GPIO5
 *   mic A: L/R -> GND      mic B: L/R -> 3V3
 *
 * BRING-UP SEQUENCE
 *   1. Flash with LEVEL_METER 1, open Serial Monitor @ 115200.
 *   2. Silence: both channels should read about -60 to -70 dBFS.
 *   3. Tap mic A. The "A" column should jump and "B" should barely move.
 *      If "B" jumps instead, set SWAP_CHANNELS to 1 and re-flash.
 *   4. Tap mic B. The "B" column should jump.
 *      If NEITHER column responds to one of the mics, that mic's L/R pin is
 *      wrong (both tied the same way) or its SD/power is not connected.
 *   5. Speak at 30 cm. Peaks should land at 20-60 % FS with clipped = 0.
 *      Too quiet -> lower SHIFT_BITS. Clipping -> raise it.
 *   6. Write the SHIFT_BITS value down. It must be identical in the
 *      inference sketch and in every recording you ever make.
 * ==========================================================================*/

#include <driver/i2s.h>

#define LEVEL_METER      0      /* 1 = meter/bring-up, 0 = stream to PC */

/* ---------------------------------------------------------------- hardware */
#define I2S_SCK          3      /* D2 - BCLK, shared */
#define I2S_WS           4      /* D3 - WS,   shared */
#define I2S_SD           5      /* D4 - SD,   both mics tied together */
#define I2S_PORT         I2S_NUM_0

#define SAMPLE_RATE      16000
#define DMA_FRAMES       256
#define DMA_BUF_COUNT    8

#define SHIFT_BITS       14     /* 16 = unity 24->16 bit; each step = 6 dB */
#define ENABLE_DC_BLOCK  1
#define DC_BLOCK_R       0.995f
#define SWAP_CHANNELS    0

#define BLOCK_FRAMES     512

#if SWAP_CHANNELS
  #define IDX_A 1
  #define IDX_B 0
#else
  #define IDX_A 0
  #define IDX_B 1
#endif

static int32_t i2s_raw[BLOCK_FRAMES * 2];
static int16_t pcm[BLOCK_FRAMES * 2];        /* interleaved A,B */
static volatile bool streaming = false;
static const char MAGIC[8] = { 'E','I','W','A','V','D','2','\n' };

/* ========================================================================= */
static bool i2s_init(void)
{
    i2s_config_t cfg = {
        .mode                 = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
        .sample_rate          = SAMPLE_RATE,
        .bits_per_sample      = I2S_BITS_PER_SAMPLE_32BIT,
        .channel_format       = I2S_CHANNEL_FMT_RIGHT_LEFT,   /* both slots */
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags     = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count        = DMA_BUF_COUNT,
        .dma_buf_len          = DMA_FRAMES,
        .use_apll             = true,        /* no-op on the S3 */
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

/* returns number of FRAMES written into pcm[] (2 int16 per frame) */
static int read_block(void)
{
    static float ax = 0.0f, ay = 0.0f, bx = 0.0f, by = 0.0f;
    size_t br = 0;
    if (i2s_read(I2S_PORT, i2s_raw, sizeof(i2s_raw), &br, portMAX_DELAY) != ESP_OK)
        return 0;
    int n = br / (sizeof(int32_t) * 2);

    for (int i = 0; i < n; i++) {
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
        pcm[2*i]     = (int16_t)a;
        pcm[2*i + 1] = (int16_t)b;
    }
    return n;
}

/* ========================================================================= */
void setup()
{
    Serial.begin(921600);
    uint32_t t0 = millis();
    while (!Serial && millis() - t0 < 3000) delay(10);
    delay(200);

    if (!i2s_init()) {
#if LEVEL_METER
        Serial.println("FATAL: I2S init failed");
#endif
        while (1) delay(1000);
    }

#if LEVEL_METER
    Serial.println();
    Serial.println("Dual INMP441 level meter");
    Serial.printf("SHIFT_BITS=%d  DC block=%s  SWAP_CHANNELS=%d\n",
                  SHIFT_BITS, ENABLE_DC_BLOCK ? "on" : "off", SWAP_CHANNELS);
    Serial.println("A = the mic with L/R tied to GND, B = the mic with L/R tied to 3V3");
    Serial.println("Tap mic A, then mic B, then speak at 30 cm, then stay silent.\n");
#endif
}

/* ========================================================================= */
void loop()
{
#if LEVEL_METER
    static double sa = 0.0, sb = 0.0;
    static uint32_t cnt = 0, clipA = 0, clipB = 0, tlast = 0;
    static int32_t pkA = 0, pkB = 0;

    int n = read_block();
    for (int i = 0; i < n; i++) {
        int32_t a = pcm[2*i], b = pcm[2*i + 1];
        int32_t aa = a < 0 ? -a : a, ab = b < 0 ? -b : b;
        if (aa > pkA) pkA = aa;
        if (ab > pkB) pkB = ab;
        if (aa >= 32700) clipA++;
        if (ab >= 32700) clipB++;
        sa += (double)a * a;
        sb += (double)b * b;
    }
    cnt += n;

    if (millis() - tlast >= 1000 && cnt > 0) {
        double ra = 20.0 * log10((sqrt(sa / cnt) + 1e-9) / 32768.0);
        double rb = 20.0 * log10((sqrt(sb / cnt) + 1e-9) / 32768.0);
        Serial.printf("A: RMS %6.1f dBFS peak %5.1f%% clip %4lu   |   "
                      "B: RMS %6.1f dBFS peak %5.1f%% clip %4lu",
                      ra, 100.0 * pkA / 32768.0, (unsigned long)clipA,
                      rb, 100.0 * pkB / 32768.0, (unsigned long)clipB);
        if (ra - rb >  25.0) Serial.print("   <-- B looks dead / miswired");
        if (rb - ra >  25.0) Serial.print("   <-- A looks dead / miswired");
        Serial.println();
        sa = sb = 0.0; cnt = 0; pkA = pkB = 0; clipA = clipB = 0;
        tlast = millis();
    }

#else   /* ---------------------------------- interleaved streaming to the PC */

    if (Serial.available()) {
        int c = Serial.read();
        if (c == 'R' && !streaming) {
            Serial.write((const uint8_t *)MAGIC, sizeof(MAGIC));
            Serial.flush();
            streaming = true;
        } else if (c == 'S') {
            streaming = false;
        }
    }

    int n = read_block();
    if (streaming && n > 0)
        Serial.write((const uint8_t *)pcm, n * 2 * sizeof(int16_t));
#endif
}
