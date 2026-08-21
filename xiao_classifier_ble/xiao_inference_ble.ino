/* ============================================================================
 * xiao_inference_ble.ino
 * XIAO ESP32-S3 · 2x INMP441 · 4-class classifier · log streamed over BLE
 * ----------------------------------------------------------------------------
 * Merge of xiao_dual_inmp441_inference.ino and ble_adv.ino.
 *
 * ble_adv.ino notified a hardcoded string once a second to prove the transport
 * worked. This sends the real classifier output instead, four times a second,
 * and the same text still goes to Serial so USB and BLE agree line for line.
 *
 * ---------------------------------------------------------------------------
 * THREE THINGS CHANGED FROM ble_adv.ino, AND WHY
 * ---------------------------------------------------------------------------
 *
 * 1. LONG LINES ARE SPLIT INTO CHUNKS.
 *    A BLE notification carries (MTU - 3) bytes. Until a central negotiates a
 *    larger MTU the default is 23, so only TWENTY bytes arrive and the rest of
 *    the line is discarded with no error anywhere. The hardcoded test string
 *    was ~140 characters, so it was almost certainly arriving truncated —
 *    setMTU(247) is a request, and it only takes effect if the other end
 *    agrees. This build reads the negotiated MTU back and splits accordingly,
 *    ending each line with '\n' so the receiver knows where a line stops.
 *
 * 2. setMTU IS CALLED AFTER BLEDevice::init(), NOT BEFORE.
 *    It configures the live GATT stack. Called first, there is no stack to
 *    configure yet and the request is dropped, which is the quiet way to end
 *    up stuck at 23 bytes.
 *
 * 3. ADVERTISING RESTARTS ON DISCONNECT.
 *    Without a disconnect callback the device stops advertising the moment the
 *    laptop drops, and the only way back is a reset. That is fine for a
 *    one-shot test and painful for a session of real testing.
 *
 * ---------------------------------------------------------------------------
 * WIRING — both microphones share ONE I2S bus (3 GPIOs total)
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
 *   The ONLY difference between the two modules is the L/R pin.
 *   100 nF + 10 uF from VDD to GND at EACH module, as close as you can.
 *
 * ---------------------------------------------------------------------------
 * ARDUINO IDE
 * ---------------------------------------------------------------------------
 *   Board XIAO_ESP32S3 · PSRAM = OPI PSRAM · CPU = 240 MHz
 *   USB CDC On Boot = Enabled
 *   Partition Scheme = Default with spiffs (3MB APP/1.5MB SPIFFS)
 *   Serial Monitor @ 115200
 *
 *   Put audio_model.h in this folder.
 * ==========================================================================*/

#include <driver/i2s.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

/* esp_ble_gatts_cb_param_t lives here, and BLEDevice.h does not include it.
 * __has_include so that a core version which has moved or renamed the header
 * costs one skipped optimisation rather than the whole build. */
#if __has_include(<esp_gatts_api.h>)
  #include <esp_gatts_api.h>
  #define HAVE_GATTS_PARAM 1
#else
  #define HAVE_GATTS_PARAM 0
#endif

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

/* --------------------------------------------------------------------- BLE */
#define BLE_NAME            "XIAO-ESP32S3"
#define SERVICE_UUID        "12345678-1234-1234-1234-123456789001"
#define CHARACTERISTIC_UUID "12345678-1234-1234-1234-123456789002"

#define BLE_REQUEST_MTU  247    /* a request; the central decides the rest */

/* Connection interval request, in 1.25 ms units. 6 = 7.5 ms, 12 = 15 ms.
 * One notification goes out per connection interval, so this number sets the
 * ceiling on how fast the log can stream. A central left to its own devices
 * often picks 100-500 ms, which is why lines were arriving seconds apart. */
#define BLE_CONN_MIN     6
#define BLE_CONN_MAX     12

/* Lines waiting to go out. The inference loop drops into this queue and
 * returns immediately, so a slow radio can never stall the classifier. */
#define BLE_QUEUE_LEN    8
#define BLE_LINE_MAX     200

/* Send the per-hop status line, not just events. 0 = events only, which is
 * much quieter if you are watching over a slow link. */
#define BLE_SEND_STATUS  1

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

/* --------------------------------------------------------------- BLE state */
static BLEServer         *bleServer     = nullptr;
static BLECharacteristic *bleChar       = nullptr;
static BLEAdvertising    *bleAdvertising = nullptr;
static volatile bool      bleConnected  = false;
static volatile bool      bleReady      = false;   /* stack came up cleanly */
static QueueHandle_t      ble_q         = NULL;
static volatile uint32_t  bleDropped    = 0;       /* lines the radio missed */
static volatile uint16_t  blePeerMTU    = 23;

struct BleLine { char txt[BLE_LINE_MAX]; };

static volatile uint32_t  loopPeriodMs  = 0;       /* measured, not assumed */

/* ========================================================================= */
/*  I2S — stereo, 32-bit, both slots                                          */
/* ========================================================================= */
static bool i2s_init(void)
{
    i2s_config_t cfg = {
        .mode                 = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
        .sample_rate          = AM_SR,                     /* per channel */
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
/*  BLE                                                                       */
/* ========================================================================= */
class ServerCallbacks : public BLEServerCallbacks {
#if HAVE_GATTS_PARAM
    /* The two-argument form carries the peer address, which is what
     * updateConnParams needs. Deliberately NOT marked override: if a future
     * core drops this overload, an unused method is a far better outcome than
     * a build that will not compile. The Serial line below is how you tell
     * whether it is actually being called. */
    void onConnect(BLEServer *s, esp_ble_gatts_cb_param_t *param) {
        bleConnected = true;
        Serial.println(F("[ble] central connected"));

        /* Ask for the fastest interval the spec allows. A central left alone
         * picks whatever suits its own power budget, often 100-500 ms, and one
         * notification goes out per interval. */
        s->updateConnParams(param->connect.remote_bda,
                            BLE_CONN_MIN, BLE_CONN_MAX, 0, 400);
        Serial.printf("[ble] requested %.1f-%.1f ms interval\n",
                      BLE_CONN_MIN * 1.25f, BLE_CONN_MAX * 1.25f);
    }
#endif
    /* Always present. On a core where the two-argument form above is not
     * called, this still tracks the connection and the advertised interval
     * hints are the only thing steering the timing. */
    void onConnect(BLEServer *s) override {
        bleConnected = true;
        Serial.println(F("[ble] central connected"));
    }

    void onDisconnect(BLEServer *s) override {
        bleConnected = false;
        blePeerMTU   = 23;
        Serial.println(F("[ble] central disconnected - advertising again"));
        /* Without this the device goes quiet permanently and only a reset
         * brings it back. The short delay lets the stack finish tearing the
         * old link down before it starts advertising the next one. */
        delay(300);
        BLEDevice::startAdvertising();
    }
};

/* Push a line onto the outgoing queue and return AT ONCE.
 *
 * This is the whole fix for slow logging. Notifying directly from the
 * inference loop meant the loop waited for the radio: one notification goes
 * out per connection interval, and a 140-character line at the default
 * 23-byte MTU is eight of them. At a 500 ms interval that is four seconds of
 * the classifier sitting idle, which is why lines arrived every five seconds
 * instead of four times a second.
 *
 * If the radio falls behind, lines are DROPPED rather than queued forever.
 * A log that silently lags further and further behind reality is worse than
 * one with a counted gap in it. */
static void bleNotifyLine(const char *line)
{
    if (!bleReady || !bleConnected || ble_q == NULL) return;

    BleLine item;
    strncpy(item.txt, line, BLE_LINE_MAX - 2);
    item.txt[BLE_LINE_MAX - 2] = '\0';
    strcat(item.txt, "\n");          /* terminator travels WITH the line, so
                                      * it costs no extra packet */

    if (xQueueSend(ble_q, &item, 0) != pdTRUE) bleDropped++;
}

/* Runs in its own task. Splits to whatever MTU was actually negotiated:
 * notify() sends at most (MTU - 3) bytes and discards the rest silently, so
 * reading the real value back is the difference between a whole line and its
 * first twenty characters. */
static void bleSendNow(const char *line)
{
    uint16_t mtu = 23;
    if (bleServer) {
        uint16_t peer = bleServer->getPeerMTU(bleServer->getConnId());
        if (peer >= 23) mtu = peer;
    }
    if (mtu != blePeerMTU) {
        blePeerMTU = mtu;
        Serial.printf("[ble] negotiated MTU %u -> %u bytes per packet, "
                      "%u packets per line\n",
                      (unsigned)mtu, (unsigned)(mtu - 3),
                      (unsigned)((strlen(line) + (mtu - 4)) / (mtu - 3)));
    }

    size_t chunk = mtu - 3;
    if (chunk < 1) chunk = 20;

    size_t len = strlen(line);
    for (size_t off = 0; off < len; off += chunk) {
        size_t n = len - off;
        if (n > chunk) n = chunk;
        bleChar->setValue((uint8_t *)(line + off), n);
        bleChar->notify();
    }
}

static void ble_task(void *arg)
{
    BleLine item;
    for (;;) {
        if (xQueueReceive(ble_q, &item, portMAX_DELAY) == pdTRUE) {
            if (bleConnected && bleChar) bleSendNow(item.txt);
        }
    }
}

static void ble_begin(void)
{
    BLEDevice::init(BLE_NAME);

    /* AFTER init, not before: this configures the running GATT stack. Called
     * beforehand it has nothing to act on and is quietly ignored, leaving the
     * link at 23 bytes. */
    BLEDevice::setMTU(BLE_REQUEST_MTU);

    bleServer = BLEDevice::createServer();
    bleServer->setCallbacks(new ServerCallbacks());

    BLEService *service = bleServer->createService(SERVICE_UUID);
    bleChar = service->createCharacteristic(
        CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_READ |
        BLECharacteristic::PROPERTY_NOTIFY);
    bleChar->addDescriptor(new BLE2902());
    service->start();

    bleAdvertising = BLEDevice::getAdvertising();
    bleAdvertising->addServiceUUID(SERVICE_UUID);
    bleAdvertising->setScanResponse(true);
    bleAdvertising->setMinPreferred(0x06);   /* hint: 7.5 ms */
    bleAdvertising->setMaxPreferred(0x12);   /* hint: 22.5 ms */

    ble_q = xQueueCreate(BLE_QUEUE_LEN, sizeof(BleLine));
    if (!ble_q) { Serial.println(F("FATAL: no BLE queue")); while (1) delay(1000); }
    xTaskCreatePinnedToCore(ble_task, "ble_tx", 4096, NULL, 1, NULL, 1);
    BLEDevice::startAdvertising();

    bleReady = true;
    Serial.printf("[ble] advertising as \"%s\"\n", BLE_NAME);
    Serial.printf("[ble] service %s\n", SERVICE_UUID);
    Serial.printf("[ble] notify  %s\n", CHARACTERISTIC_UUID);
    Serial.printf("[ble] asking for %.1f-%.1f ms connection interval\n",
                  BLE_CONN_MIN * 1.25f, BLE_CONN_MAX * 1.25f);
}

/* ========================================================================= */
/*  ONE formatter, used by both Serial and BLE, so the two can never drift    */
/* ========================================================================= */
static int fmtStatusLine(char *out, size_t cap,
                         uint32_t t_mfe, uint32_t t_nn,
                         float dbA, float dbB, int used)
{
    int n = snprintf(out, cap, "[MFE %3lu ms | NN %3lu ms]",
                     (unsigned long)(t_mfe/1000), (unsigned long)(t_nn/1000));
#if CHANNEL_REPORT
    n += snprintf(out + n, cap - n, " A %5.1f dB  B %5.1f dB  src:%s |",
                  dbA, dbB, used == 0 ? "A" : (used == 1 ? "B" : "mix"));
#endif
    for (int i = 0; i < AM_CLASSES; i++)
        n += snprintf(out + n, cap - n, "  %s %.2f", AM_LABELS[i], prob[i]);

    /* Loop period is the number that answers "why is this slow". If it reads
     * ~250 ms here but lines arrive on the phone seconds apart, the classifier
     * is fine and the radio is the bottleneck. If it reads 5000 ms, the
     * bottleneck is on this side. Without it, the two are indistinguishable. */
    n += snprintf(out + n, cap - n, " | %lu ms", (unsigned long)loopPeriodMs);

    if (bleDropped)
        n += snprintf(out + n, cap - n, "  [ble dropped %lu]", (unsigned long)bleDropped);
    if (overruns)
        n += snprintf(out + n, cap - n, "   [!] overruns=%lu", (unsigned long)overruns);
    return n;
}

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
    Serial.println(F("XIAO ESP32-S3 + 2x INMP441 - audio classifier + BLE log"));
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

    /* BLE before I2S: if the stack is going to fail to allocate, it should do
     * so while the log is still quiet and the failure is easy to read. */
    ble_begin();
    Serial.printf("free heap after BLE %u B\n", (unsigned)ESP.getFreeHeap());

    hop_sem = xSemaphoreCreateBinary();
    if (!hop_sem) { Serial.println(F("FATAL: no semaphore")); while (1) delay(1000); }
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

    {   static uint32_t prev = 0;
        uint32_t now = millis();
        if (prev) loopPeriodMs = now - prev;
        prev = now;
    }

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

    /* fuse: per class take the more confident microphone, then renormalise. */
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

    /* ---- one line, built once, sent to both destinations ---- */
    char line[220];
    fmtStatusLine(line, sizeof(line), t_mfe, t_nn, dbA, dbB, used);

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
    bleNotifyLine(line);
#endif

    if (stable >= 0 && stable != last_reported) {
        char ev[96];
        int n = snprintf(ev, sizeof(ev), ">>> EVENT: %s", AM_LABELS[stable]);
        if (used == 0 || used == 1)
            snprintf(ev + n, sizeof(ev) - n, "   (loudest on mic %c)",
                     used == 0 ? 'A' : 'B');
        Serial.println(ev);
        bleNotifyLine(ev);
        last_reported = stable;
    } else if (stable < 0) {
        bool any = false;
        for (int i = 0; i < SMOOTH_M; i++) if (hist[i] >= 0) any = true;
        if (!any) last_reported = -1;
    }
}
