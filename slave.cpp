// =================================================================
// CODICE SLAVE COMPLETO - STRUTTURA A 3 TASK INDIPENDENTI (STABILE)
// Core 1: Solo IMU (Zero Jitter) | Core 0: CAN Rx + CAN Tx + Sinusoide
// =================================================================

#include <Arduino.h>
#include <M5Unified.h>
#include <driver/twai.h>
#include <math.h>
#include <driver/i2c.h>

extern "C" {
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
}

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// SLAVE_ID: Cambiare in 1, 2 o 3 prima di flashare ciascun nodo.
#define SLAVE_ID 3

#define CAN_TX_PIN GPIO_NUM_2  // G2
#define CAN_RX_PIN GPIO_NUM_1  // G1
const uint32_t SYNC_COMMAND_ID = 0x7FF;

// TEMPI DI CICLO RIPRISTINATI A QUELLI ORIGINALI AD ALTA VELOCITÀ
#define LOOP_PERIODO_MS              20   // Periodo della traiettoria dei motori (20ms)
#define VELOCITA_AMPIEZZA_GRADI_SEC  25.0f
#define VELOCITA_CENTRO_GRADI_SEC    50.0f
const float GRADI_A_PASSI = (16384.0f / 360.0f);

#define VELOCITA_MAX_NORMALE    1000
#define VELOCITA_MAX_RIENTRO     200
#define TEMPO_RIENTRO_MS        3000 

static constexpr int IMU_SDA = 45;
static constexpr int IMU_SCL = 0;

static bool home_eseguito = false;  
typedef enum { NORMALE = 0, LIBERO = 1 } StatoMotoreSlave;
static StatoMotoreSlave stato_motore_slave = NORMALE;

static SemaphoreHandle_t mutex_stato = NULL;
volatile bool start_sync = false;

static bool  onda_attiva            = true;
static bool  home_in_corso          = false;
static bool  motore_libero          = false;
static bool  motore_rientro         = false;

static float frequenza_hz           = 0.5f;
static float ampiezza_target_passi  = 0.0f;
static float sfasamento_rad         = 0.0f;
static float centro_target_passi    = 8192.0f;

static float ampiezza_attuale_passi = 0.0f;
static float centro_attuale_passi   = 8192.0f;
static float angolo_radianti        = 0.0f;

// VARIABILI GLOBALI DI SCAMBIO DATI IMU (Core 1 scrive, Core 0 legge e invia)
volatile float gyro_x = 0.0f;
volatile float gyro_y = 0.0f;
volatile float gyro_z = 0.0f;

// --------------------------------=================================
// DEFINIZIONE DEGLI STATI DELLA MACCHINA A STATI NON BLOCCANTE
// --------------------------------=================================
typedef enum {
    STATO_NORMALE,
    STATO_ATTESA_LIBERO,
    STATO_ATTESA_RIENTRO,
    STATO_ATTESA_HOME
} StatiTransizione;

static StatiTransizione stato_transizione = STATO_NORMALE;
static unsigned long timer_transizione = 0;

// Prototipi dei task su Core 0
void task_ricezione_can(void *arg);
void task_sinusoide(void *arg);
void task_trasmissione_can_imu(void *arg);

void invia_registro_servo(uint8_t registro, uint16_t valore) {
    twai_message_t msg;
    msg.identifier       = 0x000;
    msg.extd             = 0;
    msg.rtr              = 0;
    msg.data_length_code = 7;
    msg.data[0] = 0x96;
    msg.data[1] = 0x00 + SLAVE_ID;
    msg.data[2] = registro;
    msg.data[3] = 0x02;
    msg.data[4] = valore & 0xFF;
    msg.data[5] = (valore >> 8) & 0xFF;
    uint8_t sum = 0;
    for (int i = 1; i < 6; i++) sum += msg.data[i];
    msg.data[6] = (uint8_t)(0x88 - sum);
    twai_transmit(&msg, pdMS_TO_TICKS(5));
}

void inoltra_posizione_attuatore(uint16_t valore_posizione) {
    twai_message_t messaggio_tx;
    messaggio_tx.identifier       = 0x000;
    messaggio_tx.extd             = 0;
    messaggio_tx.rtr              = 0;
    messaggio_tx.data_length_code = 7;
    messaggio_tx.data[0] = 0x96;
    messaggio_tx.data[1] = 0x00 + SLAVE_ID;
    messaggio_tx.data[2] = 0x1E;  
    messaggio_tx.data[3] = 0x02;
    messaggio_tx.data[4] = valore_posizione & 0xFF;
    messaggio_tx.data[5] = (valore_posizione >> 8) & 0xFF;
    uint8_t somma_senza_header = 0;
    for (int i = 1; i < 6; i++) somma_senza_header += messaggio_tx.data[i];
    messaggio_tx.data[6] = (uint8_t)(0x88 - somma_senza_header);
    twai_transmit(&messaggio_tx, pdMS_TO_TICKS(2));
}

void inoltra_comando_power(bool motor_free_on) {
    invia_registro_servo(0x46, motor_free_on ? 0x0200 : 0x0000);
}

// =================================================================
// TASK 1 (CORE 0) - RICEZIONE CAN + INIZIALIZZAZIONE DRIVER
// =================================================================
void task_ricezione_can(void *arg) {
    // Inizializzazione del CAN sul Core 0 (Isolamento completo interrupt)
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
    g_config.rx_queue_len = 32; // Espandiamo la coda a 32 messaggi per evitare perdite di sync
    g_config.tx_queue_len = 32; 

    twai_timing_config_t  t_config = TWAI_TIMING_CONFIG_1MBITS();
    twai_filter_config_t  f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    
    if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
        twai_start();
        Serial.println("[CORE 0] Driver CAN (Coda 32) avviato con successo sul Core 0!");
    } else {
        Serial.println("[CORE 0] Errore inizializzazione hardware CAN.");
    }

    twai_message_t rxMsg;
    while (true) {
        if (twai_receive(&rxMsg, pdMS_TO_TICKS(10)) == ESP_OK) {
            if (rxMsg.identifier == SYNC_COMMAND_ID) {
                xSemaphoreTake(mutex_stato, portMAX_DELAY);
                start_sync = true;
                xSemaphoreGive(mutex_stato);
            }
            else if (rxMsg.identifier == (uint32_t)(0x100 + SLAVE_ID)) {
                xSemaphoreTake(mutex_stato, portMAX_DELAY);
                onda_attiva   = (rxMsg.data[0] & 0x01) != 0;
                home_in_corso = (rxMsg.data[0] & 0x02) != 0;
                motore_libero = (rxMsg.data[0] & 0x04) != 0;
                motore_rientro = (rxMsg.data[0] & 0x08) != 0;

                frequenza_hz          = rxMsg.data[1] / 100.0f;
                ampiezza_target_passi = (float)(rxMsg.data[2] | (rxMsg.data[3] << 8));
                sfasamento_rad        = (float)(rxMsg.data[4] | (rxMsg.data[5] << 8)) / 1000.0f;
                centro_target_passi   = (float)(rxMsg.data[6] | (rxMsg.data[7] << 8));
                xSemaphoreGive(mutex_stato);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(5)); 
    }
}

// =================================================================
// TASK 2 (CORE 0) - TRAIETTORIA MOTORI (NON BLOCCANTE)
// =================================================================
void task_sinusoide(void *arg) {
    while (!start_sync) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    const TickType_t periodo = pdMS_TO_TICKS(LOOP_PERIODO_MS); // 20ms
    TickType_t last_wake = xTaskGetTickCount();

    while (true) {
        vTaskDelayUntil(&last_wake, periodo);

        bool  l_onda_attiva, l_home_in_corso, l_motore_libero, l_motore_rientro;
        float l_frequenza_hz, l_ampiezza_target, l_sfasamento, l_centro_target;

        xSemaphoreTake(mutex_stato, portMAX_DELAY);
        l_onda_attiva      = onda_attiva;
        l_home_in_corso    = home_in_corso;
        l_motore_libero    = motore_libero;
        l_motore_rientro   = motore_rientro;
        l_frequenza_hz     = frequenza_hz;
        l_ampiezza_target  = ampiezza_target_passi;
        l_sfasamento       = sfasamento_rad;
        l_centro_target    = centro_target_passi;
        xSemaphoreGive(mutex_stato);

        unsigned long ora_corrente = millis();

        switch (stato_transizione) {
            case STATO_NORMALE:
                if (l_motore_libero && stato_motore_slave != LIBERO) {
                    invia_registro_servo(0x54, VELOCITA_MAX_RIENTRO);
                    invia_registro_servo(0x1E, 8192);
                    timer_transizione = ora_corrente + TEMPO_RIENTRO_MS;
                    stato_transizione = STATO_ATTESA_LIBERO;
                }
                else if (l_motore_rientro && stato_motore_slave == LIBERO) {
                    invia_registro_servo(0x54, VELOCITA_MAX_RIENTRO);
                    invia_registro_servo(0x1E, 8192);
                    inoltra_comando_power(false); 
                    timer_transizione = ora_corrente + TEMPO_RIENTRO_MS;
                    stato_transizione = STATO_ATTESA_RIENTRO;
                }
                else if (l_home_in_corso && !home_eseguito) {
                    invia_registro_servo(0x54, VELOCITA_MAX_RIENTRO);
                    invia_registro_servo(0x1E, 8192);
                    timer_transizione = ora_corrente + TEMPO_RIENTRO_MS;
                    stato_transizione = STATO_ATTESA_HOME;
                }
                break;

            case STATO_ATTESA_LIBERO:
                if (ora_corrente >= timer_transizione) {
                    invia_registro_servo(0x54, VELOCITA_MAX_NORMALE);
                    inoltra_comando_power(true);
                    stato_motore_slave = LIBERO;
                    angolo_radianti = 0.0f;
                    ampiezza_attuale_passi = 0.0f;
                    home_eseguito = true;
                    stato_transizione = STATO_NORMALE;
                }
                break;

            case STATO_ATTESA_RIENTRO:
                if (ora_corrente >= timer_transizione) {
                    invia_registro_servo(0x54, VELOCITA_MAX_NORMALE);
                    stato_motore_slave = NORMALE;
                    stato_transizione = STATO_NORMALE;
                }
                break;

            case STATO_ATTESA_HOME:
                if (ora_corrente >= timer_transizione) {
                    invia_registro_servo(0x54, VELOCITA_MAX_NORMALE);
                    ampiezza_attuale_passi = 0.0f;
                    angolo_radianti = 0.0f;
                    home_eseguito = true;
                    stato_transizione = STATO_NORMALE;
                }
                break;
        }

        // Calcolo traiettoria sinusoide
        if (stato_transizione == STATO_NORMALE && stato_motore_slave == NORMALE) {
            const float due_pi = 6.283185307f;
            const float step_ampiezza = (VELOCITA_AMPIEZZA_GRADI_SEC * GRADI_A_PASSI) * (LOOP_PERIODO_MS / 1000.0f);
            const float step_centro   = (VELOCITA_CENTRO_GRADI_SEC * GRADI_A_PASSI) * (LOOP_PERIODO_MS / 1000.0f);

            if (!l_home_in_corso) home_eseguito = false; 

            float diff_amp = l_ampiezza_target - ampiezza_attuale_passi;
            if (fabsf(diff_amp) > 0.01f) {
                if (diff_amp > 0.0f) {
                    ampiezza_attuale_passi += step_ampiezza;
                    if (ampiezza_attuale_passi > l_ampiezza_target) ampiezza_attuale_passi = l_ampiezza_target;
                } else {
                    ampiezza_attuale_passi -= step_ampiezza;
                    if (ampiezza_attuale_passi < l_ampiezza_target) ampiezza_attuale_passi = l_ampiezza_target;
                }
            }
            
            float diff_centro = l_centro_target - centro_attuale_passi;
            if (fabsf(diff_centro) > 0.01f) {
                if (diff_centro > 0.0f) {
                    centro_attuale_passi += step_centro;
                    if (centro_attuale_passi > l_centro_target) centro_attuale_passi = l_centro_target;
                } else {
                    centro_attuale_passi -= step_centro;
                    if (centro_attuale_passi < l_centro_target) centro_attuale_passi = l_centro_target;
                }
            }

            if (l_onda_attiva) {
                float valore_seno = 0.0f;
                if (l_onda_attiva && !l_home_in_corso && !l_motore_libero) {
                    valore_seno = sinf(angolo_radianti + l_sfasamento);
                }

                uint16_t posizione_calcolata = (uint16_t)(centro_attuale_passi + (ampiezza_attuale_passi * valore_seno));
                inoltra_posizione_attuatore(posizione_calcolata);

                angolo_radianti += due_pi * l_frequenza_hz * (LOOP_PERIODO_MS / 1000.0f);
                if (angolo_radianti >= due_pi) angolo_radianti -= due_pi;
            }
        }
    }
}

// =================================================================
// TASK 3 (CORE 0) - TRASMISSIONE DATI IMU VIA CAN (37ms)
// =================================================================
void task_trasmissione_can_imu(void *arg) {
    const TickType_t periodo = pdMS_TO_TICKS(37); // 37ms
    while (true) {
        vTaskDelay(periodo);

        if (start_sync) {
            float l_gx = gyro_x;
            float l_gy = gyro_y;
            float l_gz = gyro_z;

            int16_t gx_i = (int16_t)(l_gx * 100.0f);
            int16_t gy_i = (int16_t)(l_gy * 100.0f);
            int16_t gz_i = (int16_t)(l_gz * 100.0f);

            twai_message_t msg;
            msg.identifier       = 0x300 + SLAVE_ID;
            msg.extd             = 0; 
            msg.rtr              = 0;
            msg.data_length_code = 8;
            msg.data[0] = 0x55;
            msg.data[1] = 0x00 + SLAVE_ID;
            msg.data[2] = gx_i & 0xFF;
            msg.data[3] = (gx_i >> 8) & 0xFF;
            msg.data[4] = gy_i & 0xFF;
            msg.data[5] = (gy_i >> 8) & 0xFF;
            msg.data[6] = gz_i & 0xFF;
            msg.data[7] = (gz_i >> 8) & 0xFF;

            twai_transmit(&msg, pdMS_TO_TICKS(2));
        }
    }
}

void setup() {
    // // Inizializzazione M5Unified sul Core 1 (Identico al test di successo)
    // auto cfg = M5.config();
    // cfg.internal_imu = true;
    // cfg.external_imu = false; // Disabilita scansioni Grove esterne per M5Unified
    // M5.begin(cfg);
    // M5.Power.begin(); // Accende l'IMU interna
    // Wire.setTimeOut(200);

    Serial.begin(115200);
    delay(500);
    Serial.println();
    Serial.println("\n--- SYSTEM BOOT (ARCHITETTURA EQUILIBRATA CORRETTA) ---");
    // configurazione I2C per IMU interna (Core 1)
    M5.In_I2C.setPort(I2C_NUM_0, IMU_SDA, IMU_SCL);
    bool imuOk = M5.Imu.begin(
        &M5.In_I2C,
        m5::board_t::board_M5AtomS3R
    );
    if (!imuOk)
    {
        Serial.println("ERRORE: IMU BMI270 non trovata.");
        while (true)
        {
            delay(1000);
        }
    }
    Serial.println("IMU inizializzata correttamente.");

    mutex_stato = xSemaphoreCreateMutex();

    // Creazione pulita e separata dei compiti sul CORE 0 (Nessun interrupt o carico CAN avverrà sul Core 1)
    xTaskCreatePinnedToCore(task_ricezione_can,     "rx_can",     4096, NULL, 3, NULL, 0); // Core 0 - Priorità ALTA
    xTaskCreatePinnedToCore(task_sinusoide,         "sinusoide", 4096, NULL, 2, NULL, 0); // Core 0 - Priorità MEDIA
    xTaskCreatePinnedToCore(task_trasmissione_can_imu, "tx_imu",  4096, NULL, 2, NULL, 0); // Core 0 - Priorità MEDIA
}

// =================================================================
// LOOP PRINCIPALE (Gira sul Core 1 - PURE I2C, ZERO CONFLITTI)
// =================================================================
void loop() {
    delay(37); // 37ms stabili 
    // M5.update(); 
    if (M5.Imu.update()) {
        const auto data = M5.Imu.getImuData();
        
        gyro_x = data.gyro.x;
        gyro_y = data.gyro.y;
        gyro_z = data.gyro.z;
        
        Serial.printf("X:%.2f Y:%.2f Z:%.2f\n", gyro_x, gyro_y, gyro_z);
    } else {
        Serial.println("IMU non pronto...");
    }
}