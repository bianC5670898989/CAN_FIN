// =================================================================
// CODICE PER ATOM MASTER - INTERFACCIA SERIALE E STREAM SERVO
// =================================================================
#include <Arduino.h>
#include <driver/twai.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <M5Unified.h> 

extern "C" {
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
}

#define GRADI_A_PASSI (16384.0f / 360.0f)

#define FREQUENZA_MIN 0.1f
#define FREQUENZA_MAX 1.0f

#define CAN_TX_PIN GPIO_NUM_2
#define CAN_RX_PIN GPIO_NUM_1

#define SYNC_COMMAND_ID 0x7FF 

static const float POSIZIONE_DEFAULT = 8192.0f;

typedef struct {
    float frequenza_hz;
    float ampiezza_passi;
    float sfasamento_rad;
    float offset_gradi;

    float buf_frequenza;
    float buf_ampiezza_passi;
    float buf_sfasamento_rad;
    float buf_offset_gradi;
    bool  ha_nuovi_parametri;
} ParametriServo;

static ParametriServo parametri_slave[3] = {
    {0.5f, 0.0f, 0.0f, 0.0f,  0.5f, 0.0f, 0.0f, 0.0f, false},
    {0.5f, 0.0f, 0.0f, 0.0f,  0.5f, 0.0f, 0.0f, 0.0f, false},
    {0.5f, 0.0f, 0.0f, 0.0f,  0.5f, 0.0f, 0.0f, 0.0f, false}
};

static SemaphoreHandle_t mutex_slave[3] = { NULL, NULL, NULL };

static float offset_manuale_gradi = 0.0f;
static bool  onda_attiva          = true;
static bool  home_in_corso        = false;

typedef enum {
    MOTORE_NORMALE = 0,    
    MOTORE_LIBERO  = 1,    
    MOTORE_RIENTRO = 2     
} StatoMotore;

static StatoMotore stato_motore = MOTORE_NORMALE;
static uint8_t servo_in_modifica  = 1;

volatile bool dump_in_corso = false; 

// =================================================================
// BUFFER CIRCOLARI STREAM SERVO
// =================================================================
#define BUFFER_STREAM_LEN 2000 

typedef struct {
    uint32_t timestamp_ms;
    uint8_t  data[8];
} CampioneStream;

typedef struct {
    CampioneStream    campioni[BUFFER_STREAM_LEN];
    uint16_t          indice_scrittura;
    bool              buffer_pieno;
    SemaphoreHandle_t mutex;
} BufferCircolareServo;

static BufferCircolareServo buffer_stream[3];

// =================================================================
// NUOVO: BUFFER CIRCOLARI IMU PROVENIENTI DAGLI SLAVE
// =================================================================
#define BUFFER_IMU_LEN 1000 // 1000 campioni temporali per ognuno dei 3 nodi

typedef struct {
    float x;
    float y;
    float z;
} CampioneImu;

typedef struct {
    CampioneImu       campioni[BUFFER_IMU_LEN];
    uint16_t          indice_scrittura;
    bool              buffer_pieno;
    SemaphoreHandle_t mutex;
} BufferCircolareImu;

static BufferCircolareImu buffer_imu[3];

typedef enum {
    STATO_ATTESA_COMANDO,
    STATO_MENU_9,
    STATO_SELEZIONE_SERVO_MODIFICA,
    STATO_ATTESA_VALORE_PARAMETRI
} stato_seriale_t;

static stato_seriale_t stato_seriale = STATO_ATTESA_COMANDO;

// =================================================================
// INVII CAN
// =================================================================
void invia_configurazione_slave_can(uint8_t id) {
    uint8_t idx = id - 1;

    float f, amp, sfas, off;
    xSemaphoreTake(mutex_slave[idx], portMAX_DELAY);
    f    = parametri_slave[idx].frequenza_hz;
    amp  = parametri_slave[idx].ampiezza_passi;
    sfas = parametri_slave[idx].sfasamento_rad;
    off  = parametri_slave[idx].offset_gradi;
    xSemaphoreGive(mutex_slave[idx]);

    twai_message_t msg;
    msg.identifier = 0x100 + id; 
    msg.extd = 0;
    msg.rtr = 0;
    msg.data_length_code = 8;

    msg.data[0] = (onda_attiva ? 0x01 : 0x00) | (home_in_corso ? 0x02 : 0x00) | (stato_motore == MOTORE_LIBERO ? 0x04 : 0x00) | (stato_motore == MOTORE_RIENTRO ? 0x08 : 0x00);
    msg.data[1] = (uint8_t)(f * 100.0f);

    uint16_t amp_u = (uint16_t)amp;
    msg.data[2] = amp_u & 0xFF;
    msg.data[3] = (amp_u >> 8) & 0xFF;

    uint16_t sfas_u = (uint16_t)(sfas * 1000.0f);
    msg.data[4] = sfas_u & 0xFF;
    msg.data[5] = (sfas_u >> 8) & 0xFF;

    uint16_t centro = (uint16_t)(POSIZIONE_DEFAULT + ((off + offset_manuale_gradi) * GRADI_A_PASSI));
    msg.data[6] = centro & 0xFF;
    msg.data[7] = (centro >> 8) & 0xFF;

    twai_transmit(&msg, pdMS_TO_TICKS(2));
}

void invia_a_tutti_gli_slave(void) {
    for (int i = 0; i < 3; i++) {
        invia_configurazione_slave_can(i + 1);
    }
}

void invia_comando_sync(void) {
    twai_message_t msg;
    msg.identifier = SYNC_COMMAND_ID;
    msg.extd = 0;
    msg.rtr = 0;
    msg.data_length_code = 0; 
    
    if (twai_transmit(&msg, pdMS_TO_TICKS(50)) == ESP_OK) {
        Serial.printf("[SYNC] Segnale di avvio (0x7FF) inviato in broadcast!\n");
    } else {
        Serial.printf("[ERRORE] Invio segnale SYNC fallito.\n");
    }
}

// =================================================================
// RICEZIONE E PARSING BUFFER
// =================================================================
void salva_in_buffer_circolare(uint8_t idx_servo, twai_message_t *frame) {
    xSemaphoreTake(buffer_stream[idx_servo].mutex, portMAX_DELAY);

    uint16_t pos = buffer_stream[idx_servo].indice_scrittura;
    buffer_stream[idx_servo].campioni[pos].timestamp_ms = millis();
    memcpy(buffer_stream[idx_servo].campioni[pos].data, frame->data, 8);

    buffer_stream[idx_servo].indice_scrittura = (pos + 1) % BUFFER_STREAM_LEN;
    if (buffer_stream[idx_servo].indice_scrittura == 0) {
        buffer_stream[idx_servo].buffer_pieno = true;
    }

    xSemaphoreGive(buffer_stream[idx_servo].mutex);
}

    void salva_in_buffer_imu(uint8_t idx_servo, twai_message_t *frame) {
    if (frame->data_length_code < 8) return;
    // data[0]=header 0x55, data[1]=SLAVE_ID, data[2..7]=gx,gy,gz

    int16_t rx_x = (int16_t)(frame->data[2] | (frame->data[3] << 8));
    int16_t rx_y = (int16_t)(frame->data[4] | (frame->data[5] << 8));
    int16_t rx_z = (int16_t)(frame->data[6] | (frame->data[7] << 8));

    xSemaphoreTake(buffer_imu[idx_servo].mutex, portMAX_DELAY);
    uint16_t pos = buffer_imu[idx_servo].indice_scrittura;
    buffer_imu[idx_servo].campioni[pos].x = rx_x / 100.0f;
    buffer_imu[idx_servo].campioni[pos].y = rx_y / 100.0f;
    buffer_imu[idx_servo].campioni[pos].z = rx_z / 100.0f;
    buffer_imu[idx_servo].indice_scrittura = (pos + 1) % BUFFER_IMU_LEN;
    if (buffer_imu[idx_servo].indice_scrittura == 0)
        buffer_imu[idx_servo].buffer_pieno = true;
    xSemaphoreGive(buffer_imu[idx_servo].mutex);
}


void task_stream(void *arg) {
    twai_message_t messaggio_rx;
    while (true) {
        if (twai_receive(&messaggio_rx, pdMS_TO_TICKS(100)) == ESP_OK) {
            // 1. Controllo pacchetti Stream Servo standard (0x56)
            if (messaggio_rx.data_length_code >= 2 && messaggio_rx.data[0] == 0x56) {
                uint8_t id_servo = messaggio_rx.data[1];
                if (id_servo >= 1 && id_servo <= 3) {
                    if (!dump_in_corso) { 
                        salva_in_buffer_circolare(id_servo - 1, &messaggio_rx);
                    }
                }
            }
            // 2. NUOVO: Controllo pacchetti IMU provenienti dagli slave (Range ID: 0x301 - 0x303)
           else if (messaggio_rx.identifier >= 0x301 && messaggio_rx.identifier <= 0x303 && messaggio_rx.data_length_code >= 8 && messaggio_rx.data[0] == 0x55){
                uint8_t id_slave = messaggio_rx.identifier - 0x300;
                if (!dump_in_corso) {
                    salva_in_buffer_imu(id_slave - 1, &messaggio_rx);
                }
            }
        }
    }
}

// =================================================================
// FUNZIONI DI DUMP SERIALE (STAMPA)
// =================================================================
void stampa_dump_buffer_servo(uint8_t idx_servo) {
    xSemaphoreTake(buffer_stream[idx_servo].mutex, portMAX_DELAY);
    uint16_t n_campioni = buffer_stream[idx_servo].buffer_pieno ? BUFFER_STREAM_LEN : buffer_stream[idx_servo].indice_scrittura;
    uint16_t partenza = buffer_stream[idx_servo].buffer_pieno ? buffer_stream[idx_servo].indice_scrittura : 0;
    xSemaphoreGive(buffer_stream[idx_servo].mutex);

    Serial.printf("\n--- DUMP BUFFER SERVO %d (%d campioni) ---\n", idx_servo + 1, n_campioni);

    for (uint16_t k = 0; k < n_campioni; k++) {
        uint16_t pos = (partenza + k) % BUFFER_STREAM_LEN;

        xSemaphoreTake(buffer_stream[idx_servo].mutex, portMAX_DELAY);
        CampioneStream c = buffer_stream[idx_servo].campioni[pos]; 
        xSemaphoreGive(buffer_stream[idx_servo].mutex);

        Serial.printf("BYTES = %02X %02X %02X %02X %02X %02X %02X %02X\n", 
                    c.data[0], c.data[1], c.data[2], c.data[3],
                    c.data[4], c.data[5], c.data[6], c.data[7]);

        vTaskDelay(pdMS_TO_TICKS(2));
    }
    Serial.printf("--- FINE DUMP SERVO %d ---\n", idx_servo + 1);
}

void stampa_dump_tutti_i_servi(void) {
    Serial.printf("\n[INFO] CONGELAMENTO DATI (SNAPSHOT SIMULTANEO SERVO)...\n");
    dump_in_corso = true; 
    vTaskDelay(pdMS_TO_TICKS(10)); 

    for (uint8_t i = 0; i < 3; i++) {
        stampa_dump_buffer_servo(i);
    }
    dump_in_corso = false; 
    Serial.printf("\n[INFO] SNAPSHOT SERVO COMPLETATO. Stream riattivato.\n");
}

// NUOVO: Stampa il dump IMU in formato CSV ultra pulito
void stampa_dump_buffer_imu(uint8_t idx_servo) {
    xSemaphoreTake(buffer_imu[idx_servo].mutex, portMAX_DELAY);
    uint16_t n_campioni = buffer_imu[idx_servo].buffer_pieno ? BUFFER_IMU_LEN : buffer_imu[idx_servo].indice_scrittura;
    uint16_t partenza = buffer_imu[idx_servo].buffer_pieno ? buffer_imu[idx_servo].indice_scrittura : 0;
    xSemaphoreGive(buffer_imu[idx_servo].mutex);

    Serial.printf("\n--- DUMP BUFFER IMU SLAVE %d (%d campioni) ---\n", idx_servo + 1, n_campioni);
    Serial.println("X,Y,Z"); // Intestazione colonne CSV

    for (uint16_t k = 0; k < n_campioni; k++) {
        uint16_t pos = (partenza + k) % BUFFER_IMU_LEN;

        xSemaphoreTake(buffer_imu[idx_servo].mutex, portMAX_DELAY);
        CampioneImu c = buffer_imu[idx_servo].campioni[pos]; 
        xSemaphoreGive(buffer_imu[idx_servo].mutex);

        // Stampa stile CSV pronta per essere copiata su Excel o un file .csv
        Serial.printf("%.2f,%.2f,%.2f\n", c.x, c.y, c.z);

        vTaskDelay(pdMS_TO_TICKS(2));
    }
    Serial.printf("--- FINE DUMP IMU SLAVE %d ---\n", idx_servo + 1);
}

// NUOVO: Snapshot e dump sequenziale di tutti i buffer IMU
void stampa_dump_tutti_gli_imu(void) {
    Serial.printf("\n[INFO] CONGELAMENTO DATI IMU (SNAPSHOT SIMULTANEO IMU)...\n");
    dump_in_corso = true; 
    vTaskDelay(pdMS_TO_TICKS(10)); 

    for (uint8_t i = 0; i < 3; i++) {
        stampa_dump_buffer_imu(i);
    }
    dump_in_corso = false; 
    Serial.printf("\n[INFO] SNAPSHOT IMU COMPLETATO. Stream riattivato.\n");
}

// =================================================================
// MENU E GESTIONE COMANDI
// =================================================================
static void stampa_menu(void) {
    Serial.printf("\n--- MENU PARAMETRI MASTER (CODA DISTRIBUITA) ---\n");
    Serial.printf("0 = HOME: Comando rientro coordinato locale sugli Slave\n");
    Serial.printf("1 = SIN: Invia parametri correnti a tutti gli Slave\n");
    Serial.printf("2 = SINISTRA: Sposta offset coda di -5°\n");
    Serial.printf("3 = DESTRA: Sposta offset coda di +5°\n");
    Serial.printf("4 = SYNC: Avvia i task sinusoide in simultanea (Broadcast 0x7FF)\n");
    Serial.printf("5 = DUMP IMU: Snapshot simultaneo dei buffer IMU degli Slave (CSV)\n"); // NUOVO
    Serial.printf("6 = DUMP BUFFER: Snapshot simultaneo dei buffer stream servo\n");
    Serial.printf("7 = MOTOR FREE: Sgancia coppia (Task slave in riposo a consumo zero)\n");
    Serial.printf("8 = MOTOR LOCK: Richiesta posizione On-Demand + rientro guidato\n");
    Serial.printf("9 -> poi 1: Entra in MODIFICA parametri -> Scegli Servo\n");
    Serial.printf("9 -> poi 0: Esegue l'UPGRADE inviando i nuovi parametri via CAN\n");
    Serial.printf("------------------------------------------------------------------------\n");
}

static void gestisci_comando_principale(int comando) {
    switch (comando) {
        case 0:
            offset_manuale_gradi = 0.0f;
            home_in_corso = true;
            onda_attiva = false; 
            stato_motore = MOTORE_NORMALE;
            invia_a_tutti_gli_slave();
            Serial.printf("[INFO] HOME inviato a tutti gli slave.\n");
            break;
        case 1:
            onda_attiva = true;
            home_in_corso = false;
            stato_motore = MOTORE_NORMALE; 
            offset_manuale_gradi = 0.0f;
            invia_a_tutti_gli_slave();
            Serial.printf("[INFO] Parametri aggiornati inviati a tutti gli slave.\n");
            break;
        case 2:
            home_in_corso = false;
            stato_motore = MOTORE_NORMALE;
            offset_manuale_gradi = -5.0f;
            invia_a_tutti_gli_slave();
            Serial.printf("[INFO] Offset Coda a SINISTRA (-5°).\n");
            break;
        case 3:
            home_in_corso = false;
            stato_motore = MOTORE_NORMALE;
            offset_manuale_gradi = 5.0f;
            invia_a_tutti_gli_slave();
            Serial.printf("[INFO] Offset Coda a DESTRA (+5°).\n");
            break;
        case 4:
            invia_comando_sync();
            break;
        case 5:
            stampa_dump_tutti_gli_imu(); // NUOVO
            break;
        case 6:
            stampa_dump_tutti_i_servi(); 
            break;
        case 7:
            stato_motore = MOTORE_LIBERO;
            home_in_corso = false; 
            onda_attiva = false;
            invia_a_tutti_gli_slave();
            Serial.printf("[INFO] MOTOR FREE inviato: Coppia disattivata.\n");
            break;
        case 8:
            stato_motore = MOTORE_RIENTRO;
            invia_a_tutti_gli_slave();
            Serial.printf("[INFO] MOTOR LOCK inviato: Richiesta snapshot posizione attiva.\n");
            break;
        case 9:
            stato_seriale = STATO_MENU_9;
            Serial.printf("\n[CONFIG 9] Scegli -> (1: Personalizza Servo) | (0: Upgrade): ");
            break;
        default:
            stampa_menu();
            break;
    }
}

static void salva_buffer_parametri(const char *testo, uint8_t id_servo) {
    float f = 0.0f, amp = 0.0f, sfas = 0.0f, off = 0.0f;
    uint8_t idx = id_servo - 1;

    int letti = sscanf(testo, "%f %f %f %f", &f, &amp, &sfas, &off);
    if (letti == 4) {
        if (f < FREQUENZA_MIN) f = FREQUENZA_MIN;
        if (f > FREQUENZA_MAX) f = FREQUENZA_MAX;

        xSemaphoreTake(mutex_slave[idx], portMAX_DELAY);
        parametri_slave[idx].buf_frequenza       = f;
        parametri_slave[idx].buf_ampiezza_passi  = amp * GRADI_A_PASSI;
        parametri_slave[idx].buf_sfasamento_rad  = sfas;
        parametri_slave[idx].buf_offset_gradi    = off;
        parametri_slave[idx].ha_nuovi_parametri  = true;
        xSemaphoreGive(mutex_slave[idx]);

        Serial.printf("\n[BUFFER SALVATO] Servo %d - 4 parametri pronti per Upgrade.\n", id_servo);
        return;
    }

    letti = sscanf(testo, "%f", &f);
    if (letti == 1) {
        if (f < FREQUENZA_MIN) f = FREQUENZA_MIN;
        if (f > FREQUENZA_MAX) f = FREQUENZA_MAX;

        xSemaphoreTake(mutex_slave[idx], portMAX_DELAY);
        parametri_slave[idx].buf_frequenza       = f;
        parametri_slave[idx].ha_nuovi_parametri  = true;
        xSemaphoreGive(mutex_slave[idx]);

        Serial.printf("\n[BUFFER SALVATO] Servo %d - Frequenza %.2f Hz salvata.\n", id_servo, f);
        return;
    }

    Serial.printf("\n[ERRORE] Formato non valido.\n");
}

static char riga_buffer[64];
static int  riga_len = 0;

void gestisci_seriale(void) {
    while (Serial.available() > 0) {
        int c = Serial.read();
        if (c < 0) return;
        if (c == '\n' || c == '\r') {
            if (riga_len > 0) {
                riga_buffer[riga_len] = '\0';
                switch (stato_seriale) {
                    case STATO_ATTESA_COMANDO:
                        if (riga_buffer[0] >= '0' && riga_buffer[0] <= '9') gestisci_comando_principale(atoi(riga_buffer));
                        else stampa_menu(); 
                        break;

                    case STATO_MENU_9: {
                        int scelta = atoi(riga_buffer);
                        if (scelta == 1) {
                            stato_seriale = STATO_SELEZIONE_SERVO_MODIFICA;
                            Serial.printf("\nScegli Servo (1, 2, 3): ");
                        }
                        else if (scelta == 0) {
                            int cont = 0;   
                            for (int i = 0; i < 3; i++) {
                                xSemaphoreTake(mutex_slave[i], portMAX_DELAY);
                                if (parametri_slave[i].ha_nuovi_parametri) {
                                    parametri_slave[i].frequenza_hz      = parametri_slave[i].buf_frequenza;
                                    parametri_slave[i].ampiezza_passi    = parametri_slave[i].buf_ampiezza_passi;
                                    parametri_slave[i].sfasamento_rad    = parametri_slave[i].buf_sfasamento_rad;
                                    parametri_slave[i].offset_gradi      = parametri_slave[i].buf_offset_gradi;
                                    parametri_slave[i].ha_nuovi_parametri = false;
                                    cont++;
                                }
                                xSemaphoreGive(mutex_slave[i]);
                            }
                            home_in_corso = false;
                            onda_attiva = true;
                            stato_motore = MOTORE_NORMALE;

                            invia_a_tutti_gli_slave();

                            Serial.printf("\n[UPGRADE] Parametri aggiornati e inviati per %d segmenti.\n", cont);
                            stato_seriale = STATO_ATTESA_COMANDO;
                            stampa_menu();
                        } else {
                            stato_seriale = STATO_ATTESA_COMANDO;
                            stampa_menu();
                        }
                        break;
                    }

                    case STATO_SELEZIONE_SERVO_MODIFICA: {
                        int id_scelto = atoi(riga_buffer);
                        if (id_scelto >= 1 && id_scelto <= 3) {
                            servo_in_modifica = id_scelto;
                            stato_seriale = STATO_ATTESA_VALORE_PARAMETRI;
                            Serial.printf("\nDigita 4 valori <freq_hz> <ampiezza_gradi> <sfas_rad> <offset_gradi>\noppure <freq_hz>:\nInput: ");
                        } else {
                            stato_seriale = STATO_ATTESA_COMANDO;
                            stampa_menu();
                        }
                        break;
                    }

                    case STATO_ATTESA_VALORE_PARAMETRI:  
                        salva_buffer_parametri(riga_buffer, servo_in_modifica);
                        stato_seriale = STATO_ATTESA_COMANDO;
                        stampa_menu();
                        break;
                }
                riga_len = 0;
            }
            return;
        }
        if (riga_len < (int)sizeof(riga_buffer) - 1) riga_buffer[riga_len++] = (char)c;
    }
}

void task_seriale(void *arg) {
    while (true) {
        gestisci_seriale();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void setup() {
    Serial.begin(115200);
    delay(1000);

    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
    twai_timing_config_t  t_config = TWAI_TIMING_CONFIG_1MBITS();
    twai_filter_config_t  f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    delay(4000);
    if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
        twai_start();
        Serial.println("\n========================================================");
        Serial.println("[OK] MASTER ATOM ONLINE - Sistema Inizializzato.");
        Serial.println("PREMI '4' PER IL SYNC INIZIALE DEI NODI SLAVE.");
        Serial.println("========================================================\n");
    } else {
        Serial.println("[ERRORE] Driver CAN non installato.");
        while (1) { delay(1000); }
    }

    stampa_menu();

    for (int i = 0; i < 3; i++) {
        mutex_slave[i] = xSemaphoreCreateMutex();
        
        buffer_stream[i].mutex = xSemaphoreCreateMutex();
        buffer_stream[i].indice_scrittura = 0;
        buffer_stream[i].buffer_pieno = false;

        // NUOVO: Inizializzazione dei mutex e indici per i buffer IMU
        buffer_imu[i].mutex = xSemaphoreCreateMutex();
        buffer_imu[i].indice_scrittura = 0;
        buffer_imu[i].buffer_pieno = false;
    }

    xTaskCreate(task_seriale, "seriale", 4096, NULL, tskIDLE_PRIORITY + 2, NULL);
    xTaskCreate(task_stream, "stream", 4096, NULL, tskIDLE_PRIORITY + 2, NULL);
}

void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000));
}