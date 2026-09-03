#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/i2c.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"

#include "esp_log.h"
#include "esp_err.h"

#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"

// =====================================================
// Definições do MPU6050
// =====================================================

#define I2C_MASTER_FREQ_HZ  100000
#define I2C_MASTER_NUM      I2C_NUM_0

#define MPU6050_ADDR        0x68
#define MPU6050_WHO_AM_I_REG 0x75
#define MPU6050_PWR_MGMT_1  0x6B
#define MPU6050_ACCEL_XOUT_H 0x3B

#define ACCEL_SCALE         16384.0f

typedef struct {
    float x;
    float y;
    float z;
} AccelerationData;

// =====================================================
// Definições do SSD1306
// =====================================================

#define SSD1306_I2C_ADDRESS 0x3C
#define SSD1306_WIDTH       128
#define SSD1306_HEIGHT      64

// =====================================================
// Definições do cartão SD
// =====================================================

#define PIN_NUM_MISO 19
#define PIN_NUM_MOSI 23
#define PIN_NUM_CLK  18
#define PIN_NUM_CS   5

static const char *TAG = "SISTEMA";

// =====================================================
// Função auxiliar para escrever em registrador I2C
// =====================================================

esp_err_t i2c_write_register(
    uint8_t device_address,
    uint8_t register_address,
    uint8_t value
) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    if (cmd == NULL) {
        return ESP_ERR_NO_MEM;
    }

    i2c_master_start(cmd);

    i2c_master_write_byte(
        cmd,
        (device_address << 1) | I2C_MASTER_WRITE,
        true
    );

    i2c_master_write_byte(
        cmd,
        register_address,
        true
    );

    i2c_master_write_byte(
        cmd,
        value,
        true
    );

    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(
        I2C_MASTER_NUM,
        cmd,
        pdMS_TO_TICKS(1000)
    );

    i2c_cmd_link_delete(cmd);

    return ret;
}

// =====================================================
// Inicialização do MPU6050
// =====================================================

esp_err_t imu_init(
    uint8_t devAddr,
    gpio_num_t sda_pin,
    gpio_num_t scl_pin
) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = sda_pin,
        .scl_io_num = scl_pin,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
        .clk_flags = 0
    };

    esp_err_t ret = i2c_param_config(
        I2C_MASTER_NUM,
        &conf
    );

    if (ret != ESP_OK) {
        ESP_LOGE(
            TAG,
            "Erro ao configurar I2C: 0x%X",
            ret
        );

        return ret;
    }

    ret = i2c_driver_install(
        I2C_MASTER_NUM,
        I2C_MODE_MASTER,
        0,
        0,
        0
    );

    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(
            TAG,
            "Erro ao instalar driver I2C: 0x%X",
            ret
        );

        return ret;
    }

    /*
     * Retira o MPU6050 do modo de suspensão.
     * Sem essa configuração, alguns módulos não iniciam corretamente.
     */
    ret = i2c_write_register(
        devAddr,
        MPU6050_PWR_MGMT_1,
        0x00
    );

    if (ret != ESP_OK) {
        ESP_LOGE(
            TAG,
            "Erro ao ativar MPU6050: 0x%X",
            ret
        );

        return ret;
    }

    vTaskDelay(pdMS_TO_TICKS(100));

    uint8_t who_am_i = 0;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    if (cmd == NULL) {
        return ESP_ERR_NO_MEM;
    }

    i2c_master_start(cmd);

    i2c_master_write_byte(
        cmd,
        (devAddr << 1) | I2C_MASTER_WRITE,
        true
    );

    i2c_master_write_byte(
        cmd,
        MPU6050_WHO_AM_I_REG,
        true
    );

    i2c_master_start(cmd);

    i2c_master_write_byte(
        cmd,
        (devAddr << 1) | I2C_MASTER_READ,
        true
    );

    i2c_master_read_byte(
        cmd,
        &who_am_i,
        I2C_MASTER_LAST_NACK
    );

    i2c_master_stop(cmd);

    ret = i2c_master_cmd_begin(
        I2C_MASTER_NUM,
        cmd,
        pdMS_TO_TICKS(1000)
    );

    i2c_cmd_link_delete(cmd);

    if (ret == ESP_OK && who_am_i == 0x68) {
        return ESP_OK;
    }

    ESP_LOGE(
        TAG,
        "MPU6050 nao encontrado. WHO_AM_I = 0x%02X",
        who_am_i
    );

    return ESP_ERR_NOT_FOUND;
}

// =====================================================
// Leitura da aceleração do MPU6050
// =====================================================

esp_err_t imu_get_acceleration_data(
    AccelerationData *data
) {
    if (data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t accel_data[6];

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    if (cmd == NULL) {
        return ESP_ERR_NO_MEM;
    }

    i2c_master_start(cmd);

    i2c_master_write_byte(
        cmd,
        (MPU6050_ADDR << 1) | I2C_MASTER_WRITE,
        true
    );

    i2c_master_write_byte(
        cmd,
        MPU6050_ACCEL_XOUT_H,
        true
    );

    i2c_master_start(cmd);

    i2c_master_write_byte(
        cmd,
        (MPU6050_ADDR << 1) | I2C_MASTER_READ,
        true
    );

    for (int i = 0; i < 5; i++) {
        i2c_master_read_byte(
            cmd,
            &accel_data[i],
            I2C_MASTER_ACK
        );
    }

    i2c_master_read_byte(
        cmd,
        &accel_data[5],
        I2C_MASTER_LAST_NACK
    );

    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(
        I2C_MASTER_NUM,
        cmd,
        pdMS_TO_TICKS(1000)
    );

    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) {
        return ret;
    }

    int16_t raw_x =
        (int16_t)((accel_data[0] << 8) | accel_data[1]);

    int16_t raw_y =
        (int16_t)((accel_data[2] << 8) | accel_data[3]);

    int16_t raw_z =
        (int16_t)((accel_data[4] << 8) | accel_data[5]);

    data->x = raw_x / ACCEL_SCALE;
    data->y = raw_y / ACCEL_SCALE;
    data->z = raw_z / ACCEL_SCALE;

    return ESP_OK;
}

// =====================================================
// Funções do SSD1306
// =====================================================

void ssd1306_send_command(uint8_t command) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    if (cmd == NULL) {
        return;
    }

    i2c_master_start(cmd);

    i2c_master_write_byte(
        cmd,
        (SSD1306_I2C_ADDRESS << 1) | I2C_MASTER_WRITE,
        true
    );

    i2c_master_write_byte(cmd, 0x00, true);
    i2c_master_write_byte(cmd, command, true);

    i2c_master_stop(cmd);

    i2c_master_cmd_begin(
        I2C_MASTER_NUM,
        cmd,
        pdMS_TO_TICKS(1000)
    );

    i2c_cmd_link_delete(cmd);
}

void ssd1306_send_data(uint8_t data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    if (cmd == NULL) {
        return;
    }

    i2c_master_start(cmd);

    i2c_master_write_byte(
        cmd,
        (SSD1306_I2C_ADDRESS << 1) | I2C_MASTER_WRITE,
        true
    );

    i2c_master_write_byte(cmd, 0x40, true);
    i2c_master_write_byte(cmd, data, true);

    i2c_master_stop(cmd);

    i2c_master_cmd_begin(
        I2C_MASTER_NUM,
        cmd,
        pdMS_TO_TICKS(1000)
    );

    i2c_cmd_link_delete(cmd);
}

void ssd1306_init(void) {
    ssd1306_send_command(0xAE);

    ssd1306_send_command(0xD5);
    ssd1306_send_command(0x80);

    ssd1306_send_command(0xA8);
    ssd1306_send_command(0x3F);

    ssd1306_send_command(0xD3);
    ssd1306_send_command(0x00);

    ssd1306_send_command(0x40);

    ssd1306_send_command(0x8D);
    ssd1306_send_command(0x14);

    ssd1306_send_command(0x20);
    ssd1306_send_command(0x00);

    ssd1306_send_command(0xA1);
    ssd1306_send_command(0xC8);

    ssd1306_send_command(0xDA);
    ssd1306_send_command(0x12);

    ssd1306_send_command(0x81);
    ssd1306_send_command(0xCF);

    ssd1306_send_command(0xD9);
    ssd1306_send_command(0xF1);

    ssd1306_send_command(0xDB);
    ssd1306_send_command(0x40);

    ssd1306_send_command(0xA4);
    ssd1306_send_command(0xA6);

    ssd1306_send_command(0xAF);

    vTaskDelay(pdMS_TO_TICKS(100));
}

void ssd1306_clear(void) {
    for (uint8_t page = 0; page < 8; page++) {
        ssd1306_send_command(0xB0 + page);
        ssd1306_send_command(0x00);
        ssd1306_send_command(0x10);

        for (uint8_t col = 0; col < 128; col++) {
            ssd1306_send_data(0x00);
        }
    }
}

// =====================================================
// Fonte 5x7
// =====================================================

const uint8_t font5x7[36][5] = {
    {0x7E, 0x81, 0xA5, 0x81, 0x7E},
    {0x00, 0x82, 0xFF, 0x80, 0x00},
    {0xC2, 0xA1, 0x91, 0x89, 0x86},
    {0x42, 0x81, 0x89, 0x89, 0x76},
    {0x30, 0x28, 0x24, 0xFF, 0x20},
    {0x4F, 0x89, 0x89, 0x89, 0x71},
    {0x7E, 0x89, 0x89, 0x89, 0x72},
    {0x01, 0xE1, 0x19, 0x05, 0x03},
    {0x76, 0x89, 0x89, 0x89, 0x76},
    {0x46, 0x89, 0x89, 0x89, 0x7E},

    {0x20, 0x54, 0x54, 0x54, 0x78},
    {0x7F, 0x48, 0x44, 0x44, 0x38},
    {0x38, 0x44, 0x44, 0x44, 0x20},
    {0x38, 0x44, 0x44, 0x48, 0x7F},
    {0x38, 0x54, 0x54, 0x54, 0x18},
    {0x08, 0x7E, 0x09, 0x01, 0x02},
    {0x0C, 0x52, 0x52, 0x52, 0x3E},
    {0x7F, 0x08, 0x04, 0x04, 0x78},
    {0x00, 0x44, 0x7D, 0x40, 0x00},
    {0x20, 0x40, 0x40, 0x3D, 0x00},
    {0x7F, 0x10, 0x28, 0x44, 0x00},
    {0x00, 0x41, 0x7F, 0x40, 0x00},
    {0x7C, 0x04, 0x18, 0x04, 0x78},
    {0x7C, 0x08, 0x04, 0x04, 0x78},
    {0x38, 0x44, 0x44, 0x44, 0x38},
    {0x7C, 0x14, 0x14, 0x14, 0x08},
    {0x08, 0x14, 0x14, 0x18, 0x7C},
    {0x7C, 0x08, 0x04, 0x04, 0x08},
    {0x48, 0x54, 0x54, 0x54, 0x20},
    {0x04, 0x3F, 0x44, 0x40, 0x20},
    {0x3C, 0x40, 0x40, 0x20, 0x7C},
    {0x1C, 0x20, 0x40, 0x20, 0x1C},
    {0x3C, 0x40, 0x30, 0x40, 0x3C},
    {0x44, 0x28, 0x10, 0x28, 0x44},
    {0x0C, 0x50, 0x50, 0x50, 0x3C},
    {0x44, 0x64, 0x54, 0x4C, 0x44}
};

void ssd1306_draw_char(
    char c,
    uint8_t x,
    uint8_t y
) {
    uint8_t index = 255;
    uint8_t custom[5] = {0};

    if (c >= '0' && c <= '9') {
        index = c - '0';
    } else if (c >= 'a' && c <= 'z') {
        index = c - 'a' + 10;
    } else if (c == '.') {
        custom[3] = 0x60;
        custom[4] = 0x60;
    } else if (c == '-') {
        custom[0] = 0x08;
        custom[1] = 0x08;
        custom[2] = 0x08;
        custom[3] = 0x08;
        custom[4] = 0x08;
    } else if (c == ' ') {
        // Caractere vazio
    } else {
        return;
    }

    ssd1306_send_command(0xB0 + y);
    ssd1306_send_command(0x00 + (x & 0x0F));
    ssd1306_send_command(0x10 + ((x >> 4) & 0x0F));

    for (uint8_t i = 0; i < 5; i++) {
        if (index != 255) {
            ssd1306_send_data(font5x7[index][i]);
        } else {
            ssd1306_send_data(custom[i]);
        }
    }

    ssd1306_send_data(0x00);
}

void ssd1306_draw_string(
    const char *str,
    uint8_t x,
    uint8_t y
) {
    while (*str != '\0') {
        if (x + 6 > SSD1306_WIDTH) {
            x = 0;
            y++;

            if (y >= 8) {
                return;
            }
        }

        ssd1306_draw_char(*str, x, y);

        x += 6;
        str++;
    }
}

// =====================================================
// Inicialização do cartão SD
// =====================================================
bool sd_card_init(sdmmc_card_t **card) {
    const char mount_point[] = "/sdcard";

    // MUDANÇA 1: false na formatação. O Wokwi já entrega o cartão formatado!
    // Se deixar true, ele tenta formatar o disco virtual, o que causa o Timeout (0x107).
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false, 
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };

    // MUDANÇA 2: Forçar o estado "Desligado" (HIGH) no CS antes do barramento iniciar.
    // Assim o cartão SD não se assusta com ruídos na inicialização.
    gpio_set_direction(PIN_NUM_CS, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_NUM_CS, 1);
    
    // Garantir que os pinos de dados não leiam ruído fantasma
    gpio_set_pull_mode(PIN_NUM_MISO, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(PIN_NUM_MOSI, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(PIN_NUM_CLK,  GPIO_PULLUP_ONLY);

    vTaskDelay(pdMS_TO_TICKS(10)); // Pausa rápida para o simulador processar os pull-ups

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = SPI2_HOST; // Usando o canal nativo padrão do ESP-IDF

    spi_bus_config_t bus_cfg = {
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = PIN_NUM_MISO,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000
    };

    printf("Inicializando barramento SPI...\n");

    esp_err_t ret_spi = spi_bus_initialize(host.slot, &bus_cfg, SDSPI_DEFAULT_DMA);
    if (ret_spi != ESP_OK && ret_spi != ESP_ERR_INVALID_STATE) {
        printf("ERRO: Falha ao inicializar SPI: 0x%X\n", ret_spi);
        return false;
    }

    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = PIN_NUM_CS;
    slot_config.host_id = host.slot;

    printf("Montando sistema de arquivos do SD Card...\n");

    esp_err_t ret_sd = esp_vfs_fat_sdspi_mount(mount_point, &host, &slot_config, &mount_config, card);

    if (ret_sd != ESP_OK) {
        printf("ERRO: Falha ao montar SD Card (0x%X).\n", ret_sd);
        spi_bus_free(host.slot);
        return false;
    }

    printf("SD Card montado com sucesso e pronto para gravar!\n");
    sdmmc_card_print_info(stdout, *card);
    
    return true;
}
// =====================================================
// Função principal
// =====================================================

void app_main(void) {
    AccelerationData accelData;

    char buffer[32];

    int loop_counter = 0;

    bool sd_ok = false;

    sdmmc_card_t *card = NULL;

    // -------------------------------------------------
    // Inicialização do MPU6050
    // -------------------------------------------------

    printf("Inicializando MPU6050...\n");

    esp_err_t ret_imu = imu_init(
        MPU6050_ADDR,
        GPIO_NUM_21,
        GPIO_NUM_22
    );

    if (ret_imu != ESP_OK) {
        printf(
            "ERRO: Falha ao inicializar o MPU6050: 0x%X\n",
            ret_imu
        );
    } else {
        printf(
            "MPU6050 inicializado com sucesso!\n"
        );
    }

    // -------------------------------------------------
    // Inicialização do display OLED
    // -------------------------------------------------

    printf("Inicializando display OLED...\n");

    ssd1306_init();
    ssd1306_clear();

    // -------------------------------------------------
    // Inicialização do cartão SD
    // -------------------------------------------------

    printf("Inicializando SD Card...\n");

    vTaskDelay(pdMS_TO_TICKS(1000));

    sd_ok = sd_card_init(&card);

    if (!sd_ok) {
        printf(
            "ERRO: Falha ao montar o SD Card.\n"
            "O programa continuara sem gravar no cartao.\n"
        );
    }

    // -------------------------------------------------
    // Loop principal
    // -------------------------------------------------

    while (1) {
        esp_err_t ret_accel =
            imu_get_acceleration_data(&accelData);

        if (ret_accel == ESP_OK) {
            printf(
                "Sensor OK - x: %.2f g | y: %.2f g | z: %.2f g\n",
                accelData.x,
                accelData.y,
                accelData.z
            );

            // -----------------------------------------
            // Atualiza o OLED
            // -----------------------------------------

            ssd1306_clear();

            snprintf(
                buffer,
                sizeof(buffer),
                "x: %.2f g",
                accelData.x
            );

            ssd1306_draw_string(
                buffer,
                10,
                1
            );

            snprintf(
                buffer,
                sizeof(buffer),
                "y: %.2f g",
                accelData.y
            );

            ssd1306_draw_string(
                buffer,
                10,
                3
            );

            snprintf(
                buffer,
                sizeof(buffer),
                "z: %.2f g",
                accelData.z
            );

            ssd1306_draw_string(
                buffer,
                10,
                5
            );

            // -----------------------------------------
            // Grava no SD a cada aproximadamente 1 s
            // -----------------------------------------

            if ((loop_counter % 2 == 0) && sd_ok) {
                FILE *file = fopen(
                    "/sdcard/datalog.txt",
                    "a"
                );

                if (file == NULL) {
                    printf(
                        "ERRO: Falha ao abrir "
                        "/sdcard/datalog.txt\n"
                    );
                } else {
                    fprintf(
                        file,
                        "x: %.2f, y: %.2f, z: %.2f\n",
                        accelData.x,
                        accelData.y,
                        accelData.z
                    );

                    fflush(file);
                    fclose(file);

                    printf(
                        "SD OK: Dados gravados com sucesso "
                        "no cartao.\n"
                    );
                }
            }
        } else {
            printf(
                "ERRO: Falha na leitura do sensor MPU6050: "
                "0x%X\n",
                ret_accel
            );
        }

        loop_counter++;

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
