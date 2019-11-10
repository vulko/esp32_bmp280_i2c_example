#include "stdio.h"
#include "bmp280_driver/bmp280.h"
#include "driver/i2c.h"
#include "esp_log.h"

#define DRIVER_TAG "BMP280_driver_facade"
#define BMP280_ADDR BMP280_I2C_ADDR_PRIM
#define ACK_CHECK_EN 0x01
#define ACK_CHECK_DIS 0x0
#define ACK_VAL 0x0
#define NACK_VAL 0x1
#define CONFIG_I2C_MASTER_PORT_NUM I2C_NUM_0
#define CONFIG_I2C_SLAVE_PORT_NUM I2C_NUM_1


void delay_ms(uint32_t period_ms);
int8_t i2c_reg_write(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length);
int8_t i2c_reg_read(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length);
void print_rslt(int8_t rslt);

void checkErr(const char* msg, esp_err_t val) {
    if (val != ESP_OK) {
        ESP_LOGE(DRIVER_TAG, "%s: ESP Error! %d", msg, val);
    }
}

/*!
 *  @brief Function that creates a mandatory delay required in some of the APIs such as "bmg250_soft_reset",
 *      "bmg250_set_foc", "bmg250_perform_self_test"  and so on.
 *
 *  @param[in] period_ms  : the required wait time in milliseconds.
 *  @return void.
 *
 */
void delay_ms(uint32_t period_ms)
{
    ets_delay_us(period_ms * 1000);
}

/*!
 *  @brief Function for writing the sensor's registers through I2C bus.
 *
 *  @param[in] i2c_addr : sensor I2C address.
 *  @param[in] reg_addr : Register address.
 *  @param[in] reg_data : Pointer to the data buffer whose value is to be written.
 *  @param[in] length   : No of bytes to write.
 *
 *  @return Status of execution
 *  @retval 0 -> Success
 *  @retval >0 -> Failure Info
 *
 */
int8_t i2c_reg_write(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length)
{
    ESP_LOGD(DRIVER_TAG, "Starting write...");
    esp_err_t ret = ESP_OK;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    checkErr("I2C: start", i2c_master_start(cmd));
    checkErr("I2C: write slave address", i2c_master_write_byte(cmd, (i2c_addr << 1) | I2C_MASTER_WRITE, true));
    checkErr("I2C: write register address", i2c_master_write_byte(cmd, reg_addr, true));
    checkErr("I2C: write data", i2c_master_write(cmd, reg_data, length, true));
    checkErr("I2C: stop", i2c_master_stop(cmd));
    ret = i2c_master_cmd_begin(CONFIG_I2C_MASTER_PORT_NUM, cmd, 100 / portTICK_RATE_MS);
    checkErr("I2C: begin", ret);
    i2c_cmd_link_delete(cmd);

    return ret == ESP_OK ? 0 : 1;
}

/*!
 *  @brief Function for reading the sensor's registers through I2C bus.
 *
 *  @param[in] i2c_addr : Sensor I2C address.
 *  @param[in] reg_addr : Register address.
 *  @param[out] reg_data    : Pointer to the data buffer to store the read data.
 *  @param[in] length   : No of bytes to read.
 *
 *  @return Status of execution
 *  @retval 0 -> Success
 *  @retval >0 -> Failure Info
 *
 */
int8_t i2c_reg_read(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length)
{
    ESP_LOGD(DRIVER_TAG, "Starting read...");
    esp_err_t ret = ESP_OK;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    checkErr("I2C: start",
             i2c_master_start(cmd));
    checkErr("I2C: write address",
             i2c_master_write_byte(cmd, (i2c_addr << 1) | I2C_MASTER_WRITE, true));
    checkErr("I2C: write register address",
             i2c_master_write_byte(cmd, reg_addr, true));
    // repeated start to enable read mode
    checkErr("I2C: re-start",
             i2c_master_start(cmd));
    checkErr("I2C: write address in read mode",
             i2c_master_write_byte(cmd, (i2c_addr << 1) | I2C_MASTER_READ, true));
    if (length > 1) {
        checkErr("I2C: read data bytes",
                 i2c_master_read(cmd, reg_data, length - 1, I2C_MASTER_ACK));
    }
    checkErr("I2C: read last data byte",
             i2c_master_read_byte(cmd, reg_data + length - 1, I2C_MASTER_LAST_NACK));
    checkErr("I2C: stop",
             i2c_master_stop(cmd));

    ret = i2c_master_cmd_begin(CONFIG_I2C_MASTER_PORT_NUM, cmd, 100 / portTICK_RATE_MS);
    checkErr("I2C: begin", ret);
    i2c_cmd_link_delete(cmd);

    return ret == ESP_OK ? 0 : 1;
}

/*!
 *  @brief Prints the execution status of the APIs.
 *
 *  @param[in] api_name : name of the API whose execution status has to be printed.
 *  @param[in] rslt     : error code returned by the API whose execution status has to be printed.
 *
 *  @return void.
 */
void print_rslt(int8_t rslt)
{
    if (rslt != BMP280_OK)
    {
        if (rslt == BMP280_E_NULL_PTR)
        {
            ESP_LOGE(DRIVER_TAG, "Error [%d] : Null pointer error\r\n", rslt);
        }
        else if (rslt == BMP280_E_COMM_FAIL)
        {
            ESP_LOGE(DRIVER_TAG, "Error [%d] : Bus communication failed\r\n", rslt);
        }
        else if (rslt == BMP280_E_IMPLAUS_TEMP)
        {
            ESP_LOGE(DRIVER_TAG, "Error [%d] : Invalid Temperature\r\n", rslt);
        }
        else if (rslt == BMP280_E_DEV_NOT_FOUND)
        {
            ESP_LOGE(DRIVER_TAG, "Error [%d] : Device not found\r\n", rslt);
        }
        else
        {
            /* For more error codes refer "*_defs.h" */
            ESP_LOGE(DRIVER_TAG, "Error [%d] : Unknown error code\r\n", rslt);
        }
    } else {
        ESP_LOGE(DRIVER_TAG, "    -> SUCCESS!");
    }
}

void initialize() {
    ESP_LOGE(DRIVER_TAG, "Starting initialize...");
    int8_t rslt;
    struct bmp280_dev bmp;

    /* Map the delay function pointer with the function responsible for implementing the delay */
    bmp.delay_ms = delay_ms;

    /* Assign device I2C address based on the status of SDO pin (GND for PRIMARY(0x76) & VDD for SECONDARY(0x77)) */
    bmp.dev_id = BMP280_ADDR;

    /* Select the interface mode as I2C */
    bmp.intf = BMP280_I2C_INTF;

    /* Map the I2C read & write function pointer with the functions responsible for I2C bus transfer */
    bmp.read = i2c_reg_read;
    bmp.write = i2c_reg_write;

    rslt = bmp280_init(&bmp);
    ESP_LOGE(DRIVER_TAG, "Init result:");
    print_rslt(rslt);
    
    ESP_LOGE(DRIVER_TAG, "Self test result:");
    print_rslt(bmp280_selftest(&bmp));

    struct bmp280_config conf;
    struct bmp280_uncomp_data ucomp_data;
    int32_t temp32;
    double temp;

    /* Always read the current settings before writing, especially when
     * all the configuration is not modified
     */
    ESP_LOGE(DRIVER_TAG, "Get config result:");
    print_rslt(bmp280_get_config(&conf, &bmp));

    /* configuring the temperature oversampling, filter coefficient and output data rate */
    /* Overwrite the desired settings */
    conf.filter = BMP280_FILTER_COEFF_2;

    /* Temperature oversampling set at 4x */
    conf.os_temp = BMP280_OS_4X;

    /* Pressure over sampling none (disabling pressure measurement) */
    conf.os_pres = BMP280_OS_NONE;

    /* Setting the output data rate as 1HZ(1000ms) */
    conf.odr = BMP280_ODR_1000_MS;
    
    ESP_LOGE(DRIVER_TAG, "Setting config...");
    print_rslt(bmp280_set_config(&conf, &bmp));

    /* Always set the power mode after setting the configuration */
    ESP_LOGE(DRIVER_TAG, "Setting power mode...");
    print_rslt(bmp280_set_power_mode(BMP280_NORMAL_MODE, &bmp));
    while (1) {
        /* Reading the raw data from sensor */
        ESP_LOGE(DRIVER_TAG, "Getting raw data...");
        print_rslt(bmp280_get_uncomp_data(&ucomp_data, &bmp));

        /* Getting the 32 bit compensated temperature */
        ESP_LOGE(DRIVER_TAG, "Getting compesated temperature data...");
        print_rslt(bmp280_get_comp_temp_32bit(&temp32, ucomp_data.uncomp_temp, &bmp));

        /* Getting the compensated temperature as floating point value */
        ESP_LOGE(DRIVER_TAG, "Getting double temprature value");
        print_rslt(bmp280_get_comp_temp_double(&temp, ucomp_data.uncomp_temp, &bmp));
        
        ESP_LOGE(DRIVER_TAG, "    ***    Measured temperature: %f (raw %d, compensated %d) \r\n", temp, ucomp_data.uncomp_temp, temp32);

        /* Sleep time between measurements = BMP280_ODR_1000_MS */
        bmp.delay_ms(1000);
    }
}