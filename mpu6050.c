#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "stm_log.h"
#include "include/mpu6050.h"

#define MPU6050_SELF_TEST_X         0x0D        /*!< SELF TEST REGISTERS */
#define MPU6050_SELF_TEST_Y         0x0E
#define MPU6050_SELF_TEST_Z         0x0F
#define MPU6050_SELF_TEST_A         0x10
#define MPU6050_SMPLRT_DIV          0x19        /*!< Sample rate diveider */
#define MPU6050_CONFIG              0x1A        /*!< Configuration */
#define MPU6050_GYRO_CONFIG         0x1B        /*!< Gyroscope configuration */
#define MPU6050_ACCEL_CONFIG        0x1C        /*!< Accelerometer configuration */
#define MPU6050_FIFO_EN             0x23        /*!< FIFO enable */
#define MPU6050_I2C_MST_CTRL        0x24        /*!< I2C master control */
#define MPU6050_I2C_SLV0_ADDR       0x25        /*!< I2C slave 0 control */
#define MPU6050_I2C_SLV0_REG        0x26
#define MPU6050_I2C_SLV0_CTRL       0x27
#define MPU6050_I2C_SLV1_ADDR       0x28        /*!< I2C slave 1 control  */
#define MPU6050_I2C_SLV1_REG        0x29
#define MPU6050_I2C_SLV1_CTRL       0x2A
#define MPU6050_I2C_SLV2_ADDR       0x2B        /*!< I2C slave 2 control */
#define MPU6050_I2C_SLV2_REG        0x2C
#define MPU6050_I2C_SLV2_CTRL       0x2D
#define MPU6050_I2C_SLV3_ADDR       0x2E        /*!< I2C slave 3 control */
#define MPU6050_I2C_SLV3_REG        0x2F
#define MPU6050_I2C_SLV3_CTRL       0x30
#define MPU6050_I2C_SLV4_ADDR       0x31        /*!< I2C slave 4 control */
#define MPU6050_I2C_SLV4_REG        0x32
#define MPU6050_I2C_SLV4_DO         0x33
#define MPU6050_I2C_SLV4_CTRL       0x34
#define MPU6050_I2C_SLV4_DI         0x35
#define MPU6050_I2C_MST_STATUS      0x36        /*!< I2C master status */
#define MPU6050_INT_PIN_CFG         0x37        /*!< I2C interrupt pin/bypass enable configuration */
#define MPU6050_INT_ENABLE          0x38        /*!< Interrupt enable */
#define MPU6050_INT_STATUS          0x3A        /*!< Interrupt status */
#define MPU6050_ACCEL_XOUT_H        0x3B        /*!< Accelerometer measurements */
#define MPU6050_ACCEL_XOUT_L        0x3C
#define MPU6050_ACCEL_YOUT_H        0x3D
#define MPU6050_ACCEL_YOUT_L        0x3E
#define MPU6050_ACCEL_ZOUT_H        0x3F
#define MPU6050_ACCEL_ZOUT_L        0x40
#define MPU6050_TEMP_OUT_H          0x41        /*!< Temperature measurements */
#define MPU6050_TEMP_OUT_L          0x42
#define MPU6050_GYRO_XOUT_H         0x43        /*!< Gyroscope measurements */
#define MPU6050_GYRO_XOUT_L         0x44
#define MPU6050_GYRO_YOUT_H         0x45
#define MPU6050_GYRO_YOUT_L         0x46
#define MPU6050_GYRO_ZOUT_H         0x47
#define MPU6050_GYRO_ZOUT_L         0x48
#define MPU6050_EXT_SENS_DATA_00    0x49        /*!< External sensor data */
#define MPU6050_EXT_SENS_DATA_01    0x4A
#define MPU6050_EXT_SENS_DATA_02    0x4B
#define MPU6050_EXT_SENS_DATA_03    0x4C
#define MPU6050_EXT_SENS_DATA_04    0x4D
#define MPU6050_EXT_SENS_DATA_05    0x4E
#define MPU6050_EXT_SENS_DATA_06    0x4F
#define MPU6050_EXT_SENS_DATA_07    0x50
#define MPU6050_EXT_SENS_DATA_08    0x51
#define MPU6050_EXT_SENS_DATA_09    0x52
#define MPU6050_EXT_SENS_DATA_10    0x53
#define MPU6050_EXT_SENS_DATA_11    0x54
#define MPU6050_EXT_SENS_DATA_12    0x55
#define MPU6050_EXT_SENS_DATA_13    0x56
#define MPU6050_EXT_SENS_DATA_14    0x57
#define MPU6050_EXT_SENS_DATA_15    0x58
#define MPU6050_EXT_SENS_DATA_16    0x59
#define MPU6050_EXT_SENS_DATA_17    0x5A
#define MPU6050_EXT_SENS_DATA_18    0x5B
#define MPU6050_EXT_SENS_DATA_19    0x5C
#define MPU6050_EXT_SENS_DATA_20    0x5D
#define MPU6050_EXT_SENS_DATA_21    0x5E
#define MPU6050_EXT_SENS_DATA_22    0x5F
#define MPU6050_EXT_SENS_DATA_23    0x60
#define MPU6050_I2C_SLV0_D0         0x63        /*!< I2C slave 0 data out */
#define MPU6050_I2C_SLV1_D0         0x64        /*!< I2C slave 1 data out */
#define MPU6050_I2C_SLV2_D0         0x65        /*!< I2C slave 2 data out */
#define MPU6050_I2C_SLV3_D0         0x66        /*!< I2C slave 3 data out */
#define MPU6050_I2C_MST_DELAY_CTRL  0x67        /*!< I2C master delay control */
#define MPU6050_SIGNAL_PATH_RESET   0x68        /*!< Signal path reset */
#define MPU6050_USER_CTRL           0x6A        /*!< User control */
#define MPU6050_PWR_MGMT_1          0x6B        /*!< Power management 1 */
#define MPU6050_PWR_MGMT_2          0x6C        /*!< Power management 2 */
#define MPU6050_FIFO_COUNTH         0x72        /*!< FIFO counter registers */
#define MPU6050_FIRO_COUNTL         0x73
#define MPU6050_FIRO_R_W            0x74        /*!< FIFO read write */
#define MPU6050_WHO_AM_I            0x75        /*!< Who am I */
#define MPU6050_ADDR                (0x68<<1)   /*!< MPU6050 Address */

#define TIMEOUT_MS_DEFAULT          100         /*!< Default MPU6050 I2C communiation timeout */
#define BUFFER_CALIB_DEFAULT        1000        /*!< Default the number of sample data when calibrate */

#define MPU6050_INIT_ERR_STR        "mpu6050 init error"
#define MPU6050_MALLOC_ERR_STR      "mpu6050 malloc error"
#define MPU6050_TRANS_ERR_STR       "mpu6050 write registers error"
#define MPU6050_REC_ERR_STR         "mpu6050 read registers error"
#define MPU6050_GET_DATA_ERR_STR    "mpu6050 get data error"
#define MPU6050_SET_BIAS_ERR_STR    "mpu6050 set bias error"
#define MPU6050_GET_BIAS_ERR_STR    "mpu6050 get bias error"
#define mutex_lock(x)               while (xSemaphoreTake(x, portMAX_DELAY) != pdPASS)
#define mutex_unlock(x)             xSemaphoreGive(x)
#define mutex_create()              xSemaphoreCreateMutex()
#define mutex_destroy(x)            vQueueDelete(x)

static const char* MPU6050_TAG = "MPU6050";
#define MPU6050_CHECK(a, str, action)  if(!(a)) {                                           \
        STM_LOGE(MPU6050_TAG,"%s:%d (%s):%s", __FILE__, __LINE__, __FUNCTION__, str);       \
        action;                                                                             \
        }

typedef stm_err_t (*read_func)(mpu6050_hardware_info_t hw_info, uint8_t reg_addr, uint8_t *buf, uint16_t len, uint32_t timeout_ms);
typedef stm_err_t (*write_func)(mpu6050_hardware_info_t hw_info, uint8_t reg_addr, uint8_t *buf, uint16_t len, uint32_t timeout_ms);

typedef struct mpu6050 {
    mpu6050_clksel_t        clksel;                 /*!< MPU6050 clock source */
    mpu6050_dlpf_cfg_t      dlpf_cfg;               /*!< MPU6050 digital low pass filter (DLPF) */
    mpu6050_sleep_mode_t    sleep_mode;             /*!< MPU6050 sleep mode */
    mpu6050_afs_sel_t       afs_sel;                /*!< MPU6050 accelerometer full scale range */
    mpu6050_fs_sel_t        fs_sel;                 /*!< MPU6050 gyroscope full scale range */
    mpu6050_accel_bias_t    accel_bias;             /*!< MPU6050 acclerometer bias */
    mpu6050_gyro_bias_t     gyro_bias;              /*!< MPU6050 gyroscope bias */
    mpu6050_if_protocol_t   if_protocol;            /*!< MPU6050 interface protocol */
    float                   accel_scaling_factor;   /*!< MPU6050 accelerometer scaling factor */
    float                   gyro_scaling_factor;    /*!< MPU6050 gyroscope scaling factor */
    SemaphoreHandle_t       lock;                   /*!< MPU6050 mutex */
    mpu6050_hardware_info_t hw_info;                /*!< MPU6050 hardware information */
    read_func               _read;                  /*!< MPU6050 read function */
    write_func              _write;                 /*!< MPU6050 write function */
} mpu6050_t;

static stm_err_t _i2c_write_func(mpu6050_hardware_info_t hw_info, uint8_t reg_addr, uint8_t *buf, uint16_t len, uint32_t timeout_ms)
{
    uint8_t buf_send[len + 1];
    buf_send[0] = reg_addr;
    for (uint8_t i = 0; i < len; i++)
    {
        buf_send[i + 1] = buf[i];
    }

    MPU6050_CHECK(!i2c_write_bytes(hw_info.i2c_num, MPU6050_ADDR, buf_send, len + 1, timeout_ms), MPU6050_TRANS_ERR_STR, return STM_FAIL);
    return STM_OK;
}

static stm_err_t _i2c_read_func(mpu6050_hardware_info_t hw_info, uint8_t reg_addr, uint8_t *buf, uint16_t len, uint32_t timeout_ms)
{
    uint8_t buffer[1];
    buffer[0] = reg_addr;
    MPU6050_CHECK(!i2c_write_bytes(hw_info.i2c_num, MPU6050_ADDR, buffer, 1, timeout_ms), MPU6050_REC_ERR_STR, return STM_FAIL);
    MPU6050_CHECK(!i2c_read_bytes(hw_info.i2c_num, MPU6050_ADDR, buf, len, timeout_ms), MPU6050_REC_ERR_STR, return STM_FAIL);

    return STM_OK;
}

static read_func _get_read_func(mpu6050_if_protocol_t if_protocol) 
{
    if (if_protocol == MPU6050_IF_I2C)
    {
        return _i2c_read_func;
    }

    return NULL;
}

static write_func _get_write_func(mpu6050_if_protocol_t if_protocol) 
{
    if (if_protocol == MPU6050_IF_I2C)
    {
        return _i2c_write_func;
    }

    return NULL;
}

static void _mpu6050_cleanup(mpu6050_handle_t handle) {
    free(handle);
}

mpu6050_handle_t mpu6050_init(mpu6050_cfg_t *config)
{
    /* Check if init structure is empty */
    MPU6050_CHECK(config, MPU6050_INIT_ERR_STR, return NULL);
    MPU6050_CHECK(config->clksel < MPU6050_CLKSEL_MAX, MPU6050_INIT_ERR_STR, return NULL);
    MPU6050_CHECK(config->dlpf_cfg < MPU6050_DLPF_CFG_MAX, MPU6050_INIT_ERR_STR, return NULL);
    MPU6050_CHECK(config->sleep_mode < MPU6050_SLEEP_MODE_MAX, MPU6050_INIT_ERR_STR, return NULL);
    MPU6050_CHECK(config->fs_sel < MPU6050_FS_SEL_MAX, MPU6050_INIT_ERR_STR, return NULL);
    MPU6050_CHECK(config->afs_sel < MPU6050_AFS_SEL_MAX, MPU6050_INIT_ERR_STR, return NULL);
    MPU6050_CHECK(config->if_protocol < MPU6050_IF_MAX, MPU6050_INIT_ERR_STR, return NULL);

    /* Allocate memory for handle structure */
    mpu6050_handle_t handle;
    handle = calloc(1, sizeof(mpu6050_t));
    MPU6050_CHECK(handle, MPU6050_INIT_ERR_STR, return NULL);

    /* Get write function */
    write_func _write = _get_write_func(config->if_protocol);

    /* Reset MPU6050 */
    uint8_t buffer = 0;
    buffer = 0x80;
    MPU6050_CHECK(!_write(config->hw_info, MPU6050_PWR_MGMT_1, &buffer, 1, TIMEOUT_MS_DEFAULT), MPU6050_INIT_ERR_STR, {_mpu6050_cleanup(handle); return NULL;});
    vTaskDelay(100/portTICK_PERIOD_MS);

    /* Configure clock source and sleep mode */
    buffer = 0;
    buffer = config->clksel & 0x07;
    buffer |= (config->sleep_mode << 6) & 0x40;
    MPU6050_CHECK(!_write(config->hw_info, MPU6050_PWR_MGMT_1, &buffer, 1, TIMEOUT_MS_DEFAULT), MPU6050_INIT_ERR_STR, {_mpu6050_cleanup(handle); return NULL;});
    vTaskDelay(100/portTICK_PERIOD_MS);

    /* Configure digital low pass filter */
    buffer = 0;
    buffer = config->dlpf_cfg & 0x07;
    MPU6050_CHECK(!_write(config->hw_info, MPU6050_CONFIG, &buffer, 1, TIMEOUT_MS_DEFAULT), MPU6050_INIT_ERR_STR, {_mpu6050_cleanup(handle); return NULL;});

    /* Configure gyroscope range */
    buffer = 0;
    buffer = (config->fs_sel << 3) & 0x18;
    MPU6050_CHECK(!_write(config->hw_info, MPU6050_GYRO_CONFIG, &buffer, 1, TIMEOUT_MS_DEFAULT), MPU6050_INIT_ERR_STR, {_mpu6050_cleanup(handle); return NULL;});

    /* Configure accelerometer range */
    buffer = 0;
    buffer = (config->afs_sel << 3) & 0x18;
    MPU6050_CHECK(!_write(config->hw_info, MPU6050_ACCEL_CONFIG, &buffer, 1, TIMEOUT_MS_DEFAULT), MPU6050_INIT_ERR_STR, {_mpu6050_cleanup(handle); return NULL;});

    /* Configure sample rate divider */
    buffer = 0;
    buffer = 0x04;
    MPU6050_CHECK(!_write(config->hw_info, MPU6050_SMPLRT_DIV, &buffer, 1, TIMEOUT_MS_DEFAULT), MPU6050_INIT_ERR_STR, {_mpu6050_cleanup(handle); return NULL;});

    /* Update accelerometer scaling factor */
    switch (config->afs_sel)
    {
    case MPU6050_AFS_SEL_2G:
        handle->accel_scaling_factor = (2.0f / 32768.0f);
        break;

    case MPU6050_AFS_SEL_4G:
        handle->accel_scaling_factor = (4.0f / 32768.0f);
        break;

    case MPU6050_AFS_SEL_8G:
        handle->accel_scaling_factor = (8.0f / 32768.0f);
        break;

    case MPU6050_AFS_SEL_16G:
        handle->accel_scaling_factor = (16.0f / 32768.0f);
        break;

    default:
        break;
    }

    /* Update gyroscope scaling factor */
    switch (config->fs_sel)
    {
    case MPU6050_FS_SEL_250:
        handle->gyro_scaling_factor = 250.0f / 32768.0f;
        break;

    case MPU6050_FS_SEL_500:
        handle->gyro_scaling_factor = 500.0f / 32768.0f;
        break;

    case MPU6050_FS_SEL_1000:
        handle->gyro_scaling_factor = 1000.0f / 32768.0f;
        break;

    case MPU6050_FS_SEL_2000:
        handle->gyro_scaling_factor = 2000.0f / 32768.0f;
        break;

    default:
        break;
    }

    /* Update handle structure */
    handle->accel_bias.x_axis = config->accel_bias.x_axis;
    handle->accel_bias.y_axis = config->accel_bias.y_axis;
    handle->accel_bias.z_axis = config->accel_bias.z_axis;
    handle->gyro_bias.x_axis = config->gyro_bias.x_axis;
    handle->gyro_bias.y_axis = config->gyro_bias.y_axis;
    handle->gyro_bias.z_axis = config->gyro_bias.z_axis;
    handle->afs_sel = config->afs_sel;
    handle->clksel = config->clksel;
    handle->dlpf_cfg = config->dlpf_cfg;
    handle->fs_sel = config->fs_sel;
    handle->sleep_mode = config->sleep_mode;
    handle->if_protocol = config->if_protocol;
    handle->lock = mutex_create();
    handle->hw_info = config->hw_info;
    handle->_read = _get_read_func(config->if_protocol);
    handle->_write = _get_write_func(config->if_protocol);

    return handle;
}

stm_err_t mpu6050_get_accel_raw(mpu6050_handle_t handle, mpu6050_raw_data_t *raw_data)
{
    MPU6050_CHECK(handle, MPU6050_GET_DATA_ERR_STR, return STM_ERR_INVALID_ARG);
    MPU6050_CHECK(raw_data, MPU6050_GET_DATA_ERR_STR, return STM_ERR_INVALID_ARG);

    mutex_lock(handle->lock);

    int ret;
    uint8_t accel_raw_data[6];

    ret = handle->_read(handle->hw_info, MPU6050_ACCEL_XOUT_H, accel_raw_data, 6, TIMEOUT_MS_DEFAULT);
    if (ret) {
        STM_LOGE(MPU6050_TAG, MPU6050_GET_DATA_ERR_STR);
        mutex_unlock(handle->lock);
        return STM_FAIL;
    }

    raw_data->x_axis = (int16_t)((accel_raw_data[0] << 8) + accel_raw_data[1]);
    raw_data->y_axis = (int16_t)((accel_raw_data[2] << 8) + accel_raw_data[3]);
    raw_data->z_axis = (int16_t)((accel_raw_data[4] << 8) + accel_raw_data[5]);

    mutex_unlock(handle->lock);
    return STM_OK;
}

stm_err_t mpu6050_get_accel_cali(mpu6050_handle_t handle, mpu6050_cali_data_t *cali_data)
{
    MPU6050_CHECK(handle, MPU6050_GET_DATA_ERR_STR, return STM_ERR_INVALID_ARG);
    MPU6050_CHECK(cali_data, MPU6050_GET_DATA_ERR_STR, return STM_ERR_INVALID_ARG);

    mutex_lock(handle->lock);

    int ret;
    uint8_t accel_raw_data[6];

    ret = handle->_read(handle->hw_info, MPU6050_ACCEL_XOUT_H, accel_raw_data, 6, TIMEOUT_MS_DEFAULT);
    if (ret) {
        STM_LOGE(MPU6050_TAG, MPU6050_GET_DATA_ERR_STR);
        mutex_unlock(handle->lock);
        return STM_FAIL;
    }

    cali_data->x_axis = (int16_t)((accel_raw_data[0] << 8) + accel_raw_data[1]) - handle->accel_bias.x_axis;
    cali_data->y_axis = (int16_t)((accel_raw_data[2] << 8) + accel_raw_data[3]) - handle->accel_bias.y_axis;
    cali_data->z_axis = (int16_t)((accel_raw_data[4] << 8) + accel_raw_data[5]) - handle->accel_bias.z_axis;

    mutex_unlock(handle->lock);
    return STM_OK;
}

stm_err_t mpu6050_get_accel_scale(mpu6050_handle_t handle, mpu6050_scale_data_t *scale_data)
{
    MPU6050_CHECK(handle, MPU6050_GET_DATA_ERR_STR, return STM_ERR_INVALID_ARG);
    MPU6050_CHECK(scale_data, MPU6050_GET_DATA_ERR_STR, return STM_ERR_INVALID_ARG);

    mutex_lock(handle->lock);

    int ret;
    uint8_t accel_raw_data[6];

    ret = handle->_read(handle->hw_info, MPU6050_ACCEL_XOUT_H, accel_raw_data, 6, TIMEOUT_MS_DEFAULT);
    if (ret) {
        STM_LOGE(MPU6050_TAG, MPU6050_GET_DATA_ERR_STR);
        mutex_unlock(handle->lock);
        return STM_FAIL;
    }

    scale_data->x_axis = (float)((int16_t)((accel_raw_data[0] << 8) + accel_raw_data[1]) - handle->accel_bias.x_axis) * handle->accel_scaling_factor;
    scale_data->y_axis = (float)((int16_t)((accel_raw_data[2] << 8) + accel_raw_data[3]) - handle->accel_bias.y_axis) * handle->accel_scaling_factor;
    scale_data->z_axis = (float)((int16_t)((accel_raw_data[4] << 8) + accel_raw_data[5]) - handle->accel_bias.z_axis) * handle->accel_scaling_factor;

    mutex_unlock(handle->lock);
    return STM_OK;
}

stm_err_t mpu6050_get_gyro_raw(mpu6050_handle_t handle, mpu6050_raw_data_t *raw_data)
{
    MPU6050_CHECK(handle, MPU6050_GET_DATA_ERR_STR, return STM_ERR_INVALID_ARG);
    MPU6050_CHECK(raw_data, MPU6050_GET_DATA_ERR_STR, return STM_ERR_INVALID_ARG);

    mutex_lock(handle->lock);

    int ret;
    uint8_t gyro_raw_data[6];

    ret = handle->_read(handle->hw_info, MPU6050_GYRO_XOUT_H, gyro_raw_data, 6, TIMEOUT_MS_DEFAULT);
    if (ret) {
        STM_LOGE(MPU6050_TAG, MPU6050_GET_DATA_ERR_STR);
        mutex_unlock(handle->lock);
        return STM_FAIL;
    }

    raw_data->x_axis = (int16_t)((gyro_raw_data[0] << 8) + gyro_raw_data[1]);
    raw_data->y_axis = (int16_t)((gyro_raw_data[2] << 8) + gyro_raw_data[3]);
    raw_data->z_axis = (int16_t)((gyro_raw_data[4] << 8) + gyro_raw_data[5]);

    mutex_unlock(handle->lock);
    return STM_OK;
}

stm_err_t mpu6050_get_gyro_cali(mpu6050_handle_t handle, mpu6050_cali_data_t *cali_data)
{
    MPU6050_CHECK(handle, MPU6050_GET_DATA_ERR_STR, return STM_ERR_INVALID_ARG);
    MPU6050_CHECK(cali_data, MPU6050_GET_DATA_ERR_STR, return STM_ERR_INVALID_ARG);

    mutex_lock(handle->lock);

    int ret;
    uint8_t gyro_raw_data[6];

    ret = handle->_read(handle->hw_info, MPU6050_GYRO_XOUT_H, gyro_raw_data, 6, TIMEOUT_MS_DEFAULT);
    if (ret) {
        STM_LOGE(MPU6050_TAG, MPU6050_GET_DATA_ERR_STR);
        mutex_unlock(handle->lock);
        return STM_FAIL;
    }

    cali_data->x_axis = (int16_t)((gyro_raw_data[0] << 8) + gyro_raw_data[1]) - handle->gyro_bias.x_axis;
    cali_data->y_axis = (int16_t)((gyro_raw_data[2] << 8) + gyro_raw_data[3]) - handle->gyro_bias.y_axis;
    cali_data->z_axis = (int16_t)((gyro_raw_data[4] << 8) + gyro_raw_data[5]) - handle->gyro_bias.z_axis;

    mutex_unlock(handle->lock);
    return STM_OK;
}

stm_err_t mpu6050_get_gyro_scale(mpu6050_handle_t handle, mpu6050_scale_data_t *scale_data)
{
    MPU6050_CHECK(handle, MPU6050_GET_DATA_ERR_STR, return STM_ERR_INVALID_ARG);
    MPU6050_CHECK(scale_data, MPU6050_GET_DATA_ERR_STR, return STM_ERR_INVALID_ARG);

    mutex_lock(handle->lock);

    int ret;
    uint8_t gyro_raw_data[6];

    ret = handle->_read(handle->hw_info, MPU6050_GYRO_XOUT_H, gyro_raw_data, 6, TIMEOUT_MS_DEFAULT);
    if (ret) {
        STM_LOGE(MPU6050_TAG, MPU6050_GET_DATA_ERR_STR);
        mutex_unlock(handle->lock);
        return STM_FAIL;
    }

    scale_data->x_axis = (float)((int16_t)((gyro_raw_data[0] << 8) + gyro_raw_data[1]) - handle->gyro_bias.x_axis) * handle->gyro_scaling_factor;
    scale_data->y_axis = (float)((int16_t)((gyro_raw_data[2] << 8) + gyro_raw_data[3]) - handle->gyro_bias.y_axis) * handle->gyro_scaling_factor;
    scale_data->z_axis = (float)((int16_t)((gyro_raw_data[4] << 8) + gyro_raw_data[5]) - handle->gyro_bias.z_axis) * handle->gyro_scaling_factor;

    mutex_unlock(handle->lock);
    return STM_OK;
}

stm_err_t mpu6050_set_accel_bias(mpu6050_handle_t handle, mpu6050_accel_bias_t accel_bias)
{
    MPU6050_CHECK(handle, MPU6050_SET_BIAS_ERR_STR, return STM_ERR_INVALID_ARG);

    mutex_lock(handle->lock);
    handle->accel_bias.x_axis = accel_bias.x_axis;
    handle->accel_bias.y_axis = accel_bias.y_axis;
    handle->accel_bias.z_axis = accel_bias.z_axis;
    mutex_unlock(handle->lock);

    return STM_OK;
}

stm_err_t mpu6050_set_gyro_bias(mpu6050_handle_t handle, mpu6050_gyro_bias_t gyro_bias)
{
    MPU6050_CHECK(handle, MPU6050_SET_BIAS_ERR_STR, return STM_ERR_INVALID_ARG);

    mutex_lock(handle->lock);
    handle->gyro_bias.x_axis = gyro_bias.x_axis;
    handle->gyro_bias.y_axis = gyro_bias.y_axis;
    handle->gyro_bias.z_axis = gyro_bias.z_axis;
    mutex_unlock(handle->lock);

    return STM_OK;
}

stm_err_t mpu6050_get_accel_bias(mpu6050_handle_t handle, mpu6050_accel_bias_t *accel_bias)
{
    MPU6050_CHECK(handle, MPU6050_GET_BIAS_ERR_STR, return STM_ERR_INVALID_ARG);

    mutex_lock(handle->lock);
    accel_bias->x_axis = handle->accel_bias.x_axis;
    accel_bias->y_axis = handle->accel_bias.y_axis;
    accel_bias->z_axis = handle->accel_bias.z_axis;
    mutex_unlock(handle->lock);

    return STM_OK;
}

stm_err_t mpu6050_get_gyro_bias(mpu6050_handle_t handle, mpu6050_gyro_bias_t *gyro_bias)
{
    MPU6050_CHECK(handle, MPU6050_GET_BIAS_ERR_STR, return STM_ERR_INVALID_ARG);

    mutex_lock(handle->lock);
    gyro_bias->x_axis = handle->gyro_bias.x_axis;
    gyro_bias->y_axis = handle->gyro_bias.y_axis;
    gyro_bias->z_axis = handle->gyro_bias.z_axis;
    mutex_unlock(handle->lock);

    return STM_OK;
}

void mpu6050_auto_calib(mpu6050_handle_t handle)
{
    int buffersize = BUFFER_CALIB_DEFAULT;
    int mean_ax, mean_ay, mean_az, mean_gx, mean_gy, mean_gz;
    long i = 0, buff_ax = 0, buff_ay = 0, buff_az = 0, buff_gx = 0, buff_gy = 0, buff_gz = 0;

    while (i < (buffersize + 101))                  /*!< Dismiss 100 first value */
    {
        mpu6050_raw_data_t accel_raw, gyro_raw;
        mpu6050_get_accel_raw(handle, &accel_raw);
        mpu6050_get_gyro_raw(handle, &gyro_raw);

        if (i > 100 && i <= (buffersize + 100))
        {
            buff_ax += accel_raw.x_axis;
            buff_ay += accel_raw.y_axis;
            buff_az += accel_raw.z_axis;
            buff_gx += gyro_raw.x_axis;
            buff_gy += gyro_raw.y_axis;
            buff_gz += gyro_raw.z_axis;
        }
        if (i == (buffersize + 100))
        {
            mean_ax = buff_ax / buffersize;
            mean_ay = buff_ay / buffersize;
            mean_az = buff_az / buffersize;
            mean_gx = buff_gx / buffersize;
            mean_gy = buff_gy / buffersize;
            mean_gz = buff_gz / buffersize;
        }
        i++;
    }

    handle->accel_bias.x_axis = mean_ax;
    handle->accel_bias.y_axis = mean_ay;
    handle->accel_bias.z_axis = mean_az - 1.0f / handle->accel_scaling_factor;
    handle->gyro_bias.x_axis = mean_gx;
    handle->gyro_bias.y_axis = mean_gy;
    handle->gyro_bias.z_axis = mean_gz;
}

