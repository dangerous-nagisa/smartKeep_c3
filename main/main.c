/**
 * @file main.c
 * @brief smartKeep 姿态解算节点
 *
 * 数据流: BMI270(200Hz) → LSB换算 → 零偏校准 → LPF → Madgwick AHRS
 *         → node_packet_t(50Hz) → FreeRTOS队列 → UDP广播
 *
 * 网络: WiFi STA → UDP 广播 255.255.255.255:8888
 *
 * 硬件:
 *   GPIO4  I2C SDA ──┐
 *   GPIO5  I2C SCL ──┤── BMI270 (0x68)
 *   GPIO2  ADC      ── 电池采样 (TODO)
 *   GPIO6  PWM      ── 蜂鸣器   (TODO)
 */

#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stddef.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "lwip/sockets.h"

#include "bmi270.h"
#include "bmi270_interface.h"
#include "node_config.h"

static const char *TAG = "smartKeep";

// =============================================================================
// § 1  配置宏
// =============================================================================

/* --- 硬件引脚 --- */
#define I2C_SDA_PIN GPIO_NUM_4
#define I2C_SCL_PIN GPIO_NUM_5
#define BMI270_I2C_ADDR 0x68
#define BUZZER_PIN GPIO_NUM_6       // 低电平有源蜂鸣器

/* --- 传感器量程 --- */
#define ACC_RANGE_G 4.0f      // ±4 g
#define GYR_RANGE_DPS 2000.0f // ±2000 °/s

/* --- WiFi 配置 --- */
#define WIFI_SSID "djrs"       // ← 填入热点名称
#define WIFI_PASS "2661760820" // ← 填入热点密码
#define WIFI_MAX_RETRY 10

/* --- UDP 配置 --- */
#define UDP_BROADCAST_IP "255.255.255.255"
#define UDP_PORT 8888

/* --- 任务参数 --- */
#define IMU_TASK_STACK 8192   // C3 无 FPU，浮点运算栈消耗大
#define IMU_TASK_PRIO 5
#define UDP_TASK_STACK 6144   // 网络操作需要更多栈空间
#define UDP_TASK_PRIO 4
#define PKT_QUEUE_DEPTH 10    // 队列深度, 满则丢弃最新
#define IMU_LOOP_PERIOD_MS 10 // 100 Hz 内部循环
#define PACKET_DECIMATION 2   // 每 2 帧输出 1 包 → 50 Hz
#define LOG_DECIMATION 50     // 每 50 包打印 1 次 → 1 Hz

/* --- 零偏校准 --- */
#define GYRO_CALIB_SAMPLES 200    // 静置采集帧数 (~2 s)
#define GYRO_CALIB_TIMEOUT_MS 5000 // 校准超时 (ms)

/* --- 低通滤波系数 (0‥1, 越小越平滑) --- */
#define LPF_ALPHA_ACC 0.3f
#define LPF_ALPHA_GYR 0.5f

/* --- Madgwick AHRS --- */
#define MADGWICK_BETA_FAST 2.0f    // 收敛阶段 (前 1 s)
#define MADGWICK_BETA_NORMAL 0.04f // 稳态
#define CONVERGE_FRAMES 100        // 快速收敛持续帧数

/* --- 步态检测阈值 --- */
// 触地 (Heel Strike): jerk 尖峰 + 加速度冲击
#define STEP_JERK_THRESH        50.0f       // m/s³  (~5g/s)
#define STEP_ACCEL_THRESH       14.7f       // m/s²  (~1.5g)
// 离地 (Toe Off): 角速度峰值 + 加速度回落
#define SWING_GYRO_THRESH       100.0f      // °/s   (甩腿动作)
#define SWING_ACCEL_LOW         11.0f       // m/s²  (~1.1g, 低于支撑期)
// 去抖
#define STEP_DEBOUNCE_MS        150         // 触地↔离地 最小间隔
#define STEP_MIN_INTERVAL_MS    300         // 两次触地 最小间隔

// step_flag 编码
#define STEP_NONE               0           // 无事件
#define STEP_HEEL_STRIKE        1           // 触地
#define STEP_TOE_OFF            2           // 离地

// 步态状态机
typedef enum {
    GAIT_SWING,     // 摆动期 (脚在空中)
    GAIT_STANCE,    // 支撑期 (脚在地上)
} gait_phase_t;

/* --- 数学常量 --- */
#define DEG_TO_RAD (M_PI / 180.0f)
#define RAD_TO_DEG (180.0f / M_PI)
#define GRAVITY 9.80665f

// =============================================================================
// § 2  node_packet_t  (44 字节，自然对齐，无 packed)
// =============================================================================

typedef struct
{
    uint8_t node_id;
    uint8_t battery;
    uint16_t seq;
    uint32_t timestamp; // ms since boot (溢出 ~49 天)

    float euler[3]; // roll / pitch / yaw  (°)

    float accel[3];  // ax / ay / az  (m/s², 滤波后，传感器坐标系)
    float gyro_norm; // |ω|  (°/s)

    float jerk;        // |da/dt|  (m/s³)
    uint8_t step_flag; // 0=无 1=触地(heel strike) 2=离地(toe off)
    uint8_t crc8;
} node_packet_t; // sizeof = 44

// =============================================================================
// § 3  全局状态
// =============================================================================

/* BMI270 驱动 */
static struct bmi2_dev g_bmi2_dev;
static i2c_master_bus_handle_t g_i2c_bus = NULL;
static SemaphoreHandle_t g_i2c_mutex = NULL;

/* Madgwick 四元数 */
static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;

/* WiFi 事件组 */
static EventGroupHandle_t g_wifi_event_group = NULL;
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1
static int s_retry_num = 0;

/* IMU → UDP 队列 */
static QueueHandle_t g_pkt_queue = NULL;

// =============================================================================
// § 4  CRC-8  (poly = 0x07, init = 0x00)
// =============================================================================

static uint8_t crc8_calc(const uint8_t *data, size_t len)
{
    uint8_t crc = 0x00;
    for (size_t i = 0; i < len; i++)
    {
        crc ^= data[i];
        for (int b = 0; b < 8; b++)
        {
            crc = (crc & 0x80) ? ((crc << 1) ^ 0x07) : (crc << 1);
        }
    }
    return crc;
}

// =============================================================================
// § 5  单位换算
// =============================================================================

static inline float lsb_to_mps2(int16_t val)
{
    return (GRAVITY * val * ACC_RANGE_G) / 32768.0f;
}

static inline float lsb_to_dps(int16_t val)
{
    return (GYR_RANGE_DPS * val) / 32768.0f;
}

// =============================================================================
// § 6  一阶 IIR 低通滤波
// =============================================================================

typedef struct
{
    float ax, ay, az;
    float gx, gy, gz;
    bool primed; // 首帧直通
} lpf_state_t;

static void lpf_update(lpf_state_t *s,
                       float ax, float ay, float az,
                       float gx, float gy, float gz)
{
    if (!s->primed)
    {
        s->ax = ax;
        s->ay = ay;
        s->az = az;
        s->gx = gx;
        s->gy = gy;
        s->gz = gz;
        s->primed = true;
        return;
    }
    const float aa = LPF_ALPHA_ACC, ga = LPF_ALPHA_GYR;
    s->ax += aa * (ax - s->ax);
    s->ay += aa * (ay - s->ay);
    s->az += aa * (az - s->az);
    s->gx += ga * (gx - s->gx);
    s->gy += ga * (gy - s->gy);
    s->gz += ga * (gz - s->gz);
}

// =============================================================================
// § 7  Madgwick AHRS — 6 轴 IMU (无磁力计)
//      参考: S. Madgwick, "An efficient orientation filter for inertial
//             and inertial/magnetic sensor arrays", 2010
// =============================================================================

static void madgwick_update(float ax, float ay, float az,
                            float gx_dps, float gy_dps, float gz_dps,
                            float dt, float beta)
{
    /* --- 陀螺仪 °/s → rad/s --- */
    float gx = gx_dps * DEG_TO_RAD;
    float gy = gy_dps * DEG_TO_RAD;
    float gz = gz_dps * DEG_TO_RAD;

    /* --- 四元数微分 (陀螺仪积分) --- */
    float qDot0 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    float qDot1 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    float qDot2 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    float qDot3 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

    /* --- 加速度计校正 (梯度下降) --- */
    float a_norm = sqrtf(ax * ax + ay * ay + az * az);
    if (a_norm > 0.01f)
    {
        float recip = 1.0f / a_norm;
        ax *= recip;
        ay *= recip;
        az *= recip;

        float _2q0 = 2.0f * q0, _2q1 = 2.0f * q1;
        float _2q2 = 2.0f * q2, _2q3 = 2.0f * q3;
        float _4q0 = 4.0f * q0, _4q1 = 4.0f * q1;
        float _4q2 = 4.0f * q2;
        float _8q1 = 8.0f * q1, _8q2 = 8.0f * q2;
        float q0q0 = q0 * q0, q1q1 = q1 * q1;
        float q2q2 = q2 * q2, q3q3 = q3 * q3;

        float s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        float s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        float s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        float s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;

        float s_norm = 1.0f / sqrtf(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
        s0 *= s_norm;
        s1 *= s_norm;
        s2 *= s_norm;
        s3 *= s_norm;

        qDot0 -= beta * s0;
        qDot1 -= beta * s1;
        qDot2 -= beta * s2;
        qDot3 -= beta * s3;
    }

    /* --- 积分 --- */
    q0 += qDot0 * dt;
    q1 += qDot1 * dt;
    q2 += qDot2 * dt;
    q3 += qDot3 * dt;

    /* --- 归一化四元数 --- */
    float qn = 1.0f / sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= qn;
    q1 *= qn;
    q2 *= qn;
    q3 *= qn;
}

// =============================================================================
// § 8  四元数 → 欧拉角 (ZYX 顺序)
// =============================================================================

static void quat_to_euler(float *roll, float *pitch, float *yaw)
{
    /* roll  (X) */
    *roll = atan2f(2.0f * (q0 * q1 + q2 * q3),
                   1.0f - 2.0f * (q1 * q1 + q2 * q2)) *
            RAD_TO_DEG;

    /* pitch (Y) — clamp 防 NaN */
    float sinp = 2.0f * (q0 * q2 - q3 * q1);
    if (sinp >= 1.0f)
        sinp = 1.0f;
    if (sinp <= -1.0f)
        sinp = -1.0f;
    *pitch = asinf(sinp) * RAD_TO_DEG;

    /* yaw   (Z) — 无磁力计，会漂移 */
    *yaw = atan2f(2.0f * (q0 * q3 + q1 * q2),
                  1.0f - 2.0f * (q2 * q2 + q3 * q3)) *
           RAD_TO_DEG;
    if (*yaw < 0.0f)
        *yaw += 360.0f; // 映射到 0~360
}

// =============================================================================
// § 9  WiFi STA 初始化
// =============================================================================

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        wifi_event_sta_disconnected_t *disc = (wifi_event_sta_disconnected_t *)event_data;
        ESP_LOGW(TAG, "WiFi 断连  reason=%d  ssid=\"%.*s\"",
                 disc->reason, disc->ssid_len, disc->ssid);

        if (s_retry_num < WIFI_MAX_RETRY)
        {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGW(TAG, "WiFi 重连 (%d/%d)…", s_retry_num, WIFI_MAX_RETRY);
        }
        else
        {
            xEventGroupSetBits(g_wifi_event_group, WIFI_FAIL_BIT);
            ESP_LOGE(TAG, "WiFi 连接失败，已达最大重试");
        }
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "WiFi 已连接  IP=" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(g_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

/**
 * @brief 初始化 WiFi STA 模式并阻塞等待连接
 * @return ESP_OK 已连接，ESP_FAIL 连接失败
 */
static esp_err_t wifi_sta_init(void)
{
    g_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    /* 注册事件处理 */
    esp_event_handler_instance_t inst_any_id, inst_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, &inst_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, &inst_got_ip));

    /* 配置 SSID / 密码 */
    wifi_config_t wifi_cfg = {
        .sta = {
            .threshold.authmode = WIFI_AUTH_OPEN, // 自动协商认证模式
            .sae_pwe_h2e = WPA3_SAE_PWE_BOTH,     // 兼容 WPA3
        },
    };
    /* strncpy 保证不越界 */
    strncpy((char *)wifi_cfg.sta.ssid, WIFI_SSID, sizeof(wifi_cfg.sta.ssid) - 1);
    strncpy((char *)wifi_cfg.sta.password, WIFI_PASS, sizeof(wifi_cfg.sta.password) - 1);

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg));

    ESP_ERROR_CHECK(esp_wifi_start());
    /* 降低 WiFi 发射功率以减少功耗峰值 (默认 20dBm → 8dBm) */
    esp_wifi_set_max_tx_power(8);

    ESP_LOGI(TAG, "WiFi STA 启动, 正在连接 \"%s\" …", WIFI_SSID);

    /* 阻塞等待连接结果 */
    EventBits_t bits = xEventGroupWaitBits(g_wifi_event_group,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE, pdFALSE, portMAX_DELAY);

    if (bits & WIFI_CONNECTED_BIT)
    {
        return ESP_OK;
    }
    ESP_LOGE(TAG, "WiFi 连接失败");
    return ESP_FAIL;
}

// =============================================================================
// § 10  UDP 发送任务
// =============================================================================

/**
 * @brief UDP 发送任务 — 从队列取 node_packet_t 广播出去
 *
 * 特性：
 *   - socket 创建失败会重试
 *   - sendto 失败仅打印警告，不阻塞 IMU 管道
 *   - WiFi 断连期间包会在队列中堆积 (满则 IMU 侧丢弃)
 */
static void udp_tx_task(void *arg)
{
    (void)arg;
    node_packet_t pkt;

    /* 创建 UDP socket */
    int sock = -1;
    struct sockaddr_in dest_addr = {
        .sin_family = AF_INET,
        .sin_port = htons(UDP_PORT),
    };
    inet_aton(UDP_BROADCAST_IP, &dest_addr.sin_addr);

    while (1)
    {
        /* 确保 socket 有效 */
        if (sock < 0)
        {
            sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
            if (sock < 0)
            {
                ESP_LOGE(TAG, "UDP socket 创建失败, 1s 后重试");
                vTaskDelay(pdMS_TO_TICKS(1000));
                continue;
            }
            /* 启用广播 */
            int broadcast = 1;
            setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast));
            ESP_LOGI(TAG, "UDP socket 就绪  → %s:%d", UDP_BROADCAST_IP, UDP_PORT);
        }

        /* 阻塞等待队列数据 */
        if (xQueueReceive(g_pkt_queue, &pkt, portMAX_DELAY) == pdTRUE)
        {
            int ret = sendto(sock, &pkt, sizeof(pkt), 0,
                             (struct sockaddr *)&dest_addr, sizeof(dest_addr));
            if (ret < 0)
            {
                ESP_LOGW(TAG, "UDP sendto 失败: errno=%d", errno);
                /* socket 可能坏了，关闭重建 */
                close(sock);
                sock = -1;
            }
        }
    }
}

// =============================================================================
// § 10.5  蜂鸣器驱动 (低电平有源蜂鸣器, GPIO6)
//
//   LOW  = 响
//   HIGH = 停
// =============================================================================

/**
 * @brief 初始化蜂鸣器 GPIO (默认关闭)
 */
static void buzzer_init(void)
{
    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << BUZZER_PIN),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg);
    gpio_set_level(BUZZER_PIN, 1);  // 默认高电平 = 关闭
}

/** 蜂鸣器开 */
static inline void buzzer_on(void)
{
    gpio_set_level(BUZZER_PIN, 0);
}

/** 蜂鸣器关 */
static inline void buzzer_off(void)
{
    gpio_set_level(BUZZER_PIN, 1);
}

/**
 * @brief 蜂鸣器短响 (阻塞)
 * @param ms 持续时间 (毫秒)
 */
static void buzzer_beep(uint32_t ms)
{
    buzzer_on();
    vTaskDelay(pdMS_TO_TICKS(ms));
    buzzer_off();
}

/**
 * @brief 蜂鸣器连续短响 (阻塞)
 * @param count    响几次
 * @param on_ms    每次响的时长
 * @param off_ms   间隔时长
 */
static void buzzer_beep_n(int count, uint32_t on_ms, uint32_t off_ms)
{
    for (int i = 0; i < count; i++) {
        buzzer_on();
        vTaskDelay(pdMS_TO_TICKS(on_ms));
        buzzer_off();
        if (i < count - 1) {
            vTaskDelay(pdMS_TO_TICKS(off_ms));
        }
    }
}

// =============================================================================
// § 11  I2C 总线初始化
// =============================================================================

static esp_err_t i2c_bus_init(void)
{
    i2c_master_bus_config_t cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = I2C_SDA_PIN,
        .scl_io_num = I2C_SCL_PIN,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    esp_err_t ret = i2c_new_master_bus(&cfg, &g_i2c_bus);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "I2C 总线初始化失败: %s", esp_err_to_name(ret));
    }
    return ret;
}

// =============================================================================
// § 12  BMI270 初始化 / 配置 / 使能
// =============================================================================

static int8_t bmi270_hw_init(void)
{
    int8_t r;

    bmi2_set_i2c_configuration(g_i2c_bus, BMI270_I2C_ADDR, g_i2c_mutex);

    r = bmi2_interface_init(&g_bmi2_dev, BMI2_I2C_INTF);
    if (r != BMI2_OK)
    {
        bmi2_error_codes_print_result(r);
        return r;
    }

    r = bmi270_init(&g_bmi2_dev);
    if (r != BMI2_OK)
    {
        bmi2_error_codes_print_result(r);
        return r;
    }
    ESP_LOGI(TAG, "BMI270 OK  Chip-ID=0x%02X", g_bmi2_dev.chip_id);

    /* --- 配置 ODR/量程 --- */
    struct bmi2_sens_config sc[2];
    sc[0].type = BMI2_ACCEL;
    sc[1].type = BMI2_GYRO;
    r = bmi2_get_sensor_config(sc, 2, &g_bmi2_dev);
    if (r != BMI2_OK)
    {
        bmi2_error_codes_print_result(r);
        return r;
    }

    sc[0].cfg.acc.odr = BMI2_ACC_ODR_200HZ;
    sc[0].cfg.acc.range = BMI2_ACC_RANGE_4G;
    sc[0].cfg.acc.bwp = BMI2_ACC_NORMAL_AVG4;
    sc[0].cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;

    sc[1].cfg.gyr.odr = BMI2_GYR_ODR_200HZ;
    sc[1].cfg.gyr.range = BMI2_GYR_RANGE_2000;
    sc[1].cfg.gyr.bwp = BMI2_GYR_NORMAL_MODE;
    sc[1].cfg.gyr.noise_perf = BMI2_POWER_OPT_MODE;
    sc[1].cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;

    r = bmi2_set_sensor_config(sc, 2, &g_bmi2_dev);
    if (r != BMI2_OK)
    {
        bmi2_error_codes_print_result(r);
        return r;
    }

    /* --- 使能 --- */
    uint8_t list[2] = {BMI2_ACCEL, BMI2_GYRO};
    r = bmi2_sensor_enable(list, 2, &g_bmi2_dev);
    if (r != BMI2_OK)
    {
        bmi2_error_codes_print_result(r);
        return r;
    }

    ESP_LOGI(TAG, "ACC 200Hz ±4g | GYR 200Hz ±2000dps");
    return BMI2_OK;
}

// =============================================================================
// § 13  IMU 管道任务
//
//  阶段 A  零偏校准 (~2 s 静置)
//  阶段 B  快速收敛 (~1 s, beta=2.0)
//  阶段 C  正常运行 (beta=0.04, 持续输出 node_packet_t @ 50 Hz)
// =============================================================================

static void imu_task(void *arg)
{
    (void)arg;

    /* ---- 校准缓存 ---- */
    float bias_gx = 0, bias_gy = 0, bias_gz = 0;

    /* ---- 滤波器 ---- */
    lpf_state_t lpf = {0};

    /* ---- Jerk 计算 ---- */
    float prev_ax = 0, prev_ay = 0, prev_az = 0;
    bool jerk_primed = false;

    /* ---- 步态状态机 ---- */
    gait_phase_t gait_phase = GAIT_SWING;   // 初始假设脚在空中
    int64_t last_gait_event_us = 0;         // 上次触地/离地时间
    int64_t last_heel_strike_us = 0;        // 上次触地时间 (步频去抖)

    /* ---- 包序列号 ---- */
    uint16_t seq = 0;
    int frame = 0; // 帧计数 (用于 decimation)
    int converge_cnt = 0;
    int log_cnt = 0;

    /* ---- 时间基准 ---- */
    int64_t t_prev = esp_timer_get_time();
    TickType_t wake = xTaskGetTickCount();

    // =========================================================================
    // 阶段 A: 陀螺仪零偏校准
    // =========================================================================
    ESP_LOGW(TAG, ">>> 陀螺仪校准中，请保持静止 …");
    {
        float sum_gx = 0, sum_gy = 0, sum_gz = 0;
        int n = 0;
        int fail_cnt = 0;
        int64_t calib_start = esp_timer_get_time();
        struct bmi2_sens_data sd;

        while (n < GYRO_CALIB_SAMPLES)
        {
            vTaskDelayUntil(&wake, pdMS_TO_TICKS(IMU_LOOP_PERIOD_MS));

            /* 超时保护：避免无限阻塞触发看门狗 */
            if ((esp_timer_get_time() - calib_start) > (GYRO_CALIB_TIMEOUT_MS * 1000LL))
            {
                ESP_LOGE(TAG, "校准超时！已成功 %d/%d 次，使用默认零偏", n, GYRO_CALIB_SAMPLES);
                if (n > 10)
                    goto calib_done;  // 至少 10 次有效数据，勉强继续
                else
                {
                    ESP_LOGE(TAG, "有效样本不足，系统暂停");
                    vTaskSuspend(NULL);  // 挂起任务，避免无限重启
                }
            }

            if (bmi2_get_sensor_data(&sd, &g_bmi2_dev) != BMI2_OK)
            {
                fail_cnt++;
                if (fail_cnt % 10 == 0)
                    ESP_LOGW(TAG, "I2C 读取失败 %d 次", fail_cnt);
                continue;
            }
            if (!(sd.status & BMI2_DRDY_GYR))
                continue;

            sum_gx += lsb_to_dps(sd.gyr.x);
            sum_gy += lsb_to_dps(sd.gyr.y);
            sum_gz += lsb_to_dps(sd.gyr.z);
            n++;
        }
calib_done:
        bias_gx = sum_gx / n;
        bias_gy = sum_gy / n;
        bias_gz = sum_gz / n;
    }
    ESP_LOGI(TAG, "校准完成  bias=(%.3f, %.3f, %.3f) °/s",
             bias_gx, bias_gy, bias_gz);
    // buzzer_beep_n(2, 50, 50);  // 校准完成: 嘀嘀两声 (暂时禁用)

    /* 重置时间戳 */
    t_prev = esp_timer_get_time();

    // =========================================================================
    // 阶段 B+C: 主循环  (100 Hz 内部, 50 Hz 输出)
    // =========================================================================
    int main_loop_cnt = 0;
    int i2c_fail_cnt = 0;
    int drdy_fail_cnt = 0;

    ESP_LOGI(TAG, ">>> 进入主循环");

    while (1)
    {
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(IMU_LOOP_PERIOD_MS));
        main_loop_cnt++;

        /* ---- 计算真实 dt ---- */
        int64_t t_now = esp_timer_get_time();
        float dt = (t_now - t_prev) * 1e-6f; // μs → s
        t_prev = t_now;
        if (dt <= 0.0f || dt > 0.1f)
            dt = 0.01f; // 安全钳位

        /* ---- 读取传感器 ---- */
        struct bmi2_sens_data sd;
        if (bmi2_get_sensor_data(&sd, &g_bmi2_dev) != BMI2_OK)
        {
            i2c_fail_cnt++;
            if (i2c_fail_cnt % 50 == 1)  // 每 50 次打印一次
                ESP_LOGW(TAG, "I2C 读取失败 #%d", i2c_fail_cnt);
            continue;
        }
        if (!((sd.status & BMI2_DRDY_ACC) && (sd.status & BMI2_DRDY_GYR)))
        {
            drdy_fail_cnt++;
            if (drdy_fail_cnt % 50 == 1)
                ESP_LOGW(TAG, "数据不就绪 #%d, status=0x%02X", drdy_fail_cnt, sd.status);
            continue;
        }

        /* 成功读取，重置计数器 */
        if (i2c_fail_cnt > 0 || drdy_fail_cnt > 0)
        {
            ESP_LOGI(TAG, "恢复正常: I2C失败=%d, DRDY失败=%d", i2c_fail_cnt, drdy_fail_cnt);
            i2c_fail_cnt = 0;
            drdy_fail_cnt = 0;
        }

        /* ---- LSB → 物理量 ---- */
        float ax = lsb_to_mps2(sd.acc.x);
        float ay = lsb_to_mps2(sd.acc.y);
        float az = lsb_to_mps2(sd.acc.z);
        float gx = lsb_to_dps(sd.gyr.x) - bias_gx;
        float gy = lsb_to_dps(sd.gyr.y) - bias_gy;
        float gz = lsb_to_dps(sd.gyr.z) - bias_gz;

        /* ---- 低通滤波 ---- */
        lpf_update(&lpf, ax, ay, az, gx, gy, gz);

        /* ---- Madgwick AHRS ---- */
        float beta = (converge_cnt < CONVERGE_FRAMES)
                         ? MADGWICK_BETA_FAST
                         : MADGWICK_BETA_NORMAL;
        converge_cnt++;
        madgwick_update(lpf.ax, lpf.ay, lpf.az,
                        lpf.gx, lpf.gy, lpf.gz, dt, beta);

        /* ---- 50 Hz 输出 ---- */
        frame++;
        if (frame < PACKET_DECIMATION)
            continue;
        frame = 0;

        /* ---- 构建 node_packet_t ---- */
        node_packet_t pkt = {0};

        pkt.node_id = node_config_get_id();
        pkt.battery = 0xFF; // TODO: ADC GPIO2
        pkt.seq = seq++;
        pkt.timestamp = (uint32_t)(t_now / 1000); // μs → ms

        /* 欧拉角 */
        quat_to_euler(&pkt.euler[0], &pkt.euler[1], &pkt.euler[2]);

        /* 滤波后加速度 */
        pkt.accel[0] = lpf.ax;
        pkt.accel[1] = lpf.ay;
        pkt.accel[2] = lpf.az;

        /* 角速度模值 */
        pkt.gyro_norm = sqrtf(lpf.gx * lpf.gx + lpf.gy * lpf.gy + lpf.gz * lpf.gz);

        /* Jerk = |da/dt|  (向量微分的模) */
        if (jerk_primed)
        {
            float inv_dt = 1.0f / (dt * PACKET_DECIMATION);
            float jx = (lpf.ax - prev_ax) * inv_dt;
            float jy = (lpf.ay - prev_ay) * inv_dt;
            float jz = (lpf.az - prev_az) * inv_dt;
            pkt.jerk = sqrtf(jx * jx + jy * jy + jz * jz);
        }
        prev_ax = lpf.ax;
        prev_ay = lpf.ay;
        prev_az = lpf.az;
        jerk_primed = true;

        /* ================================================================
         * 步态状态机  SWING(摆动) ↔ STANCE(支撑)
         *
         *   SWING → STANCE : 触地 (Heel Strike)
         *     条件: (jerk > 阈值 || |accel| > 阈值) && 去抖
         *     输出: step_flag = 1
         *
         *   STANCE → SWING : 离地 (Toe Off)
         *     条件: gyro_norm > 阈值 && |accel| < 低阈值 && 去抖
         *     输出: step_flag = 2
         * ================================================================ */
        float acc_mag = sqrtf(lpf.ax * lpf.ax + lpf.ay * lpf.ay + lpf.az * lpf.az);
        int64_t since_last_event = t_now - last_gait_event_us;

        if (gait_phase == GAIT_SWING) {
            /* --- 检测触地 --- */
            bool jerk_spike   = pkt.jerk > STEP_JERK_THRESH;
            bool accel_impact = acc_mag > STEP_ACCEL_THRESH;
            bool debounce_ok  = since_last_event > (STEP_DEBOUNCE_MS * 1000LL);
            bool step_ok      = (t_now - last_heel_strike_us) > (STEP_MIN_INTERVAL_MS * 1000LL);

            if ((jerk_spike || accel_impact) && debounce_ok && step_ok) {
                pkt.step_flag       = STEP_HEEL_STRIKE;
                gait_phase          = GAIT_STANCE;
                last_gait_event_us  = t_now;
                last_heel_strike_us = t_now;
            }
        } else {  /* GAIT_STANCE */
            /* --- 检测离地 --- */
            bool gyro_swing  = pkt.gyro_norm > SWING_GYRO_THRESH;
            bool accel_low   = acc_mag < SWING_ACCEL_LOW;
            bool debounce_ok = since_last_event > (STEP_DEBOUNCE_MS * 1000LL);

            if (gyro_swing && accel_low && debounce_ok) {
                pkt.step_flag      = STEP_TOE_OFF;
                gait_phase         = GAIT_SWING;
                last_gait_event_us = t_now;
            }
        }

        /* CRC-8 (覆盖 crc8 前所有字段) */
        pkt.crc8 = crc8_calc((const uint8_t *)&pkt,
                             offsetof(node_packet_t, crc8));

        /* ---- 低频串口打印 (1 Hz) ---- */
        log_cnt++;
        if (log_cnt >= LOG_DECIMATION)
        {
            log_cnt = 0;
            const char *gait_str = (gait_phase == GAIT_STANCE) ? "STANCE" : "SWING ";
            ESP_LOGI(TAG,
                     "R=%+6.1f P=%+6.1f Y=%+6.1f | "
                     "A=(%+5.1f,%+5.1f,%+5.1f) |ω|=%5.1f J=%5.1f [%s]",
                     pkt.euler[0], pkt.euler[1], pkt.euler[2],
                     pkt.accel[0], pkt.accel[1], pkt.accel[2],
                     pkt.gyro_norm, pkt.jerk, gait_str);
        }

        /* 步态事件实时打印 (只在事件发生时输出) */
        if (pkt.step_flag == STEP_HEEL_STRIKE) {
            ESP_LOGI(TAG, ">>> HEEL STRIKE  J=%.1f |a|=%.1f",
                     pkt.jerk, acc_mag);
        } else if (pkt.step_flag == STEP_TOE_OFF) {
            ESP_LOGI(TAG, "<<< TOE OFF      |ω|=%.1f |a|=%.1f",
                     pkt.gyro_norm, acc_mag);
        }

        /* ---- 发送到 UDP 队列 (满则丢弃，不阻塞 IMU) ---- */
        xQueueSend(g_pkt_queue, &pkt, 0);
    }
}

// =============================================================================
// § 14  app_main
// =============================================================================

void app_main(void)
{
    ESP_LOGI(TAG, "========== smartKeep 姿态节点启动 ==========");

    /* 0. 蜂鸣器初始化 + 启动提示音 (暂时禁用) */
    // buzzer_init();
    // buzzer_beep(100);

    /* 1. NVS 初始化 (WiFi + 节点配置 依赖) */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* 2. 节点 ID 配置 (从 NVS 加载) */
    uint8_t nid = node_config_init();
    ESP_LOGI(TAG, "节点 ID: %d", nid);

    /* 2.5 延迟等待电源稳定后再开启 WiFi */
    vTaskDelay(pdMS_TO_TICKS(500));

    /* 3. WiFi 连接 */
    bool wifi_ok = (wifi_sta_init() == ESP_OK);
    if (!wifi_ok)
    {
        ESP_LOGW(TAG, "WiFi 未连接, UDP 不可用 — 仅本地串口输出");
    }

    /* 4. 启动 UDP 配置监听 (远程修改 ID) */
    if (wifi_ok)
    {
        node_config_start_udp_listener();
    }

    /* 5. 创建 IMU → UDP 队列 */
    g_pkt_queue = xQueueCreate(PKT_QUEUE_DEPTH, sizeof(node_packet_t));
    if (!g_pkt_queue)
    {
        ESP_LOGE(TAG, "队列创建失败，系统暂停");
        while (1) { vTaskDelay(pdMS_TO_TICKS(1000)); }
    }

    /* 6. 启动 UDP 发送任务 */
    if (xTaskCreate(udp_tx_task, "udp_tx", UDP_TASK_STACK, NULL,
                    UDP_TASK_PRIO, NULL) != pdPASS)
    {
        ESP_LOGE(TAG, "创建 UDP 任务失败，系统暂停");
        while (1) { vTaskDelay(pdMS_TO_TICKS(1000)); }
    }

    /* 7. I2C 互斥锁 */
    g_i2c_mutex = xSemaphoreCreateMutex();
    if (!g_i2c_mutex)
    {
        ESP_LOGE(TAG, "互斥锁创建失败，系统暂停");
        while (1) { vTaskDelay(pdMS_TO_TICKS(1000)); }
    }

    /* 8. I2C 总线 */
    if (i2c_bus_init() != ESP_OK)
    {
        ESP_LOGE(TAG, "I2C 总线初始化失败，系统暂停");
        while (1) { vTaskDelay(pdMS_TO_TICKS(1000)); }
    }

    /* 9. BMI270 初始化 + 配置 + 使能 */
    if (bmi270_hw_init() != BMI2_OK)
    {
        ESP_LOGE(TAG, "BMI270 初始化失败，系统暂停");
        while (1) { vTaskDelay(pdMS_TO_TICKS(1000)); }
    }

    /* 10. 启动 IMU 管道任务 */
    if (xTaskCreate(imu_task, "imu", IMU_TASK_STACK, NULL,
                    IMU_TASK_PRIO, NULL) != pdPASS)
    {
        ESP_LOGE(TAG, "创建 IMU 任务失败，系统暂停");
        while (1) { vTaskDelay(pdMS_TO_TICKS(1000)); }
    }

    ESP_LOGI(TAG, "系统就绪 — 节点%d IMU(50Hz) → UDP(%s:%d)",
             node_config_get_id(), UDP_BROADCAST_IP, UDP_PORT);
}
