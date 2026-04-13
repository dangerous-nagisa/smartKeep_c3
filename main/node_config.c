/**
 * @file node_config.c
 * @brief 节点 ID 配置 — NVS 持久存储 + UDP 远程配置
 */

#include "node_config.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "lwip/sockets.h"

static const char *TAG = "node_cfg";

/* NVS 命名空间与键 */
#define NVS_NAMESPACE   "smartkeep"
#define NVS_KEY_NODE_ID "node_id"

/* 当前节点 ID */
static uint8_t s_node_id = 1;

/* 配置包结构 (6 字节) */
typedef struct __attribute__((packed)) {
    char     magic[4];  // "SKCF" 或 "SKCK"
    uint8_t  cmd;       // 0x01=SET_ID, 0x02=GET_ID
    uint8_t  value;     // ID 值
} node_cfg_pkt_t;

// =============================================================================
// NVS 读写
// =============================================================================

/**
 * @brief 从 NVS 加载 node_id
 * @return true 读取成功且值有效, false 需使用默认值
 */
static bool nvs_load_id(uint8_t *id)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &h) != ESP_OK) {
        return false;
    }
    uint8_t val = 0;
    esp_err_t ret = nvs_get_u8(h, NVS_KEY_NODE_ID, &val);
    nvs_close(h);

    if (ret == ESP_OK && val >= 1 && val <= NODE_MAX_ID) {
        *id = val;
        return true;
    }
    return false;
}

/**
 * @brief 保存 node_id 到 NVS
 */
static esp_err_t nvs_save_id(uint8_t id)
{
    nvs_handle_t h;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h);
    if (ret != ESP_OK) return ret;

    ret = nvs_set_u8(h, NVS_KEY_NODE_ID, id);
    if (ret == ESP_OK) {
        ret = nvs_commit(h);
    }
    nvs_close(h);
    return ret;
}

// =============================================================================
// 公开 API
// =============================================================================

uint8_t node_config_init(void)
{
    if (nvs_load_id(&s_node_id)) {
        ESP_LOGI(TAG, "NVS 加载 node_id=%d", s_node_id);
    } else {
        s_node_id = 1;
        nvs_save_id(s_node_id);
        ESP_LOGW(TAG, "NVS 无有效 ID, 默认 node_id=1 已保存");
    }
    return s_node_id;
}

uint8_t node_config_get_id(void)
{
    return s_node_id;
}

esp_err_t node_config_set_id(uint8_t id)
{
    if (id < 1 || id > NODE_MAX_ID) {
        ESP_LOGE(TAG, "无效 ID: %d (范围 1~%d)", id, NODE_MAX_ID);
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = nvs_save_id(id);
    if (ret == ESP_OK) {
        s_node_id = id;
        ESP_LOGI(TAG, "节点 ID 已更新: %d (已保存 NVS)", id);
    }
    return ret;
}

// =============================================================================
// UDP 配置监听任务
// =============================================================================

static void node_cfg_udp_task(void *arg)
{
    (void)arg;

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) {
        ESP_LOGE(TAG, "配置 socket 创建失败");
        vTaskDelete(NULL);
        return;
    }

    /* 绑定到配置端口 */
    struct sockaddr_in bind_addr = {
        .sin_family = AF_INET,
        .sin_port   = htons(NODE_CFG_UDP_PORT),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };

    if (bind(sock, (struct sockaddr *)&bind_addr, sizeof(bind_addr)) < 0) {
        ESP_LOGE(TAG, "配置端口 %d 绑定失败", NODE_CFG_UDP_PORT);
        close(sock);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "配置监听就绪  UDP :%d", NODE_CFG_UDP_PORT);

    node_cfg_pkt_t req;
    struct sockaddr_in src_addr;
    socklen_t addr_len;

    while (1) {
        addr_len = sizeof(src_addr);
        int n = recvfrom(sock, &req, sizeof(req), 0,
                         (struct sockaddr *)&src_addr, &addr_len);

        if (n != sizeof(node_cfg_pkt_t)) continue;
        if (memcmp(req.magic, NODE_CFG_MAGIC, 4) != 0) continue;

        /* 构建应答 */
        node_cfg_pkt_t ack;
        memcpy(ack.magic, NODE_CFG_ACK_MAGIC, 4);
        ack.cmd = req.cmd;

        switch (req.cmd) {
        case NODE_CFG_CMD_SET_ID:
            if (req.value >= 1 && req.value <= NODE_MAX_ID) {
                node_config_set_id(req.value);
                ack.value = req.value;
                ESP_LOGI(TAG, "远程设置 ID=%d  来自 %s",
                         req.value, inet_ntoa(src_addr.sin_addr));
            } else {
                ack.value = 0xFF; // 错误标记
                ESP_LOGW(TAG, "远程设置无效 ID=%d", req.value);
            }
            break;

        case NODE_CFG_CMD_GET_ID:
            ack.value = s_node_id;
            break;

        default:
            continue; // 未知命令，不回复
        }

        /* 发送应答 */
        sendto(sock, &ack, sizeof(ack), 0,
               (struct sockaddr *)&src_addr, addr_len);
    }
}

void node_config_start_udp_listener(void)
{
    xTaskCreate(node_cfg_udp_task, "node_cfg", 4096, NULL, 3, NULL);
}
