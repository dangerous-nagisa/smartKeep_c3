/**
 * @file node_config.h
 * @brief 节点 ID 配置模块 — NVS 持久存储 + UDP 远程配置
 *
 * 节点 ID (1~5) 保存在 NVS 中，一次烧录可复用所有板子：
 *   - 首次上电默认 ID=1，保存到 NVS
 *   - PC 端通过 UDP 8889 端口远程修改 ID，立即生效并持久化
 *
 * UDP 配置协议 (6 字节):
 *   请求: "SKCF" (4B) + cmd (1B) + value (1B)
 *   应答: "SKCK" (4B) + cmd (1B) + value (1B)
 *
 *   cmd=0x01  SET_ID   value=1~5
 *   cmd=0x02  GET_ID   value=忽略, 应答返回当前 ID
 */

#pragma once

#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 最大节点数 */
#define NODE_MAX_ID         5

/* UDP 配置端口 */
#define NODE_CFG_UDP_PORT   8889

/* 配置协议 */
#define NODE_CFG_MAGIC      "SKCF"  // 请求魔数
#define NODE_CFG_ACK_MAGIC  "SKCK"  // 应答魔数
#define NODE_CFG_CMD_SET_ID 0x01
#define NODE_CFG_CMD_GET_ID 0x02

/**
 * @brief 初始化节点配置 (从 NVS 加载)
 *
 * @note 必须在 nvs_flash_init() 之后调用
 * @return uint8_t 节点 ID (1~5)
 */
uint8_t node_config_init(void);

/**
 * @brief 获取当前节点 ID
 * @return uint8_t 1~5
 */
uint8_t node_config_get_id(void);

/**
 * @brief 设置节点 ID 并保存到 NVS
 * @param id 1~5
 * @return ESP_OK 成功
 */
esp_err_t node_config_set_id(uint8_t id);

/**
 * @brief 启动 UDP 配置监听任务
 *
 * 监听 UDP 8889 端口，接收 SET_ID / GET_ID 指令。
 * WiFi 连接成功后调用。
 */
void node_config_start_udp_listener(void);

#ifdef __cplusplus
}
#endif
