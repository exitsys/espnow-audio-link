/* 内网调参面板（HTTP）。
 *
 * 三个接口：
 *   GET /             → index.html（编在 flash 里，不占 RAM）
 *   GET /api/state    → 当前状态 JSON，前端每 800ms 拉一次
 *   GET /api/set?...  → 改参数，立刻生效，返回最新状态
 *
 * 为什么用 GET 而不是 POST：浏览器地址栏直接敲就能改参，调试时少一层
 * fetch 构造；参数里没有敏感信息，也不改服务端状态机，用 GET 没有副作用问题。
 *
 * 任务绑 core0：WiFi/ESP-NOW 回调都在 core0，音频的 rx/play 在 core1。
 * HTTP 处理只在有请求时占用 CPU，且 core0 本身就是非实时侧，不碰音频时序。
 */
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "esp_http_server.h"
#include "esp_log.h"

#include "web_ui.h"

static const char *TAG = "web";

/* EMBED_TXTFILES 生成的符号 */
extern const uint8_t index_html_start[] asm("_binary_index_html_start");
extern const uint8_t index_html_end[]   asm("_binary_index_html_end");

/* ---------- 状态 JSON ---------- */

static int build_state_json(char *buf, size_t cap)
{
    alink_stats_t s;
    alink_tune_t  t;
    alink_get_stats(&s);
    alink_get_tune(&t);

    const double lostpct = (s.pkt + s.lost)
                           ? (double)s.lost * 100.0 / (double)(s.pkt + s.lost) : 0.0;

    return snprintf(buf, cap,
        "{\"q\":%ld,\"qmin\":%ld,\"qmax\":%ld,\"ratio\":%.3f,"
        "\"rate\":%lu,\"rssi\":%ld,\"lost\":%lu,\"lostpct\":%.2f,"
        "\"under\":%lu,\"pad\":%lu,\"plc\":%lu,\"over\":%lu,\"clamp\":%lu,"
        "\"jumpMax\":%ld,\"zero\":%lu,\"i2srate\":%ld,\"outrate\":%.1f,"
        "\"dbfs\":%.1f,"
        "\"tune\":{\"tq\":%ld,\"db\":%.1f,\"g\":%.1f,\"lim\":%ld}}",
        (long)s.queue, (long)s.q_min, (long)s.q_max, (double)s.ratio_pct,
        (unsigned long)s.pkt, (long)s.rssi, (unsigned long)s.lost, lostpct,
        (unsigned long)s.underrun, (unsigned long)s.pad, (unsigned long)s.plc,
        (unsigned long)s.overrun, (unsigned long)s.phase_clamp,
        (long)s.jump_max, (unsigned long)s.zero_run_max,
        (long)s.i2s_rate, (double)s.out_rate, (double)s.peak_dbfs,
        (long)t.target_q, (double)t.deadband, (double)t.gain, (long)t.adj_limit);
}

static esp_err_t send_state(httpd_req_t *req)
{
    char buf[512];
    const int n = build_state_json(buf, sizeof(buf));
    if (n <= 0 || n >= (int)sizeof(buf)) {
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "json overflow");
    }
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, buf, n);
}

/* ---------- 处理器 ---------- */

static esp_err_t h_index(httpd_req_t *req)
{
    const size_t len = (size_t)(index_html_end - index_html_start);
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, (const char *)index_html_start, len);
}

static esp_err_t h_state(httpd_req_t *req)
{
    return send_state(req);
}

static esp_err_t h_set(httpd_req_t *req)
{
    char qs[160];
    char v[24];
    alink_tune_t t;

    alink_get_tune(&t);
    if (httpd_req_get_url_query_str(req, qs, sizeof(qs)) == ESP_OK) {
        if (httpd_query_key_value(qs, "tq",  v, sizeof(v)) == ESP_OK) t.target_q  = (int32_t)atoi(v);
        if (httpd_query_key_value(qs, "db",  v, sizeof(v)) == ESP_OK) t.deadband  = (float)atof(v);
        if (httpd_query_key_value(qs, "g",   v, sizeof(v)) == ESP_OK) t.gain      = (float)atof(v);
        if (httpd_query_key_value(qs, "lim", v, sizeof(v)) == ESP_OK) t.adj_limit = (int32_t)atoi(v);

        if (alink_set_tune(&t)) {
            /* 每次都打一条：调参时对照串口最直接，且频率很低不会堵串口 */
            ESP_LOGW(TAG, "调参 TARGET=%ld 死区=%.1f 增益=%.1f 上限=%ld",
                     (long)t.target_q, (double)t.deadband,
                     (double)t.gain, (long)t.adj_limit);
        }
    }
    return send_state(req);
}

/* ---------- 启动 ---------- */

void web_ui_start(void)
{
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.core_id        = 0;          /* 和 WiFi 同核，别碰 core1 的音频任务 */
    cfg.stack_size     = 5120;
    cfg.max_uri_handlers = 8;
    cfg.lru_purge_enable = true;     /* 手机频繁刷新时回收旧连接 */
    cfg.recv_wait_timeout = 5;
    cfg.send_wait_timeout = 5;

    httpd_handle_t srv = NULL;
    esp_err_t r = httpd_start(&srv, &cfg);
    if (r != ESP_OK) {
        ESP_LOGE(TAG, "HTTP 启动失败 %s", esp_err_to_name(r));
        return;
    }

    static const httpd_uri_t u_index = { .uri = "/",          .method = HTTP_GET,  .handler = h_index };
    static const httpd_uri_t u_state = { .uri = "/api/state", .method = HTTP_GET,  .handler = h_state };
    static const httpd_uri_t u_set   = { .uri = "/api/set",   .method = HTTP_GET,  .handler = h_set   };

    httpd_register_uri_handler(srv, &u_index);
    httpd_register_uri_handler(srv, &u_state);
    httpd_register_uri_handler(srv, &u_set);

    ESP_LOGW(TAG, "调参面板就绪：连热点 alink-rx 后打开 http://192.168.4.1/");
}
