#include "config_manager.h"

#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "cJSON.h"
#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "fs_utils.h"
#include "logger.h"

// logger.c 的单条日志缓冲区较小，这里预留文件名/行号前缀空间，
// 避免 JSON 分段后仍在 logger 内部被再次截断。
#define CONFIG_LOG_CHUNK 192

user_config_t g_user_config;
static int32_t s_device_rpm = 0;

int32_t config_manager_get_device_rpm(void) {
  return s_device_rpm;
}

static bool json_positive_finite_number(const cJSON *item, float *out) {
  if (!cJSON_IsNumber(item) || !isfinite(item->valuedouble) || item->valuedouble <= 0.0) {
    return false;
  }
  if (out) {
    *out = (float)item->valuedouble;
  }
  return true;
}

static bool apply_bearing_profile(user_config_t *cfg, const cJSON *bearing) {
  if (!cfg) {
    return false;
  }

  memset(&cfg->bearing, 0, sizeof(cfg->bearing));
  if (!cJSON_IsObject(bearing)) {
    return false;
  }

  const cJSON *orders = cJSON_GetObjectItemCaseSensitive(bearing, "fault_orders");
  if (!cJSON_IsObject(orders) ||
      !json_positive_finite_number(cJSON_GetObjectItemCaseSensitive(bearing, "shaft_rpm"),
                                   &cfg->bearing.shaft_rpm) ||
      !json_positive_finite_number(cJSON_GetObjectItemCaseSensitive(orders, "bpfo"),
                                   &cfg->bearing.bpfo_order) ||
      !json_positive_finite_number(cJSON_GetObjectItemCaseSensitive(orders, "bpfi"),
                                   &cfg->bearing.bpfi_order) ||
      !json_positive_finite_number(cJSON_GetObjectItemCaseSensitive(orders, "bsf"),
                                   &cfg->bearing.bsf_order) ||
      !json_positive_finite_number(cJSON_GetObjectItemCaseSensitive(orders, "ftf"),
                                   &cfg->bearing.ftf_order)) {
    memset(&cfg->bearing, 0, sizeof(cfg->bearing));
    return false;
  }

  cfg->bearing.configured = true;
  return true;
}

// 安全拷贝字符串，保证结尾有 '\0' 且不溢出。
static void safe_copy(char *dst, size_t dst_size, const char *src) {
  if (!dst || dst_size == 0) {
    return;
  }
  if (!src) {
    dst[0] = '\0';
    return;
  }
  size_t len = strnlen(src, dst_size - 1);
  memcpy(dst, src, len);
  dst[len] = '\0';
}

static uint32_t next_pow2(uint32_t v) {
  v--;
  v |= v >> 1;
  v |= v >> 2;
  v |= v >> 4;
  v |= v >> 8;
  v |= v >> 16;
  return ++v;
}

/* 根据 rpm 与 target_rev 计算所需的 FFT 点数（向上取 2^k） */
static uint32_t calc_fft_points(uint32_t rpm, uint32_t target_rev) {
  const float fs = 26667.0f;         // IIS3DWB 固定采样率
  float f_rot = rpm / 60.0f;         // 转频 Hz
  float t_need = target_rev / f_rot; // 需要的采集时长（秒）
  uint32_t N_raw = (uint32_t)ceilf(t_need * fs);
  uint32_t N_pow2 = next_pow2(N_raw);
  if (N_pow2 > MAX_ALLOWED_POINTS) {
    LOG_WARNF("Calculated FFT points %lu exceed max %u, clipping", N_pow2,
              MAX_ALLOWED_POINTS);
    N_pow2 = MAX_ALLOWED_POINTS;
  }
  return N_pow2;
}

// 挂载 system/user 分区，为后续读写做准备。user分区包含设备身份，禁止自动格式化。
esp_err_t config_manager_init(void) {
  esp_err_t err = ESP_OK;

  // 挂载 system 分区（不允许自动格式化）
  if (!fsu_is_storage_mounted()) {
    err = fsu_mount_storage(false);
    if (err != ESP_OK) {
      LOG_ERRORF("Failed to mount system storage: %s", esp_err_to_name(err));
      return err; // 返回具体的错误码，而不是 ESP_FAIL
    }
  }
  // LOG_INFO("system storage mounted.");

  // 挂载 user 分区。挂载失败时保留原始内容，不能格式化并擦除设备SN。
  if (!fsu_is_user_mounted()) {
    err = fsu_mount_user(false);
    if (err != ESP_OK) {
      LOG_ERRORF("Failed to mount user storage: %s", esp_err_to_name(err));
      return err; // 返回具体的错误码，而不是 ESP_FAIL
    }
  }
  // LOG_INFO("user storage mounted.");
  return ESP_OK;
}

// 将 JSON 字段映射到配置结构体。
static void apply_json_to_config(user_config_t *cfg, const cJSON *root) {
  if (!cfg || !root) {
    return;
  }

  const cJSON *item = NULL;
  item = cJSON_GetObjectItemCaseSensitive(root, "api_host");
  if (cJSON_IsString(item)) {
    safe_copy(cfg->api_host, sizeof(cfg->api_host), item->valuestring);
  }

  item = cJSON_GetObjectItemCaseSensitive(root, "target_rev");
  if (cJSON_IsNumber(item)) {
    cfg->target_rev = (int32_t)item->valueint;
  } else if (cfg->target_rev == 0) {
    cfg->target_rev = 20; // default target rev
  }

  item = cJSON_GetObjectItemCaseSensitive(root, "patrol");
  if (cJSON_IsNumber(item)) {
    cfg->patrol = (int16_t)item->valuedouble;
  }

  item = cJSON_GetObjectItemCaseSensitive(root, "diagnosis");
  if (cJSON_IsNumber(item)) {
    cfg->diagnosis = (int16_t)item->valuedouble;
  }

  item = cJSON_GetObjectItemCaseSensitive(root, "range_g");
  if (cJSON_IsNumber(item)) {
    cfg->range_g = (int16_t)item->valueint;
  }

  item = cJSON_GetObjectItemCaseSensitive(root, "months");
  if (cJSON_IsNumber(item)) {
    cfg->months = (int16_t)item->valueint;
  }

  item = cJSON_GetObjectItemCaseSensitive(root, "battery");
  if (cJSON_IsNumber(item)) {
    cfg->battery = (int16_t)item->valueint;
  }

  item = cJSON_GetObjectItemCaseSensitive(root, "network");
  if (cJSON_IsNumber(item)) {
    cfg->network = (int8_t)item->valueint;
  }

  item = cJSON_GetObjectItemCaseSensitive(root, "ble");
  if (cJSON_IsBool(item)) {
    cfg->ble = cJSON_IsTrue(item);
  }

  item = cJSON_GetObjectItemCaseSensitive(root, "configured");
  if (cJSON_IsBool(item)) {
    cfg->is_configured = cJSON_IsTrue(item);
  }

  const cJSON *wifi = cJSON_GetObjectItemCaseSensitive(root, "wifi");
  if (cJSON_IsObject(wifi)) {
    const cJSON *ssid = cJSON_GetObjectItemCaseSensitive(wifi, "ssid");
    if (cJSON_IsString(ssid)) {
      safe_copy(cfg->wifi.ssid, sizeof(cfg->wifi.ssid), ssid->valuestring);
    }
    const cJSON *pass = cJSON_GetObjectItemCaseSensitive(wifi, "pass");
    if (cJSON_IsString(pass)) {
      safe_copy(cfg->wifi.pass, sizeof(cfg->wifi.pass), pass->valuestring);
    }
  }

  const cJSON *iso = cJSON_GetObjectItemCaseSensitive(root, "iso");
  if (cJSON_IsObject(iso)) {
    item = cJSON_GetObjectItemCaseSensitive(iso, "standard");
    if (cJSON_IsNumber(item)) {
      cfg->iso.standard = (int8_t)item->valueint;
    }
    item = cJSON_GetObjectItemCaseSensitive(iso, "category");
    if (cJSON_IsNumber(item)) {
      cfg->iso.category = (int8_t)item->valueint;
    }

    item = cJSON_GetObjectItemCaseSensitive(iso, "foundation");
    if (cJSON_IsNumber(item)) {
      cfg->iso.foundation = (int8_t)item->valueint;
    }
  }

  const cJSON *pos = cJSON_GetObjectItemCaseSensitive(root, "pos");
  if (cJSON_IsObject(pos)) {
    const cJSON *x = cJSON_GetObjectItemCaseSensitive(pos, "x");
    if (cJSON_IsString(x) && x->valuestring[0] != '\0') {
      cfg->pos.x = x->valuestring[0]; // 取第一个字符
    }
    const cJSON *y = cJSON_GetObjectItemCaseSensitive(pos, "y");
    if (cJSON_IsString(y) && y->valuestring[0] != '\0') {
      cfg->pos.y = y->valuestring[0]; // 取第一个字符
    }
    const cJSON *z = cJSON_GetObjectItemCaseSensitive(pos, "z");
    if (cJSON_IsString(z) && z->valuestring[0] != '\0') {
      cfg->pos.z = z->valuestring[0]; // 取第一个字符
    }
  }
}

static void parser_apply_wrapper(cJSON *root, void *ctx) {
  apply_json_to_config((user_config_t *)ctx, root);
}

// 从路径读取 JSON 并套用到配置。
static esp_err_t load_and_apply(const char *path, user_config_t *cfg) {
  return fsu_parse_json(path, parser_apply_wrapper, cfg);
}

static bool device_sn_is_valid(const char *sn) {
  if (!sn) {
    return false;
  }

  size_t len = strnlen(sn, LEN_MAX_DEVICE_ID);
  if (len == 0 || len >= LEN_MAX_DEVICE_ID) {
    return false;
  }

  for (size_t i = 0; i < len; ++i) {
    unsigned char ch = (unsigned char)sn[i];
    if (!isalnum(ch) && ch != '_' && ch != '-') {
      return false;
    }
  }
  return true;
}

static esp_err_t load_device_profile(user_config_t *cfg) {
  if (!cfg || !fsu_is_user_mounted()) {
    return ESP_ERR_INVALID_STATE;
  }

  char *json = fsu_read_file_alloc(FILE_PATH_DEVICE_PROFILE, NULL);
  if (!json) {
    LOG_ERRORF("Device profile not found: %s", FILE_PATH_DEVICE_PROFILE);
    return ESP_ERR_NOT_FOUND;
  }

  cJSON *root = cJSON_Parse(json);
  free(json);
  if (!cJSON_IsObject(root)) {
    cJSON_Delete(root);
    LOG_ERROR("Device profile is not valid JSON");
    return ESP_ERR_INVALID_RESPONSE;
  }

  const cJSON *sn = cJSON_GetObjectItemCaseSensitive(root, "sn");
  if (!cJSON_IsString(sn) || !device_sn_is_valid(sn->valuestring)) {
    cJSON_Delete(root);
    LOG_ERROR("Device profile contains an invalid SN");
    return ESP_ERR_INVALID_RESPONSE;
  }

  safe_copy(cfg->sn, sizeof(cfg->sn), sn->valuestring);

  const cJSON *device_id = cJSON_GetObjectItemCaseSensitive(root, "device_id");
  if (cJSON_IsString(device_id)) {
    safe_copy(cfg->device_id, sizeof(cfg->device_id), device_id->valuestring);
  } else {
    cfg->device_id[0] = '\0';
  }

  s_device_rpm = 0;
  const cJSON *rpm_item = cJSON_GetObjectItemCaseSensitive(root, "rpm");
  if (cJSON_IsNumber(rpm_item)) {
    s_device_rpm = (int32_t)rpm_item->valueint;
  }

  const cJSON *bearing = cJSON_GetObjectItemCaseSensitive(root, "bearing");
  if (apply_bearing_profile(cfg, bearing)) {
    LOG_INFOF("Bearing profile loaded: shaft_rpm=%.1f", cfg->bearing.shaft_rpm);
  } else if (cJSON_IsObject(bearing)) {
    LOG_WARN("Bearing profile is incomplete; bearing feature extraction disabled");
  }

  cJSON_Delete(root);
  return ESP_OK;
}

static void log_json_chunks(const char *prefix, const char *json, size_t len) {
  if (!json) {
    return;
  }

  if (prefix) {
    // LOG_DEBUG(prefix);
  }

  for (size_t i = 0; i < len; i += CONFIG_LOG_CHUNK) {
    size_t chunk = (len - i > CONFIG_LOG_CHUNK) ? CONFIG_LOG_CHUNK : (len - i);
    LOG_DEBUGF("%.*s", (int)chunk, json + i);
  }
}

// 打印指定路径的 JSON 内容（分段避免日志过长）。
static esp_err_t log_config_json(const char *path, const char *label) {
  if (!path) {
    return ESP_ERR_INVALID_ARG;
  }

  if (config_manager_init() != ESP_OK) {
    return ESP_FAIL;
  }

  size_t len = 0;
  char *json = fsu_read_file_alloc(path, &len);
  if (!json) {
    LOG_ERRORF("%s config not found: %s", label ? label : "unknown", path);
    return ESP_ERR_NOT_FOUND;
  }

  char header[64];
  snprintf(header, sizeof(header),
           "%s config (len=%u):", label ? label : "config", (unsigned)len);
  log_json_chunks(header, json, len);

  free(json);
  return ESP_OK;
}

// 打印默认配置文件内容，便于调试查看实际默认值。
esp_err_t config_manager_log_default_json(void) {
  // return log_config_json(FILE_PATH_CONFIG_DEFAULT, "Default");
  return ESP_OK;
}

esp_err_t config_manager_load(user_config_t *out_cfg) {
  if (!out_cfg) {
    return ESP_ERR_INVALID_ARG;
  }

  // 确保所需分区已挂载
  if (config_manager_init() != ESP_OK) {
    return ESP_FAIL;
  }

  // 先清零输出结构
  memset(out_cfg, 0, sizeof(*out_cfg));

  // 第一步：加载默认配置（system 分区）
  esp_err_t err = load_and_apply(FILE_PATH_CONFIG_DEFAULT, out_cfg);
  if (err != ESP_OK) {
    LOG_ERRORF("Failed to load default config: %s", FILE_PATH_CONFIG_DEFAULT);
    return err;
  }

  // 第二步：若存在用户配置，则覆盖默认配置
  if (fsu_is_user_mounted() && fsu_file_exists(FILE_PATH_CONFIG_USER)) {
    err = load_and_apply(FILE_PATH_CONFIG_USER, out_cfg);
    if (err != ESP_OK) {
      LOG_WARN("User config parse failed; using defaults");
      (void)config_manager_log_default_json();
      out_cfg->is_configured = false;
    } else {
      // 仅在用户配置解析成功时打印当前用户配置
      (void)log_config_json(FILE_PATH_CONFIG_USER, "User");
    }
  } else {
    // 没有用户配置，保持默认值并标记未配置
    // LOG_INFO("User config not found; using defaults");
    (void)config_manager_log_default_json();
    out_cfg->is_configured = false;
  }

  // 第三步：最后加载不可变的设备身份，运行配置不能覆盖SN。
  err = load_device_profile(out_cfg);
  if (err != ESP_OK) {
    out_cfg->sn[0] = '\0';
    LOG_ERRORF("Device identity unavailable: %s", esp_err_to_name(err));
    return err;
  }

  // 第四步：根据检测配置动态分配内存。RPM 不参与启动有效性判断；
  // RPM 缺失时仍为基础检测分配固定长度缓冲区。
  if (g_user_config.vib_buf) {
    heap_caps_free(g_user_config.vib_buf);
    g_user_config.vib_buf = NULL;
  }

  if (g_user_config.fft_scratch) {
    heap_caps_free(g_user_config.fft_scratch);
    g_user_config.fft_scratch = NULL;
  }

  if (g_user_config.fft_mag) {
    heap_caps_free(g_user_config.fft_mag);
    g_user_config.fft_mag = NULL;
  }

  if (g_user_config.fft_work_buf) {
    heap_caps_free(g_user_config.fft_work_buf);
    g_user_config.fft_work_buf = NULL;
  }

  // A factory-new device does not run detection before binding, so it does not
  // need capture or DSP buffers yet.
  if (out_cfg->device_id[0] == '\0') {
    out_cfg->fft_points = 0;
    LOG_INFO("Device is not bound; deferring detection buffer allocation");
    return ESP_OK;
  }

  uint32_t fft_points = DEFAULT_CAPTURE_POINTS;
  if (s_device_rpm > 0) {
    fft_points = calc_fft_points((uint32_t)s_device_rpm,
                                 (uint32_t)out_cfg->target_rev);
  } else {
    LOG_WARNF("RPM is %ld; using %u capture points. RPM-dependent features "
              "will be skipped during detection",
              (long)s_device_rpm, DEFAULT_CAPTURE_POINTS);
  }
  out_cfg->fft_points = fft_points;

  out_cfg->vib_buf = heap_caps_calloc(fft_points * 3, sizeof(float),
                                      MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  out_cfg->fft_scratch = heap_caps_malloc(fft_points * sizeof(float),
                                          MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  out_cfg->fft_mag = heap_caps_calloc(fft_points / 2, sizeof(float),
                                      MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  // 内部工作缓存
  out_cfg->fft_work_buf = heap_caps_calloc(fft_points, sizeof(float),
                                           MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);

  if (!out_cfg->vib_buf || !out_cfg->fft_scratch || !out_cfg->fft_mag ||
      !out_cfg->fft_work_buf) {
    LOG_ERROR("Failed to allocate DSP buffers in PSRAM");
    if (out_cfg->vib_buf)
      heap_caps_free(out_cfg->vib_buf);
    if (out_cfg->fft_scratch)
      heap_caps_free(out_cfg->fft_scratch);
    if (out_cfg->fft_mag)
      heap_caps_free(out_cfg->fft_mag);
    if (out_cfg->fft_work_buf)
      heap_caps_free(out_cfg->fft_work_buf);
    out_cfg->vib_buf = NULL;
    out_cfg->fft_scratch = NULL;
    out_cfg->fft_mag = NULL;
    out_cfg->fft_work_buf = NULL;
    return ESP_ERR_NO_MEM;
  }

  g_user_config = *out_cfg;
  return ESP_OK;
}

// 直接保存前端传入的原始 JSON。
esp_err_t config_manager_save_user_json(const char *json) {
  if (!json) {
    return ESP_ERR_INVALID_ARG;
  }
  char header[48];
  snprintf(header, sizeof(header),
           "file contents (len=%u):", (unsigned)strlen(json));
  log_json_chunks(header, json, strlen(json));

  if (config_manager_init() != ESP_OK) {
    return ESP_FAIL;
  }

  if (!fsu_is_user_mounted()) {
    LOG_ERROR("User SPIFFS not mounted");
    return ESP_FAIL;
  }
  LOG_DEBUGF("saving user config to %s", FILE_PATH_CONFIG_USER);
  return fsu_write_file(FILE_PATH_CONFIG_USER, json, strlen(json));
}

// 将结构体转 JSON 后写入用户分区。
esp_err_t config_manager_save_user(const user_config_t *cfg) {
  if (!cfg) {
    return ESP_ERR_INVALID_ARG;
  }

  cJSON *root = cJSON_CreateObject();
  if (!root) {
    return ESP_ERR_NO_MEM;
  }

  cJSON_AddNumberToObject(root, "months", cfg->months);
  cJSON_AddStringToObject(root, "api_host", cfg->api_host);
  cJSON_AddNumberToObject(root, "patrol", cfg->patrol);
  cJSON_AddNumberToObject(root, "diagnosis", cfg->diagnosis);
  cJSON_AddNumberToObject(root, "range_g", cfg->range_g);
  cJSON_AddNumberToObject(root, "battery", cfg->battery);
  cJSON_AddNumberToObject(root, "target_rev", cfg->target_rev);
  cJSON_AddNumberToObject(root, "network", cfg->network);
  cJSON_AddBoolToObject(root, "ble", cfg->ble);
  cJSON_AddBoolToObject(root, "configured", cfg->is_configured);

  cJSON *iso = cJSON_CreateObject();
  cJSON_AddNumberToObject(iso, "standard", cfg->iso.standard);
  cJSON_AddNumberToObject(iso, "category", cfg->iso.category);
  cJSON_AddNumberToObject(iso, "foundation", cfg->iso.foundation);
  cJSON_AddItemToObject(root, "iso", iso);

  cJSON *pos = cJSON_CreateObject();
  char x_str[2] = {cfg->pos.x, '\0'};
  char y_str[2] = {cfg->pos.y, '\0'};
  char z_str[2] = {cfg->pos.z, '\0'};
  cJSON_AddStringToObject(pos, "x", x_str);
  cJSON_AddStringToObject(pos, "y", y_str);
  cJSON_AddStringToObject(pos, "z", z_str);
  cJSON_AddItemToObject(root, "pos", pos);

  cJSON *wifi = cJSON_CreateObject();
  cJSON_AddStringToObject(wifi, "ssid", cfg->wifi.ssid);
  cJSON_AddStringToObject(wifi, "pass", cfg->wifi.pass);
  cJSON_AddItemToObject(root, "wifi", wifi);

  char *json_str = cJSON_PrintUnformatted(root);
  cJSON_Delete(root);

  if (!json_str) {
    return ESP_ERR_NO_MEM;
  }

  esp_err_t err = config_manager_save_user_json(json_str);
  free(json_str);
  return err;
}

esp_err_t config_manager_save_device_profile(const char* device_id, int32_t rpm, const cJSON* bearing_item) {
  if (!fsu_is_user_mounted()) {
    return ESP_ERR_INVALID_STATE;
  }

  char *json = fsu_read_file_alloc(FILE_PATH_DEVICE_PROFILE, NULL);
  if (!json) {
    LOG_ERRORF("Device profile not found: %s", FILE_PATH_DEVICE_PROFILE);
    return ESP_ERR_NOT_FOUND;
  }

  cJSON *root = cJSON_Parse(json);
  free(json);
  if (!cJSON_IsObject(root)) {
    cJSON_Delete(root);
    return ESP_ERR_INVALID_RESPONSE;
  }

  // Update or add device_id
  cJSON *dev_id_item = cJSON_GetObjectItemCaseSensitive(root, "device_id");
  if (dev_id_item) {
    cJSON_ReplaceItemInObjectCaseSensitive(root, "device_id", cJSON_CreateString(device_id));
  } else {
    cJSON_AddStringToObject(root, "device_id", device_id);
  }

  // Update or add rpm
  cJSON *rpm_item_local = cJSON_GetObjectItemCaseSensitive(root, "rpm");
  if (rpm_item_local) {
    cJSON_ReplaceItemInObjectCaseSensitive(root, "rpm", cJSON_CreateNumber(rpm));
  } else {
    cJSON_AddNumberToObject(root, "rpm", rpm);
  }

  // Update or add bearing
  if (bearing_item && cJSON_IsObject(bearing_item)) {
    cJSON *bearing_dup = cJSON_Duplicate(bearing_item, true);
    if (bearing_dup) {
      if (cJSON_GetObjectItemCaseSensitive(root, "bearing")) {
        cJSON_ReplaceItemInObjectCaseSensitive(root, "bearing", bearing_dup);
      } else {
        cJSON_AddItemToObject(root, "bearing", bearing_dup);
      }
    }
  } else {
    cJSON_DeleteItemFromObjectCaseSensitive(root, "bearing");
  }

  char *new_json = cJSON_PrintUnformatted(root);
  cJSON_Delete(root);

  if (!new_json) {
    return ESP_ERR_NO_MEM;
  }

  esp_err_t err = fsu_write_file(FILE_PATH_DEVICE_PROFILE, new_json, strlen(new_json));
  free(new_json);

  if (err == ESP_OK) {
    safe_copy(g_user_config.device_id, sizeof(g_user_config.device_id), device_id);
    s_device_rpm = rpm;
    (void)apply_bearing_profile(&g_user_config, bearing_item);
  }
  return err;
}

esp_err_t config_manager_save_binding_profile(const cJSON* binding_data) {
  if (!cJSON_IsObject(binding_data)) {
    return ESP_ERR_INVALID_ARG;
  }
  if (!fsu_is_user_mounted()) {
    return ESP_ERR_INVALID_STATE;
  }

  char *json = fsu_read_file_alloc(FILE_PATH_DEVICE_PROFILE, NULL);
  if (!json) {
    LOG_ERRORF("Device profile not found: %s", FILE_PATH_DEVICE_PROFILE);
    return ESP_ERR_NOT_FOUND;
  }

  cJSON *current = cJSON_Parse(json);
  free(json);
  if (!cJSON_IsObject(current)) {
    cJSON_Delete(current);
    return ESP_ERR_INVALID_RESPONSE;
  }

  const cJSON *sn = cJSON_GetObjectItemCaseSensitive(current, "sn");
  if (!cJSON_IsString(sn) || !device_sn_is_valid(sn->valuestring)) {
    cJSON_Delete(current);
    return ESP_ERR_INVALID_RESPONSE;
  }

  cJSON *updated = cJSON_CreateObject();
  if (!updated) {
    cJSON_Delete(current);
    return ESP_ERR_NO_MEM;
  }

  static const char *local_fields[] = {"sn", "version", "generated_at"};
  for (size_t i = 0; i < sizeof(local_fields) / sizeof(local_fields[0]); ++i) {
    const cJSON *item =
        cJSON_GetObjectItemCaseSensitive(current, local_fields[i]);
    if (!item) {
      continue;
    }
    cJSON *copy = cJSON_Duplicate(item, true);
    if (!copy) {
      cJSON_Delete(updated);
      cJSON_Delete(current);
      return ESP_ERR_NO_MEM;
    }
    cJSON_AddItemToObject(updated, local_fields[i], copy);
  }

  const cJSON *item = NULL;
  cJSON_ArrayForEach(item, binding_data) {
    if (!item->string ||
        strcmp(item->string, "sn") == 0 ||
        strcmp(item->string, "version") == 0 ||
        strcmp(item->string, "generated_at") == 0) {
      continue;
    }
    cJSON *copy = cJSON_Duplicate(item, true);
    if (!copy) {
      cJSON_Delete(updated);
      cJSON_Delete(current);
      return ESP_ERR_NO_MEM;
    }
    cJSON_AddItemToObject(updated, item->string, copy);
  }

  char *new_json = cJSON_PrintUnformatted(updated);
  cJSON_Delete(updated);
  cJSON_Delete(current);
  if (!new_json) {
    return ESP_ERR_NO_MEM;
  }

  esp_err_t err =
      fsu_write_file(FILE_PATH_DEVICE_PROFILE, new_json, strlen(new_json));
  free(new_json);
  if (err != ESP_OK) {
    return err;
  }

  const cJSON *device_id =
      cJSON_GetObjectItemCaseSensitive(binding_data, "device_id");
  safe_copy(g_user_config.device_id, sizeof(g_user_config.device_id),
            cJSON_IsString(device_id) ? device_id->valuestring : "");

  const cJSON *rpm_item =
      cJSON_GetObjectItemCaseSensitive(binding_data, "rpm");
  s_device_rpm =
      cJSON_IsNumber(rpm_item) ? (int32_t)rpm_item->valueint : 0;
  const cJSON *bearing =
      cJSON_GetObjectItemCaseSensitive(binding_data, "bearing");
  (void)apply_bearing_profile(&g_user_config, bearing);
  return ESP_OK;
}
