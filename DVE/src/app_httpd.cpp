#include "esp_http_server.h"
#include "esp_timer.h"
#include "esp_camera.h"
#include "img_converters.h"
#include "fb_gfx.h"
#include "Arduino.h"
#include <algorithm>

#define PART_BOUNDARY "123456789000000000000987654321"
static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* _STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* _STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

httpd_handle_t stream_httpd = NULL;
httpd_handle_t camera_httpd = NULL;

float calculate_gradient(uint8_t *buf, size_t width, size_t height, size_t x, size_t y) {
    size_t idx = y * width + x;
    float dx = abs(buf[idx+1] - buf[idx-1]) / 2.0f;
    float dy = abs(buf[idx+width] - buf[idx-width]) / 2.0f;
    return sqrt(dx*dx + dy*dy);
}

float calculate_weighted_sharpness(camera_fb_t *fb) {
    if (!fb || !fb->buf || fb->len == 0) return 0;
    
    uint8_t *buf = fb->buf;
    size_t width = fb->width;
    size_t height = fb->height;
    float total_sharpness = 0;
    float total_weight = 0;
    
    // 增大采样步长，减少计算量
    size_t step_x = width / 20;  // 每行只取20个点
    size_t step_y = height / 15; // 每列只取15个点
    
    // 只关注中心区域
    size_t start_x = width / 4;
    size_t end_x = width * 3 / 4;
    size_t start_y = height / 4;
    size_t end_y = height * 3 / 4;
    
    for (size_t y = start_y; y < end_y; y += step_y) {
        for (size_t x = start_x; x < end_x; x += step_x) {
            size_t idx = y * width + x;
            
            // 简化的拉普拉斯算子
            int laplacian = abs(4 * buf[idx] - 
                              buf[idx-1] - buf[idx+1] - 
                              buf[idx-width] - buf[idx+width]);
            
            // 简化的权重计算
            float weight = 1.0;
            if (x < width/3 || x > width*2/3 || y < height/3 || y > height*2/3) {
                weight = 0.5;
            }
            
            total_sharpness += laplacian * weight;
            total_weight += weight;
        }
    }
    
    return total_weight > 0 ? total_sharpness / total_weight : 0;
}

// 修改图像质量评估函数，专注于文字清晰度
float calculate_text_sharpness(camera_fb_t *fb) {
    if (!fb || !fb->buf || fb->len == 0) return 0;
    
    uint8_t *buf = fb->buf;
    size_t width = fb->width;
    size_t height = fb->height;
    float total_sharpness = 0;
    int samples = 0;
    
    // 扩大采样区域，因为文字可能在任何位置
    size_t start_x = width / 6;
    size_t end_x = width * 5 / 6;
    size_t start_y = height / 6;
    size_t end_y = height * 5 / 6;
    
    // 减小采样步长以提高精度
    size_t step = 5;
    
    for (size_t y = start_y; y < end_y; y += step) {
        for (size_t x = start_x; x < end_x; x += step) {
            size_t idx = y * width + x;
            
            // Sobel算子检测边缘
            int gx = (-1 * buf[idx-1-width] + 1 * buf[idx+1-width] +
                     -2 * buf[idx-1] + 2 * buf[idx+1] +
                     -1 * buf[idx-1+width] + 1 * buf[idx+1+width]);
                     
            int gy = (-1 * buf[idx-1-width] - 2 * buf[idx-width] - 1 * buf[idx+1-width] +
                     1 * buf[idx-1+width] + 2 * buf[idx+width] + 1 * buf[idx+1+width]);
                     
            float gradient = sqrt(gx*gx + gy*gy);
            
            // 计算局部对比度
            uint8_t max_val = 0, min_val = 255;
            for(int dy = -1; dy <= 1; dy++) {
                for(int dx = -1; dx <= 1; dx++) {
                    uint8_t pixel = buf[idx + dy*width + dx];
                    max_val = max(max_val, pixel);
                    min_val = min(min_val, pixel);
                }
            }
            float contrast = (max_val - min_val) / 255.0f;
            
            // 综合评分，更重视高对比度区域的边缘
            float sharpness = gradient * (0.3 + 0.7 * contrast);
            total_sharpness += sharpness;
            samples++;
        }
    }
    
    return samples > 0 ? total_sharpness / samples : 0;
}

// 添加更多图像质量评估指标
float calculate_image_quality(camera_fb_t *fb) {
    if (!fb || !fb->buf || fb->len == 0) return 0;
    
    uint8_t *buf = fb->buf;
    size_t width = fb->width;
    size_t height = fb->height;
    float total_quality = 0;
    int samples = 0;
    
    // 中心区域范围（关注中心40%的区域）
    size_t start_x = width * 3 / 10;
    size_t end_x = width * 7 / 10;
    size_t start_y = height * 3 / 10;
    size_t end_y = height * 7 / 10;
    
    // 采样步长（每10个像素取一个样本）
    size_t step = 10;
    
    for (size_t y = start_y; y < end_y; y += step) {
        for (size_t x = start_x; x < end_x; x += step) {
            size_t idx = y * width + x;
            
            // 1. 局部对比度
            int local_contrast = abs(buf[idx] - buf[idx+1]) + 
                               abs(buf[idx] - buf[idx+width]);
            
            // 2. 边缘强度
            int gx = buf[idx+1] - buf[idx-1];
            int gy = buf[idx+width] - buf[idx-width];
            float edge = sqrt(gx*gx + gy*gy);
            
            // 3. 局部方差
            float local_mean = 0;
            float local_var = 0;
            for(int dy = -1; dy <= 1; dy++) {
                for(int dx = -1; dx <= 1; dx++) {
                    local_mean += buf[idx + dy*width + dx];
                }
            }
            local_mean /= 9;
            
            for(int dy = -1; dy <= 1; dy++) {
                for(int dx = -1; dx <= 1; dx++) {
                    float diff = buf[idx + dy*width + dx] - local_mean;
                    local_var += diff * diff;
                }
            }
            local_var /= 9;
            
            // 综合评分
            float quality = (local_contrast * 0.3 + edge * 0.4 + sqrt(local_var) * 0.3);
            total_quality += quality;
            samples++;
        }
    }
    
    return samples > 0 ? total_quality / samples : 0;
}

static esp_err_t stream_handler(httpd_req_t *req) {
    camera_fb_t * fb = NULL;
    esp_err_t res = ESP_OK;
    size_t _jpg_buf_len = 0;
    uint8_t * _jpg_buf = NULL;
    char * part_buf[64];
    static int64_t last_frame = 0;
    
    // 初始化相机参数
    sensor_t * s = esp_camera_sensor_get();
    if (s) {
        // 提高亮度和对比度
        s->set_brightness(s, 1);     // 适度提高亮度
        s->set_contrast(s, 2);       // 提高对比度以增强文字清晰度
        s->set_saturation(s, -1);    // 降低饱和度以减少色偏
        s->set_sharpness(s, 2);      // 增加锐度
        s->set_quality(s, 10);       // 提高图像质量
        s->set_colorbar(s, 0);       // 关闭彩条测试
        s->set_whitebal(s, 1);       // 开启自动白平衡
        s->set_gain_ctrl(s, 1);      // 开启自动增益
        s->set_exposure_ctrl(s, 1);  // 开启自动曝光
        s->set_hmirror(s, 0);        // 关闭水平镜像
        s->set_vflip(s, 0);          // 关闭垂直翻转
        s->set_awb_gain(s, 1);       // 开启自动白平衡增益
        s->set_aec2(s, 1);           // 开启自动曝光控制
        s->set_ae_level(s, 1);       // 略微提高曝光等级
        s->set_aec_value(s, 400);    // 设置较高的曝光值
        s->set_gainceiling(s, GAINCEILING_4X); // 设置增益上限
        s->set_bpc(s, 1);            // 开启坏点校正
        s->set_wpc(s, 1);            // 开启白点校正
        s->set_raw_gma(s, 1);        // 开启伽马校正
        s->set_lenc(s, 1);           // 开启镜头畸变校正
        s->set_dcw(s, 1);            // 开启下采样
        s->set_special_effect(s, 0);  // 关闭特殊效果
    }

    res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
    if(res != ESP_OK) {
        return res;
    }

    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "X-Framerate", "60");

    while(true) {
        fb = esp_camera_fb_get();
        if (!fb) {
            Serial.println("Camera capture failed");
            res = ESP_FAIL;
        } else {
            if(fb->format != PIXFORMAT_JPEG){
                bool jpeg_converted = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);
                esp_camera_fb_return(fb);
                fb = NULL;
                if(!jpeg_converted){
                    Serial.println("JPEG compression failed");
                    res = ESP_FAIL;
                }
            } else {
                _jpg_buf_len = fb->len;
                _jpg_buf = fb->buf;
            }
        }

        if(res == ESP_OK){
            res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
        }
        if(res == ESP_OK){
            size_t hlen = snprintf((char *)part_buf, 64, _STREAM_PART, _jpg_buf_len);
            res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
        }
        if(res == ESP_OK){
            res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
        }

        if(fb){
            esp_camera_fb_return(fb);
            fb = NULL;
            _jpg_buf = NULL;
        } else if(_jpg_buf){
            free(_jpg_buf);
            _jpg_buf = NULL;
        }

        if(res != ESP_OK){
            break;
        }

        int64_t fr_end = esp_timer_get_time();
        int64_t frame_time = fr_end - last_frame;
        last_frame = fr_end;
        frame_time /= 1000;
        Serial.printf("MJPG: %uB %ums (%.1ffps)\n",
            (uint32_t)(_jpg_buf_len),
            (uint32_t)frame_time, 1000.0 / (uint32_t)frame_time);
    }

    return res;
}

static esp_err_t index_handler(httpd_req_t *req){
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, 
        "<!DOCTYPE html>"
        "<html>"
        "<head>"
        "<meta charset=\"utf-8\">"
        "<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">"
        "<title>ESP32-CAM Stream</title>"
        "<style>"
        "body{font-family:Arial,Helvetica,sans-serif;background:#181818;color:#EFEFEF;font-size:16px}"
        "h2{font-size:18px}"
        ".main-content{max-width:800px;margin:0 auto;padding:20px}"
        "#stream{width:100%;max-width:800px;height:auto}"
        "</style>"
        "</head>"
        "<body>"
        "<div class=\"main-content\">"
        "<h1>ESP32-CAM Stream</h1>"
        "<img src=\"/stream\" id=\"stream\">"
        "</div>"
        "<script>"
        "document.getElementById('stream').onload = function() {"
        "    this.style.transform = 'rotate(0deg)';"
        "};"
        "</script>"
        "</body>"
        "</html>", -1);
}

void startCameraServer(){
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;
    config.ctrl_port = 32767;
    config.max_open_sockets = 12;
    config.max_uri_handlers = 16;
    config.stack_size = 8192;
    config.task_priority = tskIDLE_PRIORITY+5;
    config.core_id = 1;

    httpd_uri_t index_uri = {
        .uri       = "/",
        .method    = HTTP_GET,
        .handler   = index_handler,
        .user_ctx  = NULL
    };

    httpd_uri_t stream_uri = {
        .uri       = "/stream",
        .method    = HTTP_GET,
        .handler   = stream_handler,
        .user_ctx  = NULL
    };

    if (httpd_start(&camera_httpd, &config) == ESP_OK) {
        httpd_register_uri_handler(camera_httpd, &index_uri);
        httpd_register_uri_handler(camera_httpd, &stream_uri);
        Serial.println("HTTP server started on port 80");
    }
} 
