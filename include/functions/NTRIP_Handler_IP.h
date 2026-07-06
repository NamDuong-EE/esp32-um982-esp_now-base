#ifndef NTRIP_HANDLER_IP_H
#define NTRIP_HANDLER_IP_H

// ============ GẮN CÁC THƯ VIỆN CẦN THIẾT ==============

#include <Arduino.h>
#include "Top_Lvl_Config.h"

extern SemaphoreHandle_t rtcmBufferMutex;
extern SemaphoreHandle_t tcpStreamMutex;

// =============== KHAI BÁO HÀM =================

int setupNTRIP();
int loopNTRIP(String& rtcmData);
int connectNTRIP();
bool isNtripConnected(); // Thêm hàm lấy trạng thái NTRIP

extern String latestRtcm; // Biến toàn cục để lưu dữ liệu RTCM mới nhất từ NTRIP

#endif