#include <Arduino.h>
#include <unity.h>

#include "Top_Lvl_Config.h"
#include "functions/NTRIP_Handler_IP.h"
#include "Prog_Config.h"
#include <WiFi.h>

extern WiFiClient ntripClient;

/*=========== SETTING UP FAKEIT ============*/

void setUp(void) {
  ArduinoFakeReset();
  ntripClient.mockReset();

  fakeit::When(OverloadedMethod(ArduinoFake(Serial), print, size_t(const char[]))).AlwaysReturn(1);
  fakeit::When(OverloadedMethod(ArduinoFake(Serial), print, size_t(int, int))).AlwaysReturn(1);
  fakeit::When(OverloadedMethod(ArduinoFake(Serial), println, size_t(const char[]))).AlwaysReturn(1);
  fakeit::When(OverloadedMethod(ArduinoFake(Serial), println, size_t(const String&))).AlwaysReturn(1);
  fakeit::When(OverloadedMethod(ArduinoFake(Serial), write, size_t(const uint8_t*, size_t))).AlwaysReturn(1);

  fakeit::When(OverloadedMethod(ArduinoFake(Function), random, long(long))).Return(1234);
  fakeit::When(Method(ArduinoFake(Function), millis)).AlwaysReturn(20001);
  fakeit::When(Method(ArduinoFake(Function), delay)).AlwaysReturn();
}

void tearDown(void) {}

/*=========== TESTS ============*/

void test_setupNTRIP_resets_connection_state(void) {
  setupNTRIP();
  TEST_ASSERT_FALSE(isNtripConnected());
}

void test_connectNTRIP_sends_header_and_accepts_icy_ok(void) {
  setupNTRIP();
  ntripClient.mockSetConnectResult(true);
  ntripClient.mockQueueResponseLine("ICY 200 OK\n");

  int rc = connectNTRIP();

  TEST_ASSERT_EQUAL(0, rc);
  TEST_ASSERT_TRUE(isNtripConnected());
  TEST_ASSERT_EQUAL(1, ntripClient.connectCallCount_);
  TEST_ASSERT_EQUAL_STRING(NTRIP_CASTER_IP, ntripClient.lastHost_.c_str());
  TEST_ASSERT_EQUAL_UINT16(NTRIP_CASTER_PORT, ntripClient.lastPort_);
  TEST_ASSERT_TRUE(ntripClient.lastPrinted_.indexOf("GET ") != -1);
}

void test_loopNTRIP_reads_rtcm_and_writes_to_serial1(void) {
  setupNTRIP();
  ntripClient.mockSetConnectResult(true);
  ntripClient.mockQueueResponseLine("ICY 200 OK\n");
  TEST_ASSERT_EQUAL(0, connectNTRIP());

  uint8_t rtcm[3] = {0xD3, 0x00, 0x13};
  ntripClient.mockSetConnected(true);
  ntripClient.mockQueueReadBytes(rtcm, sizeof(rtcm));

  int rc = loopNTRIP("$GPGGA,VALID,DATA");

  TEST_ASSERT_EQUAL(NTRIP_MODE + 4, rc);
  TEST_ASSERT_EQUAL(1, ntripClient.readCallCount_);
  fakeit::Verify(OverloadedMethod(ArduinoFake(Serial), write, size_t(const uint8_t*, size_t))).Exactly(1);
}

/*=========== MAIN FUNCTIONS ============*/

#ifndef NATIVE_BUILD
void setup() {}

void loop() {}
#else

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_setupNTRIP_resets_connection_state);
  RUN_TEST(test_connectNTRIP_sends_header_and_accepts_icy_ok);
  RUN_TEST(test_loopNTRIP_reads_rtcm_and_writes_to_serial1);
  return UNITY_END();
}
#endif