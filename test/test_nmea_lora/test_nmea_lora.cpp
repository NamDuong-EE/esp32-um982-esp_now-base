#include <Arduino.h>
#include <unity.h>

#include "hardware/Lora_handler.h"
#include "functions/Nmea_Handler_LoRa.h"

/*=========== SETTING UP FAKEIT ============*/

void setUp(void) {
  ArduinoFakeReset();

  fakeit::When(OverloadedMethod(ArduinoFake(Serial), print, size_t(const char[]))).AlwaysReturn(1);
  fakeit::When(OverloadedMethod(ArduinoFake(Serial), print, size_t(int, int))).AlwaysReturn(1);
  fakeit::When(OverloadedMethod(ArduinoFake(Serial), println, size_t(const char[]))).AlwaysReturn(1);
  fakeit::When(OverloadedMethod(ArduinoFake(Serial), println, size_t(const String&))).AlwaysReturn(1);
  fakeit::When(OverloadedMethod(ArduinoFake(Serial), println, size_t(void))).AlwaysReturn(1);
  fakeit::When(OverloadedMethod(ArduinoFake(Serial), write, size_t(const uint8_t*, size_t))).AlwaysReturn(1);

  fakeit::When(OverloadedMethod(ArduinoFake(Function), random, long(long))).Return(1234);
  fakeit::When(Method(ArduinoFake(Function), delay)).AlwaysReturn();
}

void tearDown(void) {}

/*=========== TESTS ============*/

void test_downLinkDataHandle() {
    TEST_MESSAGE("Testing downLinkDataHandle with valid McpsIndication_t...");
    McpsIndication_t mcpsIndication;
    mcpsIndication.RxSlot = RX_SLOT_WIN_1;
    mcpsIndication.BufferSize = 3;
    mcpsIndication.Port = 2;
    uint8_t buffer[3] = {0x61, 0x62, 0x63};
    mcpsIndication.Buffer = buffer;

    downLinkDataHandle(&mcpsIndication);

    fakeit::Verify(OverloadedMethod(ArduinoFake(Serial), println, size_t(const String&))).Exactly(2);
    fakeit::Verify(OverloadedMethod(ArduinoFake(Serial), println, size_t(void))).Exactly(1);
}

void test_pushNmeaLoRaToGnss_success() {
    TEST_MESSAGE("Testing pushNmeaLoRaToGnss with valid McpsIndication_t...");
    McpsIndication_t mcpsIndication;
    mcpsIndication.BufferSize = 5;
    uint8_t buffer[5] = {'H', 'E', 'L', 'L', 'O'};
    mcpsIndication.Buffer = buffer;

    int result = pushNmeaLoRaToGnss(&mcpsIndication);
    TEST_ASSERT_EQUAL(0, result);

    fakeit::Verify(OverloadedMethod(ArduinoFake(Serial), write, size_t(const uint8_t*, size_t))).Exactly(1);
    fakeit::Verify(OverloadedMethod(ArduinoFake(Serial), println, size_t(const char[]))).Exactly(1);
    fakeit::Verify(OverloadedMethod(ArduinoFake(Serial), println, size_t(const String&))).Exactly(2);
}

void test_pushNmeaLoRaToGnss_zeroLengthBuffer() {
    TEST_MESSAGE("Testing pushNmeaLoRaToGnss with zero-length buffer...");
    McpsIndication_t mcpsIndication;
    mcpsIndication.BufferSize = 0;
    mcpsIndication.Buffer = nullptr;

    int result = pushNmeaLoRaToGnss(&mcpsIndication);
    TEST_ASSERT_EQUAL(-1, result);

    fakeit::Verify(OverloadedMethod(ArduinoFake(Serial), write, size_t(const uint8_t*, size_t))).Exactly(0);
    fakeit::Verify(OverloadedMethod(ArduinoFake(Serial), println, size_t(const char[]))).Exactly(1);
    fakeit::Verify(OverloadedMethod(ArduinoFake(Serial), println, size_t(const String&))).Exactly(0);
}

void test_pushNmeaLoRaToGnss_nullBuffer() {
    TEST_MESSAGE("Testing pushNmeaLoRaToGnss with non-zero-length buffer starting with a null pointer...");
    McpsIndication_t mcpsIndication;
    mcpsIndication.BufferSize = 5;
    mcpsIndication.Buffer = nullptr;

    int result = pushNmeaLoRaToGnss(&mcpsIndication);
    TEST_ASSERT_EQUAL(-2, result);

    fakeit::Verify(OverloadedMethod(ArduinoFake(Serial), write, size_t(const uint8_t*, size_t))).Exactly(0);
    fakeit::Verify(OverloadedMethod(ArduinoFake(Serial), println, size_t(const char[]))).Exactly(1);
    fakeit::Verify(OverloadedMethod(ArduinoFake(Serial), println, size_t(const String&))).Exactly(0);
}

/*=========== MAIN FUNCTIONS ============*/

#ifndef NATIVE_BUILD
void setup() {}

void loop() {}

#else

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_downLinkDataHandle);
  RUN_TEST(test_pushNmeaLoRaToGnss_success);
  RUN_TEST(test_pushNmeaLoRaToGnss_zeroLengthBuffer);
  RUN_TEST(test_pushNmeaLoRaToGnss_nullBuffer);
  return UNITY_END();
}
#endif