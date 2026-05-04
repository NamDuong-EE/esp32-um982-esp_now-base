#include <Arduino.h>
#include <unity.h>

#include "functions/MQTT_Manager.h"
#include "Prog_Config.h"

/*=========== SETTING UP FAKEIT ============*/

extern int connectMQTT();

void setUp(void) {
  ArduinoFakeReset();
  mqtt.mockReset();

  fakeit::When(OverloadedMethod(ArduinoFake(Serial), print, size_t(const char[]))).AlwaysReturn(1);
  fakeit::When(OverloadedMethod(ArduinoFake(Serial), print, size_t(int, int))).AlwaysReturn(1);
  fakeit::When(OverloadedMethod(ArduinoFake(Serial), println, size_t(const char[]))).AlwaysReturn(1);
  fakeit::When(OverloadedMethod(ArduinoFake(Serial), println, size_t(const String&))).AlwaysReturn(1);

  fakeit::When(OverloadedMethod(ArduinoFake(Function), random, long(long))).Return(1234);
  fakeit::When(Method(ArduinoFake(Function), delay)).AlwaysReturn();
}

void tearDown(void) {}

/*=========== TESTS ============*/

void test_setupMQTT_sets_server_and_callback(void) {
  TEST_MESSAGE("Testing Setup MQTT");
  int rc = setupMQTT();

  TEST_MESSAGE("Testing Setup MQTT: Testing the function setupMQTT");
  TEST_ASSERT_EQUAL(0, rc);
  TEST_MESSAGE("Testing Setup MQTT: Getting server");
  TEST_ASSERT_EQUAL_STRING(MQTT_SERVER, mqtt.server_.c_str());
  TEST_MESSAGE("Testing Setup MQTT: Getting port");
  TEST_ASSERT_EQUAL_UINT16(MQTT_PORT, mqtt.port_);
}

void test_connectMQTT_subscribes_when_connect_success(void) {
  TEST_MESSAGE("Testing Connect MQTT: Simulating successful connection and subscription");
  int connectCallCount_old = mqtt.connectCallCount_;
  int subscribeCallCount_old = mqtt.subscribeCallCount_;
  
  mqtt.mockSetConnectResult(true);

  int rc = connectMQTT();

  TEST_ASSERT_EQUAL(0, rc);
  TEST_ASSERT_TRUE(mqtt.connected());
  TEST_ASSERT_EQUAL(1, mqtt.connectCallCount_ - connectCallCount_old);
  TEST_ASSERT_EQUAL(1, mqtt.subscribeCallCount_ - subscribeCallCount_old);
  TEST_ASSERT_EQUAL_STRING(TOPIC_SUB_CMD, mqtt.lastSubscribedTopic_.c_str());
}

void test_publishData_routes_to_gga_topic(void) {
  TEST_MESSAGE("Testing Publish Data...");
  int publishCallCount_old = mqtt.publishCallCount_;

  mqtt.mockSetConnected(true);

  int rc = publishData("$GGA,abc", true);

  TEST_ASSERT_EQUAL(0, rc);
  TEST_ASSERT_EQUAL(1, mqtt.publishCallCount_ - publishCallCount_old);
  TEST_ASSERT_EQUAL_STRING(TOPIC_PUB_DATA_GGA, mqtt.lastPublishedTopic_.c_str());
  TEST_ASSERT_EQUAL_STRING("$GGA,abc", mqtt.lastPublishedPayload_.c_str());
}

void test_publishData_returns_error_when_disconnected(void) {
  TEST_MESSAGE("Testing Publish Data: Simulating disconnected state");
  int publishCallCount_old = mqtt.publishCallCount_;

  mqtt.mockSetConnected(false);

  int rc = publishData("payload", false);

  TEST_ASSERT_EQUAL(-1, rc);
  TEST_ASSERT_EQUAL(0, mqtt.publishCallCount_ - publishCallCount_old);
}

void test_receive_mqtt_callback_and_forward_to_serial(void) {
  TEST_MESSAGE("Testing MQTT Callback: Simulating receiving MQTT message and forwarding to Serial");
  mqtt.mockSetConnected(true);

  char* testTopic = "tdm2402/um980/cmd";
  char* testPayload = "TEST_CMD";
  mqttCallback(testTopic, (byte*)testPayload, strlen(testPayload));

  TEST_ASSERT_EQUAL_STRING(testPayload, command.c_str());
}

/*=========== MAIN FUNCTIONS ============*/

#ifndef NATIVE_BUILD
void setup() {
  UNITY_BEGIN();
  RUN_TEST(test_setupMQTT_sets_server_and_callback);
  RUN_TEST(test_connectMQTT_subscribes_when_connect_success);
  RUN_TEST(test_publishData_routes_to_gga_topic);
  RUN_TEST(test_publishData_returns_error_when_disconnected);
  RUN_TEST(test_receive_mqtt_callback_and_forward_to_serial);
  UNITY_END();
}

void loop() {}
#else
int main() {
  UNITY_BEGIN();
  RUN_TEST(test_setupMQTT_sets_server_and_callback);
  RUN_TEST(test_connectMQTT_subscribes_when_connect_success);
  RUN_TEST(test_publishData_routes_to_gga_topic);
  RUN_TEST(test_publishData_returns_error_when_disconnected);
  RUN_TEST(test_receive_mqtt_callback_and_forward_to_serial);
  return UNITY_END();
}
#endif