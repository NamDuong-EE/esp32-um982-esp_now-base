#include <unity.h>
#include <Arduino.h>
#include "Top_Lvl_Config.h"
#include "hardware/Lora_handler.h"

/*=========== TESTS ============*/
void test_loraSetup_success() {
    Serial.println("Testing loraSetup for successful initialization...");
    int result = loraSetup();
    TEST_ASSERT_EQUAL(0, result);
}

void test_loraSend_success() {
    delay(10000);
    char testData[] = "Hello, LoRa!";
    int result;
    Serial.println("Testing loraSend for successful data transmission...");
    for (int i = 0; i < 2; i++) {
        result = loraSend(testData, sizeof(testData));
        TEST_ASSERT_EQUAL(0, result);
        delay(1000);
    }
}

/*=========== MAIN FUNCTIONS ============*/

void setup() {
    UNITY_BEGIN();
    Serial.begin(115200);
    delay(2000);
    // Initialize board hardware as done in production `main()`
    Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);
    pinMode(LED_PIN, OUTPUT);
    // Run the test cases
    RUN_TEST(test_loraSetup_success);
    RUN_TEST(test_loraSend_success);

    Serial.println("All tests completed. Waiting before ending the test suite.");
    for (int i = 0; i < 60; i++) {
        Serial.printf("Ending in %d seconds...\n", 60 - i);
        digitalWrite(LED_PIN, HIGH);
        delay(500);
        digitalWrite(LED_PIN, LOW);
        delay(500);
    }

    UNITY_END();
}

void loop() {

}