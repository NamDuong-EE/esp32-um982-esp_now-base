#include <unity.h>
#include <Arduino.h>
#include "Top_Lvl_Config.h"
#include "hardware/Lora_handler.h"

void setUp(void)
{
  // set stuff up here
}

void tearDown(void)
{
  // clean stuff up here
}


/*=========== TESTS ============*/
void test_loraSetup_success() {
    Serial.println("Testing loraSetup for successful initialization...");
    int result = loraSetup();
    TEST_ASSERT_EQUAL(0, result);
}

void test_loraSend_largeData() {
    Serial.println("Testing loraSend with data larger than BUFFER_SIZE...");
    Serial.println("Waiting for 10 seconds...");
    delay(10000);
    char largeData[] = "Contrary to popular belief, Lorem Ipsum is not simply random text. It has roots in a piece of classical Latin literature from 45 BC, making it over 2000 years old. Richard McClintock, a Latin professor at Hampden-Sydney College in Virginia, looked up one of the more obscure Latin words, consectetur, from a Lorem Ipsum passage, and going through the cites of the word in classical literature, discovered the undoubtable source. Lorem Ipsum comes from sections 1.10.32 and 1.10.33 of \"de Finibus Bonorum et Malorum\" (The Extremes of Good and Evil) by Cicero, written in 45 BC.\n\0";
    
    int result;
    for (int i = 1; i <= 15; i++) {
        Serial.printf("Iteration %d: ", i);
        Serial.println("Testing loraSend with data larger than BUFFER_SIZE...");
        result = loraSend(largeData, sizeof(largeData));
        delay(1000);
    }
    TEST_ASSERT_EQUAL(0, result);
}

/*=========== MAIN FUNCTIONS ============*/

void setup() {
    UNITY_BEGIN();
    Serial.begin(115200);
    delay(2000);

    // Initialize board hardware as done in production `main()`
    Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);
    pinMode(LED_PIN, OUTPUT);
    delay(2000);

    // Run the test cases
    RUN_TEST(test_loraSetup_success);
    RUN_TEST(test_loraSend_largeData);
    
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