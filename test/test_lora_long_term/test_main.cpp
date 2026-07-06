#include "Top_Lvl_Config.h"
#include <unity.h>
#include <Arduino.h>
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

String largeData = "";

void test_loraSend_largeData() {
    Serial.println("Testing loraSend with data larger than BUFFER_SIZE...");
    if (largeData.isEmpty())
        largeData = "Contrary to popular belief, Lorem Ipsum is not simply random text. It has roots in a piece of classical Latin literature from 45 BC, making it over 2000 years old. Richard McClintock, a Latin professor at Hampden-Sydney College in Virginia, looked up one of the more obscure Latin words, consectetur, from a Lorem Ipsum passage, and going through the cites of the word in classical literature, discovered the undoubtable source. Lorem Ipsum comes from sections 1.10.32 and 1.10.33 of \"de Finibus Bonorum et Malorum\" (The Extremes of Good and Evil) by Cicero, written in 45 BC.\n\0";
    
    Serial.println("Testing loraSend with data larger than BUFFER_SIZE...");
    lora_packet_process(largeData);
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
    Serial.println("Testing loraSetup for successful initialization...");
    int result = loraSetup();
    TEST_ASSERT_EQUAL(0, result);
}

void loop() {
    if (0)
        UNITY_END();

    RUN_TEST(test_loraSend_largeData);
    delay(200);

}