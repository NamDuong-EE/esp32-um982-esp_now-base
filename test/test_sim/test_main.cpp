#include "Top_Lvl_Config.h"
#include <unity.h>
#include <Arduino.h>
#include "Prog_Config.h"
#include "hardware/Sim_handler.h"

void setUp(void)
{
  // set stuff up here
}

void tearDown(void)
{
  // clean stuff up here
}

void test_startSIM() {
    Serial.println("Testing startSIM...");
    bool result = startSIM();
    TEST_ASSERT_TRUE(result);
}

void setup() {
    UNITY_BEGIN();
    Serial.begin(115200);
    SerialAT.begin(115200, SERIAL_8N1, RX_TO_MODEM_TX, TX_TO_MODEM_RX);
    delay(2000);

    // Initialize board hardware as done in production `main()`
    pinMode(LED_PIN, OUTPUT);
    delay(2000);

    // Run the test cases
    RUN_TEST(test_startSIM);
}

void loop() {
    // Nothing to do here, as the tests are run in `setup()`
}