#include "Top_Lvl_Config.h"

#if CONNECT_USING_4G
#define GSM_CODE

#include "Prog_Config.h"
#include "hardware/Sim_handler.h"

#ifdef DUMP_AT_COMMANDS
#include <StreamDebugger.h>
static StreamDebugger debugger(SerialAT, SerialMon);
TinyGsm        modem(debugger);
#else
TinyGsm        modem(Serial);
#endif

bool startSIM() {
    int retrys = 1;
    SerialMon.println("[GSM] Khoi tao modem... So lan thu: " + String(retrys));
    #ifndef NATIVE_BUILD
    modem.restart();
    #endif
    String modemInfo = modem.getModemInfo();
    while (modemInfo.length() == 0) {
        if (retrys > 10) {
            SerialMon.println("[GSM] Lay thong tin modem that bai sau nhieu lan thu ! Khoi dong modem that bai !");
            return false;
        }
        SerialMon.println("[GSM] Lay thong tin modem that bai ! So lan thu: " 
            + String(retrys));
        delay(1000);
        #ifndef NATIVE_BUILD
        modem.restart();
        #endif
        modemInfo = modem.getModemInfo();
        retrys++;
    }
    SerialMon.print("[GSM] Khoi dong modem thanh cong.Thong tin modem: ");
    SerialMon.println(modemInfo);

    #ifndef NATIVE_BUILD
    SerialAT.begin(115200, SERIAL_8N1, RX_TO_MODEM_TX, TX_TO_MODEM_RX);
    #endif

    delay(3000);
    return true;
}

bool connectGSM() {
    uint8_t retries = 0;
    while (retries < 10) {
        SerialMon.println("[GSM] Dang ket noi mang GSM...");
        if (modem.isNetworkConnected()) {
            SerialMon.println("[GSM] Mang GSM da ket noi.");
            return true;
        }
        while (!modem.waitForNetwork(60000)) {
            SerialMon.println("[GSM] Ket noi mang GSM that bai. Dang thu lai... So lan thu: " + String(retries));
            retries++;
            delay(10000);
        }
    }
    SerialMon.println("[GSM] Ket noi mang GSM that bai sau " + String(retries) + " lan thu. Ket thuc ket noi !");
    return false;
}

#endif // GSM_CODE
