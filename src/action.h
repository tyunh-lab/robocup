#include <Arduino.h>
#include <jyro.h>
#include <motor.h>
#include <kicker.h>

void system_stop(void)
{
    Serial.println("System stop");
    stop();
    // some reset code
    SCB->AIRCR = ((0x5FA << SCB_AIRCR_VECTKEY_Pos) | SCB_AIRCR_SYSRESETREQ_Msk);
}