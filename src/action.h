void face_forward(double angle)
{
    angle = get_angle();
    while (angle <= 12.5 && angle >= -12.5)
    {
        angle = get_angle();
        if (angle >= 12.5)
        {
            rotateRight(angle / 2);
        }
        else if (angle <= -12.5)
        {
            rotateLeft(abs(angle) / 2);
        }
    }
}

void system_stop(void)
{
    Serial.println("System stop");
    stop();
    // some reset code
    SCB->AIRCR = ((0x5FA << SCB_AIRCR_VECTKEY_Pos) | SCB_AIRCR_SYSRESETREQ_Msk);
}