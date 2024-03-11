#ifndef MANAGER_H
#define MANAGER_H

int motor_power[4] = {128, 128, 128, 128};

void set_motor(int index, int power)
{
    if (power > 255)
    {
        power = 255;
    }
    else if (power < 0)
    {
        power = 0;
    }
    motor_power[index] = power;
}

#endif