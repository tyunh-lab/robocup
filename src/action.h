// #include <jyro.h>
// #include <motor.h>

double angle;
void face_forward()
{
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