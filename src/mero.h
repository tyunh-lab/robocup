#include <Arduino.h>

int melody[] = {
    165,
    220,
    330,
    494,
    659,
    880,
    1661,
};

int noteDurations[] = {
    6, 6, 6, 6, 6, 6, 2};

void playMelody(int buzzerPin)
{
    for (int i = 0; i < sizeof(melody) / sizeof(melody[0]); i++)
    {
        int duration = 1000 / noteDurations[i];
        tone(buzzerPin, melody[i], duration);

        // 周波数と長さの後に一定の遅延
        int pauseBetweenNotes = duration * 1.30;
        delay(pauseBetweenNotes);

        // ブザーを停止
        noTone(buzzerPin);
    }
}
