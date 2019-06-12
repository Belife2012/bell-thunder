#include <Arduino.h>
#include "generalFunc.h"

int mathRandomInt(int min, int max) {
    if(min > max) {
        int temp = min;
        min = max;
        max = temp;
    }
    return min + (rand() % (max - min + 1));
}

// 对浮点数进行四舍五入输出整数
int Thunder_Round(float number) {
    int m;
    m = (int)(number + 0.5);
    return m;
}
