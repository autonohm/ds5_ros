#pragma once

float rgbIntToFloat(int R, int G, int B){
    int rgb_int = 0;
    rgb_int = (R << 16) + (G << 8) + B;
    return static_cast<float>(rgb_int);
}