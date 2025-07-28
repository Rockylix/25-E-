#include "filter.h"
#include <stdlib.h>
#include <math.h>

// ==== 均值滤波 ====
float DMA_U16_Filter_Average(const uint16_t* data, uint16_t len)
{
    if (len == 0) return 0;
    uint32_t sum = 0;
    for (uint16_t i = 0; i < len; i++) sum += data[i];
    return (float)sum / len * 3.3 / 4096; 
}

// ==== 中值滤波 ====
static int compare_u16(const void* a, const void* b)
{
    return ((*(uint16_t*)a) - (*(uint16_t*)b))*3.3/4096;
}

float DMA_U16_Filter_Median(const uint16_t* data, uint16_t len)
{
    if (len == 0) return 0;

    uint16_t temp[len];
    for (uint16_t i = 0; i < len; i++) temp[i] = data[i];

    qsort(temp, len, sizeof(uint16_t), compare_u16);

    if (len % 2 == 0)
        return (temp[len / 2 - 1] + temp[len / 2]) / 2.0f * 3.3 / 4096;
    else
        return (float)temp[len / 2] * 3.3 / 4096;
}

// ==== 限幅平均 ====
float DMA_U16_Filter_ClippedAverage(const uint16_t* data, uint16_t len, uint16_t limit)
{
    if (len == 0) return 0;
    uint32_t sum = data[0];
    uint16_t base = data[0];
    uint16_t count = 1;

    for (uint16_t i = 1; i < len; i++) {
        if (abs(data[i] - base) <= limit) {
            sum += data[i];
            count++;
        }
    }
    return (float)sum / count*3.3/4096;
}

// ==== 加权平均 ====
float DMA_U16_Filter_WeightedAverage(const uint16_t* data, uint16_t len)
{
    if (len == 0) return 0;
    float sum = 0;
    float weight_sum = 0;

    for (uint16_t i = 0; i < len; i++) {
        float weight = (float)(i + 1);
        sum += data[i] * weight;
        weight_sum += weight;
    }

    return sum / weight_sum * 3.3/4096;
}

// 低通滤波器（IIR）
float DMA_U16_Filter_LowPass(uint16_t input, float alpha)
{
    static float last_output = 0;
    last_output = alpha * input + (1.0f - alpha) * last_output;
    return last_output * 3.3 / 4096;
}
