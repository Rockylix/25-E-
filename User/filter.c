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
    return (*(uint16_t*)a) - (*(uint16_t*)b); 
}

float DMA_U16_Filter_Median(const uint16_t* data, uint16_t len)
{
    if (len == 0) return 0;

    uint16_t temp[len];
    for (uint16_t i = 0; i < len; i++) temp[i] = data[i];

    qsort(temp, len, sizeof(uint16_t), compare_u16);

    if (len % 2 == 0)
        return (temp[len / 2 - 1] + temp[len / 2]) / 2.0f * 3.3f / 4096.0f;
    else
        return (float)temp[len / 2] * 3.3f / 4096.0f;
}

float DMA_U16_Filter_ClippedAverage_MedianBase(const uint16_t* data, uint16_t len, uint16_t limit)
{
    if (len == 0) return 0.0f;

    // 第一步：计算中值 base
    uint16_t temp[10];
    for (uint16_t i = 0; i < len; i++) temp[i] = data[i];


    // 插入排序
    for (uint16_t i = 1; i < 10; i++) {
        uint16_t key = temp[i];
        int j = i - 1;
        while (j >= 0 && temp[j] > key) {
            temp[j + 1] = temp[j];
            j--;
        }
        temp[j + 1] = key;
    }
    // 取中值（长度为10是偶数，取中间两数平均）
    uint16_t base = (temp[4] + temp[5]) / 2;

    // 第二步：限幅平均
    uint32_t sum = 0;
    uint16_t count = 0;
    for (uint16_t i = 0; i < len; i++) {
        if (abs((int)data[i] - base) <= limit) {
            sum += data[i];
            count++;
        }
    }

    if (count == 0) return (float)base * 3.3f / 4096.0f;
    return ((float)sum / (float)count) * 3.3f / 4096.0f;
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
