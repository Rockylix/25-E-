#ifndef __DMA_FILTER_U16_H__
#define __DMA_FILTER_U16_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// ===== DMA采样数组滤波接口（uint16_t 版本）=====

// 均值滤波
float DMA_U16_Filter_Average(const uint16_t* data, uint16_t len);

// 中值滤波
float DMA_U16_Filter_Median(const uint16_t* data, uint16_t len);

float DMA_U16_Filter_ClippedAverage_MedianBase(const uint16_t* data, uint16_t len, uint16_t limit);
// 限幅平均滤波（过滤偏差过大的点）
float DMA_U16_Filter_ClippedAverage(const uint16_t* data, uint16_t len, uint16_t limit);

// 加权平均滤波（越新的权重越大）
float DMA_U16_Filter_WeightedAverage(const uint16_t* data, uint16_t len);

float DMA_U16_Filter_LowPass(uint16_t input, float alpha);

#ifdef __cplusplus
}
#endif

#endif
