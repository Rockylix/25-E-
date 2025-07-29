#ifndef __GET__OFFSET__H__
#define __GET__OFFSET__H__
typedef struct {
    float offset_estimate;
    float alpha;  // 建议值：0.001~0.01
} DCBlock_TypeDef;

extern DCBlock_TypeDef dcblock;

void DCBlock_Init(DCBlock_TypeDef* dc, float alpha);
float DCBlock_Update(DCBlock_TypeDef* dc, float v_in);
#endif