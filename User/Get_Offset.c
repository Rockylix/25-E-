#include "Get_Offset.h"

DCBlock_TypeDef dcblock;

void DCBlock_Init(DCBlock_TypeDef* dc, float alpha)
{
    dc->offset_estimate = 0.0f;
    dc->alpha = alpha;
}

float DCBlock_Update(DCBlock_TypeDef* dc, float v_in)
{
    dc->offset_estimate += dc->alpha * (v_in - dc->offset_estimate);
    return v_in - dc->offset_estimate;  // 返回去除偏置后的交流成分
}
