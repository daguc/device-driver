#ifndef __QMC5883_H__
#define __QMC5883_H__

//#include "stm32f0xx_hal.h"
#include "stm32f0xx.h"
#include <stdbool.h>

//向量结构体定义
typedef struct {
	float x;
	float y;
	float z;
}vector3f_t;

//全局声明
extern bool qmc5883_init(void);
extern void qmc5883_update(vector3f_t* mag);
#endif
