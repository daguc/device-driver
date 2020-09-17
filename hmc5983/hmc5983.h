#ifndef _HMC5983_H_
#define _HMC5983_H_

//头文件
#include "stm32f0xx.h"
#include <stdbool.h>

//宏定义

//数据结构定义
//向量结构体定义
typedef struct {
	float x;
	float y;
	float z;
}vector3f_t;

//函数原型声明
extern bool hmc5983_init(void);
extern void hmc5983_update(vector3f_t* mag);
		 				    
#endif	//_HMC5983_H_

//文件结束
