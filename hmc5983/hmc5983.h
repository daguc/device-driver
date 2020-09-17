#ifndef _HMC5983_H_
#define _HMC5983_H_

//ͷ�ļ�
#include "stm32f0xx.h"
#include <stdbool.h>

//�궨��

//���ݽṹ����
//�����ṹ�嶨��
typedef struct {
	float x;
	float y;
	float z;
}vector3f_t;

//����ԭ������
extern bool hmc5983_init(void);
extern void hmc5983_update(vector3f_t* mag);
		 				    
#endif	//_HMC5983_H_

//�ļ�����
