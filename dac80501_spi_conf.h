#ifndef __DAC80501_SPI_CONF__H__
#define __DAC80501_SPI_CONF__H__
/*
@filename   dac80501_spi_conf.h

@brief		����������SPI��DAC80501����������ͷ�ļ�
            ʹ�ñ�����ʱ������ʵ�ֱ��ļ����г��ĺ꺯��

@time		2024/08/24

@author		�轡

@attention  


*/
#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "delay.h"

//��ӡ������Ϣ��־����
#define DAC80501_PRINT_DEBUG_INFO 1

//�����ṩ��ʱ1us�ĺ���,�Թ�����SYNC���ź�ʱ��
#define DAC80501_DELAY_1US do{delay_us(1);}while(0)

//�����ṩ��̬����ռ�ĺ������������ʼ��DAC80501������
#define DAC80501_MALLOC(type) (type*)malloc(sizeof(type))
    
//�����ṩ�ͷŶ�̬����ռ�ĺ����������㷴��ʼ��DAC80501������
#define DAC80501_FREE(ptr)  do{\
                                if(ptr)\
                                {\
                                    free(ptr);\
                                    ptr = NULL;\
                                }\
                            }while(0)

#ifdef __cplusplus
}
#endif

#endif /* __DAC80501_SPI_CONF__H__ */