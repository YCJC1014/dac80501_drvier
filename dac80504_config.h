#ifndef __DAC80501_SPI_CONF__H__
#define __DAC80501_SPI_CONF__H__
/*
@filename   dac80501_spi_conf.h

@brief		基于三线制SPI的DAC80501驱动的配置头文件
            使用本驱动时，必须实现本文件所列出的宏函数

@time		2024/08/24

@author		丁鹏龙

@attention  


*/
#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "delay.h"

//打印调试信息日志开关
#define DAC80501_PRINT_DEBUG_INFO 1

//必须提供延时1us的函数,以供满足SYNC的信号时序
#define DAC80501_DELAY_1US do{delay_us(1);}while(0)

//必须提供动态申请空间的函数，以满足初始化DAC80501的需求
#define DAC80501_MALLOC(type) (type*)malloc(sizeof(type))
    
//必须提供释放动态申请空间的函数，以满足反初始化DAC80501的需求
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
