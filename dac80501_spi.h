#ifndef __DAC80501_SPI_H__
#define __DAC80501_SPI_H__
/*
@filename   dac80501_spi.h

@brief		基于三线制SPI的DAC80501驱动头文件，需要支持HAL库

@time		2024/08/30

@author		黎健

@version    2.0

(1)修复了当使用外部基准电压源并设置输出电压后，再次更改外部基准电压源的电压时输出电压会随之成比例变化的BUG；
(2)由于DAC80501的SPI驱动模式的特殊性，将所有参数对用户隐藏，包括基准电压和期望输出电压；
(3)屏蔽对M后缀和Z后缀两种不同型号下重置DAC芯片后输出电压不同的特性，当重置芯片时，输出电压仍然保持上一次设定的值；
(4)在dac80501_spi_conf.h中增加了打印调试日志信息到标准输出（使用printf输出）的宏开关

--------------------------------------------------------
@time		2024/08/24

@author		黎健

@version    1.0

完成了对DAC80501的驱动函数设计，并测试通过


@attention  这里SPI接口最高时钟速率为50MHz，只能向SPI写入数据而不能读，SPI接口如下：
            （1）SCLK：时钟线
            （2）SDIN: 数据输入线，相当于MOSI
            （3）SYNC#：串行数据使能，低电平有效。该信号是串行数据的帧同步信号，DAC80501的串行接口输入移位寄存器在其下降沿使能。
            注意，当SYNC#信号下降沿到来时启动操作周期，即主机可向DAC80501发送数据，SDIN一帧数据为24位。
            在发送一帧完整的数据之前若检测到SYNC#信号上升沿，则此次写操作无效。
            
            本驱动不负责初始化SPI硬件接口，也不负责初始化SYNC#信号的硬件接口，这两者应由用户参照芯片手册要求自行初始化！
            可以回调函数的形式初始化两者的硬件接口

*/
#ifdef __cplusplus
extern "C" {
#endif

//引入系统头文件
#include <stdint.h>
#include "stm32f1xx_hal.h"


/*
    （1）定义关于dac80501的寄存器信息
*/

//定义内部基准电压
#define DAC80501_INTERNAL_VREF 2.5

//定义DAC80501的最大输出电压为5.5V
#define DAC80501_MAX_VOUT 5.5

//DAC80501寄存器结构体声明
typedef union _DAC80501_Reg_NOOP    DAC80501_Reg_NOOP;
typedef union _DAC80501_Reg_DEVID   DAC80501_Reg_DEVID;
typedef union _DAC80501_Reg_SYNC    DAC80501_Reg_SYNC;
typedef union _DAC80501_Reg_CONFIG  DAC80501_Reg_CONFIG;
typedef union _DAC80501_Reg_GAIN    DAC80501_Reg_GAIN;
typedef union _DAC80501_Reg_TRIGGER DAC80501_Reg_TRIGGER;
typedef union _DAC80501_Reg_STATUS  DAC80501_Reg_STATUS;
typedef union _DAC80501_Reg_DAC     DAC80501_Reg_DAC;

//DAC80501其他设置结构体声明
typedef struct _DAC80501_Option DAC80501_Option;

/*
    (2)定义操作DAC80501时的错误类型
*/
typedef union
{
    struct
    {
        uint8_t dev     : 1; //设备不存在
        uint8_t malloc  : 1; //申请动态空间失败
        uint8_t spi     : 1; //spi接口无效 
        uint8_t sync    : 1; //sync#信号引脚无效
        uint8_t gain    : 1; //缓冲放大器增益设置有误
        uint8_t div     : 1; //基准电压源分压系数设置有误        
        uint8_t ref_volt: 1; //基准电压源电压设置小于0
        uint8_t out_volt: 1; //DAC输出电压电压超出了理论值
        
    };
    uint8_t data;
}DAC80501_Error;    

/*
    (3)定义DAC80501设备描述符
*/

typedef struct _dac80501_t dac80501_t;

struct _dac80501_t
{
    //实际可设置的寄存器指针，用于与DAC80501底层通信
    //禁止直接写下列寄存器，否则可能导致未知错误
    DAC80501_Reg_SYNC       *sync;
    DAC80501_Reg_CONFIG     *config;
    DAC80501_Reg_GAIN       *gain;
    DAC80501_Reg_TRIGGER    *trigger;
    DAC80501_Reg_DAC        *dac;
    
	//其他配置， 禁止直接写该配置结构体，否则可能导致未知错误
    DAC80501_Option *option;
    
    //SYNC信号描述
    GPIO_TypeDef*   sync_GPIO;    //SYNC#信号所属GPIO
    uint16_t        sync_BIT;     //SYNC#信号的位号
    
    //SPI接口描述符
    SPI_HandleTypeDef* hspi;
    
    //操作接口
    
    /*
        初始化DAC80501, 
        对于spi接口，注意该函数绑定spi接口，但并不负责初始化对应的SPI接口
        对于SYNC#信号来说,也同样如此
    */
    DAC80501_Error (* Init)(dac80501_t* dev,  SPI_HandleTypeDef *hspi,  GPIO_TypeDef* sync_GPIO, const uint16_t sync_BIT, double vout_default, void (*fun_callback)(void));
    
     /*
        反初始化DAC80501, 
        注意该函数绑定spi接口，但并不负责初始化对应的SPI接口
    */
    DAC80501_Error (* DeInit)(dac80501_t* dev, void (*fun_callback)(void));

    /*
        设置外部基准电压，注意调用该函数会自动禁用内部基准源
    */
    DAC80501_Error(*SetRefVolt)(dac80501_t* dev, const double ref_volt);

    /*
        设置 SYNC 寄存器的 DAC_SYNC_EN 字段
        enable:只有最低位有效；最低位为1时，DAC输出设置为响应LDAC触发而更新（同步模式）。
        当为0时，DAC输出设置为立即更新（异步模式）。
    */
    DAC80501_Error (* SetDacSync)(dac80501_t* dev, const uint8_t enable);
    
    /*
        设置内部基准电压源
        disable:只有最低位有效；最低位为0时使能内部基准源。
    */
    DAC80501_Error (* SetRefPower)(dac80501_t* dev, const uint8_t disable);
    
     /*
        设置DAC输出使能
        disable:只有最低位有效；最低位为1时，DAC处于关断模式，DAC输出通过1 kΩ内部电阻连接至GND。
    */
    DAC80501_Error (* SetDacPower)(dac80501_t* dev, const uint8_t disable);
    
    /*
        设置DAC基准电压分频系数
        div: 只能为1或2；可以将器件的基准电压（来自内部或外部基准电压源）除以div
    */
    DAC80501_Error (* SetRefDiv)(dac80501_t* dev, const uint8_t div);
    
     /*
        设置DAC内部缓冲放大器增益
        gain: 只能为1或2；为1时实际增益为1（即无输出增益），为2时实际增益为2。
    */
    DAC80501_Error (* SetBuffGain)(dac80501_t* dev, const uint8_t gain); 
    
    /*
        设置LDAC模式
        enable：只有最低位有效；最低位为1时，以同步模式同步加载DAC设定值
    */
    DAC80501_Error (* SetLDAC)(dac80501_t* dev, const uint8_t enable);
    
     /*
        软重置DAC80501芯片，DAC将恢复为默认上电状态
    */
    DAC80501_Error (* SoftReset)(dac80501_t* dev);
    
    /*
        设置DAC输出值
        vout: 期望输出的电压
        注意，调用该函数时，会根据期望输出的电压动态的调节分压比和增益系数
    */
    DAC80501_Error (* SetDacOut)(dac80501_t* dev, const double vout); 
};

/*
    (4)给出初始化DAC80501驱动的函数接口
*/

DAC80501_Error DAC80501_SPI_API_INIT(dac80501_t* dev);



#ifdef __cplusplus
}
#endif

#endif /* __DAC80501_SPI_H__ */