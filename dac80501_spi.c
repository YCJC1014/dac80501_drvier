#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "dac80501_spi.h"
#include "dac80501_spi_conf.h"

/*
    （1）定义关于dac80501的寄存器信息
    注意，寄存器定义用到了位域，其地址分布与芯片手册的顺序相反
*/

//打印调试信息
#ifdef DAC80501_PRINT_DEBUG_INFO 
#define DAC80501_PRINT_DEBUG(fmt,args...) do{printf("file:%s(%d) func %s:\n", __FILE__,__LINE__,  __FUNCTION__);printf(fmt, ##args);}while(0)
#else
#define DAC80501_PRINT_DEBUG(fmt,args...) 
#endif

//检查指针非空
#define CHECK_PTR(ptr, param, field) do{\
                                    if(ptr == NULL) \
                                    { \
                                        param.field = 1;\
										DAC80501_PRINT_DEBUG("指针 %s 为空指针。\n", #ptr);\
                                        return param;\
                                    } \
                                }while(0)

//定义最大DAC值, 2^16 
#define DAC80501_MAX_DAC_DATA 65536

//定义DAC80501的寄存器列表，同时也包含其偏移地址
typedef enum _DAC80501_RegList
{
    NOOP    = 0,    //空操作寄存器
    DEVID,          //设备信息寄存器      
    SYNC,           //同步寄存器
    CONFIG,         //配置寄存器
    GAIN,           //增益寄存器
    TRIGGER,        //触发寄存器
    STATUS = 7,     //状态寄存器
    DAC             //DAC数据寄存器
}DAC80501_RegList;

//NOOP寄存器结构体字段描述
union _DAC80501_Reg_NOOP
{
    struct
    {
        uint16_t placeholder;
    };
    
    uint16_t data;
};

//DEVID寄存器结构体字段描述
//注意, SPI模式下没有用到该寄存器
union _DAC80501_Reg_DEVID
{
    struct
    {
        uint16_t  : 7;
        uint16_t rstsel : 1;
        uint16_t  : 4;
        uint16_t resolution : 3;
        uint16_t  : 1;
    }; 
    
    uint16_t data;
};

//SYNC寄存器结构体字段描述
union _DAC80501_Reg_SYNC
{
    struct
    {
        uint16_t dac_sync_en : 1;
        uint16_t : 15;
    }; 
    
    uint16_t data;
};

//CONFIG寄存器结构体字段描述
union _DAC80501_Reg_CONFIG
{
    struct
    {
        uint16_t dac_pwdwn : 1;
        uint16_t  : 7;
        uint16_t ref_pwdwn : 1;
        uint16_t  : 7;
    }; 
    
    uint16_t data;
};

//GAIN寄存器结构体字段描述
union _DAC80501_Reg_GAIN
{
    struct
    { 
        uint16_t buff_gain : 1;   
        uint16_t : 7;
        uint16_t ref_div : 1;
        uint16_t : 7;
    }; 
    
    uint16_t data;
};

//TRIGGER寄存器结构体字段描述
union _DAC80501_Reg_TRIGGER 
{
    struct
    {
        uint16_t soft_reset : 4;
        uint16_t ldac : 1;
        uint16_t : 11;
    }; 
    
    uint16_t data;
};

//STATUS寄存器结构体字段描述
union _DAC80501_Reg_STATUS
{
    struct
    {
        uint16_t ref_alarm : 1;
        uint16_t : 15;
    }; 
    
    uint16_t data;
};

//DAC寄存器结构体字段描述
union _DAC80501_Reg_DAC
{
    struct
    {
        uint16_t dac_data;
    }; 
    
    uint16_t data;
};

//额外定义其他配置结构体
struct _DAC80501_Option
{
	//参考电压数组
    double ref_volt[3];
	
	//期望输出电压
	double vout_set;
};

//定义DAC80501内部寄存器的配置常量

#define TRIGGER_SOFT_RESET  0B1010   //  重置命令码


/*
    （2）实现对DAC880501的底层通信
    注意，SPI模式下主机无法对DAC80501进行读操作
*/


//控制SYNC#信号
#define ENABLE_SYNC(dev)    do{HAL_GPIO_WritePin(dev->sync_GPIO, dev->sync_BIT, 0);DAC80501_DELAY_1US;}while(0)
#define DISABLE_SYNC(dev)   do{HAL_GPIO_WritePin(dev->sync_GPIO, dev->sync_BIT, 1);DAC80501_DELAY_1US;}while(0)


static DAC80501_Error Dac80501_SPI_Write(dac80501_t* dev, DAC80501_RegList reg, uint16_t data);

static DAC80501_Error Dac80501_SPI_Write(dac80501_t* dev, DAC80501_RegList reg, uint16_t data)
{
    DAC80501_Error error;
    error.data = 0;
    
    //若设备不存在，直接返回
    CHECK_PTR(dev, error, dev);
    
    //若设备没有绑定SPI接口, 直接返回
    CHECK_PTR(dev->hspi, error, spi);
    
    //发送数据
    uint8_t send_data[3] = {(uint8_t)reg, (data>>8) & 0xFF, data&0xFF};
    
    ENABLE_SYNC(dev);
    HAL_SPI_Transmit(dev->hspi, send_data, 3, HAL_MAX_DELAY);
    DISABLE_SYNC(dev);
    
    return error;
}



/*
    （3）实现提供给用户调用的应用层接口
*/

/*
    初始化DAC80501, 
    注意该函数绑定spi接口，但并不负责初始化对应的SPI接口
*/
static DAC80501_Error  DAC80501_Init(dac80501_t* dev, SPI_HandleTypeDef *hspi, GPIO_TypeDef* sync_GPIO, const uint16_t sync_BIT, 
	double vout_default, void (*fun_callback)(void))
{
    DAC80501_Error error;
    error.data = 0;
    
    //若设备不存在，直接返回
    CHECK_PTR(dev, error, dev);
    
    //若设备没有绑定SPI接口, 直接返回
    CHECK_PTR(hspi, error, spi);
    
    //如果SYNC#信号线无效，直接返回
    CHECK_PTR(sync_GPIO, error, sync);
	
	//如果默认输出电压小于0V或者大于内部基准电压的2倍，则报错
	if((vout_default > 2 * DAC80501_INTERNAL_VREF) || (vout_default < 0))
	{
		DAC80501_PRINT_DEBUG("The default vout(%lfV) is illegal.", vout_default);
		error.out_volt = 1;
		return error;
	}	
    
    //为结构体成员动态申请空间
    dev->sync   = DAC80501_MALLOC(DAC80501_Reg_SYNC);
    CHECK_PTR(dev->sync, error, malloc);
    
    dev->config = DAC80501_MALLOC(DAC80501_Reg_CONFIG);
    CHECK_PTR(dev->config, error, malloc);
  
    dev->gain   = DAC80501_MALLOC(DAC80501_Reg_GAIN);
    CHECK_PTR(dev->gain, error, malloc);
    
    dev->trigger= DAC80501_MALLOC(DAC80501_Reg_TRIGGER);
    CHECK_PTR(dev->trigger, error, malloc);
    
    dev->dac    = DAC80501_MALLOC(DAC80501_Reg_DAC);
    CHECK_PTR(dev->dac, error, malloc);
	
	//为其他配置结构体申请空间
	dev->option = DAC80501_MALLOC(DAC80501_Option);
    CHECK_PTR(dev->option, error, malloc);
	
	//设置默认输出电压
	dev->option->vout_set = vout_default;
	
    //绑定SYNC#信号
    dev->sync_GPIO  = sync_GPIO;
    dev->sync_BIT   = sync_BIT;
    
    //绑定SPI接口
    dev->hspi = hspi;
    
    //重置芯片
    error = dev->SoftReset(dev);
    
    //调用回调函数，用户可在回调函数中初始化相关硬件接口
    if(fun_callback != NULL)
        fun_callback();
    
    return error;
}

 /*
    反初始化DAC80501, 
    注意该函数绑定spi接口，但并不负责初始化对应的SPI接口
*/
static DAC80501_Error DAC80501_DeInit(dac80501_t* dev, void (*fun_callback)(void))
{
    DAC80501_Error error;
    error.data = 0;
    
    //若设备不存在，直接返回
    CHECK_PTR(dev, error, dev);
    
    //重置芯片
    error = dev->SoftReset(dev);
    
    //释放寄存器空间
    DAC80501_FREE(dev->sync);
    DAC80501_FREE(dev->config);
    DAC80501_FREE(dev->gain);
    DAC80501_FREE(dev->trigger);
    DAC80501_FREE(dev->dac);
	
	//释放配置结构体空间
    DAC80501_FREE(dev->option);
	
    //先将SYNC信号失效
    DISABLE_SYNC(dev);
    
    //解绑SYNC#信号
    dev->sync_GPIO  = NULL;
    dev->sync_BIT   = 0;
    
    //解绑SPI接口
    dev->hspi = NULL;
    
    //调用回调函数，用户可在回调函数中反初始化相关硬件接口
    if(fun_callback != NULL)
        fun_callback();
    
    return error;
}

  /*
        设置外部基准电压，注意调用该函数会自动禁用内部基准源
    */
static DAC80501_Error DAC80501_SetRefVolt(dac80501_t* dev, const double ref_volt)
{
    DAC80501_Error error;
    error.data = 0;
    
    //若设备不存在，直接返回
    CHECK_PTR(dev, error, dev);
    
    //若基准电压小于0，直接返回
    if(ref_volt<0)
    {
        error.ref_volt = 1;
		DAC80501_PRINT_DEBUG("The ref_volt(%lfV) is smaller than 0V.\n", ref_volt);
        return error;
    }
	
	//若基准电压大于DAC的最大供电电压，直接返回
	if(ref_volt > DAC80501_MAX_VOUT)
	{
		error.ref_volt = 1;
		DAC80501_PRINT_DEBUG("The ref_volt(%lfV) is bigger than %lfV.\n", ref_volt, DAC80501_MAX_VOUT);
        return error;
	}
	
	//基准电压就是当前设置，直接返回
	if(ref_volt == dev->option->ref_volt[1])
		return error;
    
	//使用外部基准源
	error = dev->SetRefPower(dev, 1);
	
	if(!error.data)
	{
		dev->option->ref_volt[0] = ref_volt / 2.0;
		dev->option->ref_volt[1] = ref_volt;
		dev->option->ref_volt[2] = ref_volt * 2.0;
		
		//更改外部基准电压后，再同步DAC寄存器的值
		dev->SetDacOut(dev, dev->option->vout_set);
	}
	else
		DAC80501_PRINT_DEBUG("Set ref_volt failed, error code is %d.\n", error.data);

    return error;
}


/*
    设置 SYNC 寄存器的 DAC_SYNC_EN 字段
    enable:只有最低位有效；最低位为1时，DAC输出设置为响应LDAC触发而更新（同步模式）。
    当为0时，DAC输出设置为立即更新（异步模式）。
*/
static DAC80501_Error DAC80501_SetDacSync(dac80501_t* dev, const uint8_t enable)
{
    DAC80501_Error error;
    error.data = 0;
    
    //若设备不存在，直接返回
    CHECK_PTR(dev, error, dev);
    
    //写入数据
    dev->sync->dac_sync_en = enable & 0x1;
    error.data |= Dac80501_SPI_Write(dev, SYNC, dev->sync->data).data;
    
    return error;
}        

/*
    设置内部基准电压源
    disable:只有最低位有效；最低位为0时使能内部基准源。
*/
static DAC80501_Error DAC80501_SetRefPower(dac80501_t* dev, const uint8_t disable)
{
    DAC80501_Error error;
    error.data = 0;
    
    //若设备不存在，直接返回
    CHECK_PTR(dev, error, dev);
    
    //写入数据
    dev->config->ref_pwdwn = disable & 0x1;
    error.data |= Dac80501_SPI_Write(dev, CONFIG, dev->config->data).data;
    
    //如果启用内部基准电压源，则同步修改基准电压设置
    if((error.data == 0) && (disable == 0))
    {  
		dev->option->ref_volt[0] = DAC80501_INTERNAL_VREF / 2.0;
		dev->option->ref_volt[1] = DAC80501_INTERNAL_VREF;
		dev->option->ref_volt[2] = DAC80501_INTERNAL_VREF * 2.0;
    }
    return error;
}

 /*
    设置DAC输出
    disable:只有最低位有效；最低位为1时，DAC处于关断模式，DAC输出通过1 kΩ内部电阻连接至GND。
*/
static DAC80501_Error DAC80501_SetDacPower(dac80501_t* dev, const uint8_t disable)
{
    DAC80501_Error error;
    error.data = 0;
    
    //若设备不存在，直接返回
    CHECK_PTR(dev, error, dev);
    
    //写入数据
    dev->config->dac_pwdwn = disable & 0x1;
    error.data |= Dac80501_SPI_Write(dev, CONFIG, dev->config->data).data;
    
    return error;
}

/*
    设置DAC基准电压分压系数
    div: 只能为1或2；可以将器件的基准电压（来自内部或外部基准电压源）除以div
*/
static DAC80501_Error Dac80501_SetRefDiv(dac80501_t* dev, const uint8_t div)
{
    DAC80501_Error error;
    error.data = 0;
    
    //若设备不存在，直接返回
    CHECK_PTR(dev, error, dev);
    
    //若div非法，直接返回
    if((div != 1) && (div != 2))
    {
        error.div = 1;
		DAC80501_PRINT_DEBUG("The div is only set to 1 or 2，but this is %d\n", div);
        return error;
    }
    
    //写入数据
    dev->gain->ref_div = div - 1;
    error.data |= Dac80501_SPI_Write(dev, GAIN, dev->gain->data).data;
    
    return error;
}

 /*
    设置DAC内部缓冲放大器增益
    gain: 只能为1或2；为1时实际增益为1（即无输出增益），为2时实际增益为2。
*/
static DAC80501_Error Dac80501_SetBuffGain(dac80501_t* dev, const uint8_t gain)
{
    DAC80501_Error error;
    error.data = 0;
    
    //若设备不存在，直接返回
    CHECK_PTR(dev, error, dev);
    
    //若gain非法，直接返回
    if((gain != 1) && (gain != 2))
    {
        error.gain = 1;
		DAC80501_PRINT_DEBUG("The gain is only set to 1 or 2，but this is %d\n", gain);
        return error;
    }
    
    //写入数据
    dev->gain->buff_gain = gain - 1;
    error.data |= Dac80501_SPI_Write(dev, GAIN, dev->gain->data).data;
    
    return error;
}    


 /*
    软重置DAC80501芯片，DAC将恢复为默认上电状态
*/
static DAC80501_Error Dac80501_SoftReset(dac80501_t* dev)
{
    DAC80501_Error error;
    error.data = 0;
    
    //若设备不存在，直接返回
    CHECK_PTR(dev, error, dev);
    
    //若芯片此时正在复位，保险起见先延时
    //复位以后需要至少延时1ms等待期间复位完成，这里延时1ms
    for(uint32_t i=0; i<1000; i++)
        DAC80501_DELAY_1US;
    
    //写入数据
    dev->trigger->soft_reset = TRIGGER_SOFT_RESET;
    error = Dac80501_SPI_Write(dev, TRIGGER, dev->trigger->data);
    
    //同步更新寄存器的值
    if(!error.data)
    {
        //设置参考电压为内部基准电压
		dev->option->ref_volt[0] = DAC80501_INTERNAL_VREF / 2.0;
		dev->option->ref_volt[1] = DAC80501_INTERNAL_VREF;
		dev->option->ref_volt[2] = DAC80501_INTERNAL_VREF * 2.0;
        
        //重置寄存器参数
        dev->sync->data = 0;
        dev->config->data = 0;
        dev->gain->data = 1;
        dev->trigger->data = 0;
        dev->dac->data = 0; //由于SPI模式无法读取芯片型号，这里暂时默认为0
    }
    
    //复位以后需要至少延时1ms等待期间复位完成，这里延时1ms
    for(uint32_t i=0; i<1000; i++)
        DAC80501_DELAY_1US;
	
	//同步更新DAC寄存器的值，保证复位后实际输出电压为设置的输出电压
	error = dev->SetDacOut(dev, dev->option->vout_set);
    
    return error;
}

/*
    设置DAC输出值
    dac_data: 该值将直接送入DAC数据寄存器。数据以直接二进制格式进行MSB对齐
*/
static DAC80501_Error Dac80501_SetDacOut(dac80501_t* dev, const double vout)
{
    DAC80501_Error error;
    error.data = 0;
    
    //若设备不存在，直接返回
    CHECK_PTR(dev, error, dev);
    
    //若基准电压值小于0V，直接返回
    if(dev->option->ref_volt[1] < 0)
    {
        error.ref_volt = 1;
		DAC80501_PRINT_DEBUG("The ref_volt(%lfV) is smaller than 0V.\n", vout);
        return error;
    }
    
    //若设置的DAC输出电压大于芯片所能输出最大的输出电压
    //或者依据当前基准电压，需要输出的电压大于实际可输出的最大电压，则返回
    if((vout > DAC80501_MAX_VOUT) || (vout > dev->option->ref_volt[2]))
    {
        error.out_volt = 1;
		DAC80501_PRINT_DEBUG("The expected voltage(%lfV) is bigger than %lfV or %lfV\n", 
		vout, DAC80501_MAX_VOUT, dev->option->ref_volt[2]);
        return error;
		
    }
    
	//更新设置输出电压
	dev->option->vout_set = vout;
	
    //如果按照当前配置，需要输出的电压大于实际可输出电压，则更改增益配置
    double vout_max =  dev->option->ref_volt[(!dev->gain->ref_div) + dev->gain->buff_gain];
	
    //如果输出电压大于基准电压，则增益为2
    if(vout > dev->option->ref_volt[1])
    {
        //将分压比设置为1，增益设置为2
        if(vout_max != dev->option->ref_volt[2])
        {
            dev->gain->buff_gain = 1;
            dev->gain->ref_div   = 0;
            error = Dac80501_SPI_Write(dev, GAIN, dev->gain->data);
        
            if(error.data)
                return error;
        
            vout_max = dev->option->ref_volt[2];
        }
    }
    
    //如果输出电压大于基准电压的一半，则不分压也不增益
    else if(vout > dev->option->ref_volt[0])
    {
        //将分压比设置为1，增益设置为1
        if(vout_max != dev->option->ref_volt[1])
        {
            dev->gain->buff_gain = 0;
            dev->gain->ref_div   = 0;
            error = Dac80501_SPI_Write(dev, GAIN, dev->gain->data);
        
            if(error.data)
                return error;
        
            vout_max = dev->option->ref_volt[1];
        }
    }
    
    //如果输出电压不大于基准电压的一半，自动分压
    else if(vout_max != dev->option->ref_volt[0])
    {
        //将分压比设置为2，增益设置为1
        dev->gain->buff_gain = 0;
        dev->gain->ref_div   = 1;
        error = Dac80501_SPI_Write(dev, GAIN, dev->gain->data);
        
        if(error.data)
            return error;
        
        vout_max = dev->option->ref_volt[0];
    }
        
    
    //将电压值转换为16位DAC数据
    if(vout == vout_max)
        dev->dac->dac_data = DAC80501_MAX_DAC_DATA - 1;
    else
        dev->dac->dac_data = round((vout * DAC80501_MAX_DAC_DATA) / vout_max);
    
    //写入数据
    error.data |= Dac80501_SPI_Write(dev, DAC, dev->dac->dac_data).data;
    
    DAC80501_PRINT_DEBUG("DAC Setting: DIV:%d, GAIN:%d, VOUT_MAX:%lfV, VOUT:%lfV\n", 
    dev->gain->ref_div, dev->gain->buff_gain, vout_max, vout);
    
    return error;
}    

 /*
        设置LDAC模式
        enable：只有最低位有效；最低位为1时，以同步模式同步加载DAC设定值
    */
static DAC80501_Error DAC80501_SetLDAC(dac80501_t* dev, const uint8_t enable)
{
    DAC80501_Error error;
    error.data = 0;
    
    //若设备不存在，直接返回
    CHECK_PTR(dev, error, dev);
    
    //写入数据
    dev->trigger->ldac = enable & 0x1;
    error.data |= Dac80501_SPI_Write(dev, TRIGGER, dev->trigger->data).data;
    
    return error;
}

/*
    (4)给出初始化DAC80501驱动的函数接口
*/

DAC80501_Error DAC80501_SPI_API_INIT(dac80501_t* dev)
{
    DAC80501_Error error;
    error.data = 0;
    
    //若设备不存在，直接返回
    CHECK_PTR(dev, error, dev);
    
    //绑定函数接口
    dev->Init           = DAC80501_Init;
    dev->DeInit         = DAC80501_DeInit;
    dev->SetRefVolt     = DAC80501_SetRefVolt;
    dev->SetBuffGain    = Dac80501_SetBuffGain;
    dev->SetDacOut      = Dac80501_SetDacOut;
    dev->SetDacPower    = DAC80501_SetDacPower;
    dev->SetRefPower    = DAC80501_SetRefPower;
    dev->SetDacSync     = DAC80501_SetDacSync;
    dev->SoftReset      = Dac80501_SoftReset;
    dev->SetRefDiv      = Dac80501_SetRefDiv;
    dev->SetLDAC        = DAC80501_SetLDAC;
    
    return error;
}

