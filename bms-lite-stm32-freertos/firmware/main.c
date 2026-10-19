/*============================================================
 * M5-P2: active-low PB12 relay, temperature protection demonstration.
 * RTU timing candidate: TIM3 1MHz, 115200 8E1. Hardware acceptance pending.
 * Sensor Queue + OLED + Key + Alarm + Buzzer + I2C Mutex
 *
 * SysTick -> FreeRTOS
 * TIM2    -> 1MHz DS18B20 微秒时序
 *
 * INA226 与 OLED 共用 I2C1，因此 task_sample / task_display
 * 用 FreeRTOS Mutex 串行访问 I2C1。
 *============================================================*/
#include "stm32f10x.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include <stdio.h>
#include <string.h>
#include <stdint.h>

#define INA226_ADDR  0x40
#define OLED_ADDR    0x3C

/* R010 = 10000 micro-ohms. Current_LSB=250uA; CAL=0.00512/(0.00025*0.01). */
#define INA226_CAL_VALUE 2048U
#define INA226_CURRENT_LSB_UA 250L
#define INA226_SHUNT_UOHM 10000U
#define ELECTRICAL_VALID_MASK 0x007FU

#define MODBUS_BAUD_RATE 115200UL
#define MODBUS_EVEN_PARITY 1
#if MODBUS_BAUD_RATE <= 19200UL
#error This RTU timing configuration requires baud above 19200.
#endif

#define KEY_DOWN()   (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0) == Bit_RESET)
#define BEEP_ON()    GPIO_ResetBits(GPIOB, GPIO_Pin_1)   /* 低电平触发 */
#define BEEP_OFF()   GPIO_SetBits(GPIOB, GPIO_Pin_1)

/* L-trigger relay IN: PB12 FT pin, open drain; release=OFF, sink=ON. */
#define RELAY_ON()  GPIO_ResetBits(GPIOB, GPIO_Pin_12)
#define RELAY_OFF() GPIO_SetBits(GPIOB, GPIO_Pin_12)
#define RELAY_WAIT   0U
#define RELAY_NORMAL 1U
#define RELAY_HOT    2U
#define RELAY_SENSOR 3U
#define RELAY_STALE  4U
#define RELAY_MAX_AGE_MS 1500U

/* PA1 joins MAX485 DE and /RE: low=receive, high=transmit. */
#define RS485_RECEIVE() GPIO_ResetBits(GPIOA, GPIO_Pin_1)
#define RS485_TRANSMIT() GPIO_SetBits(GPIOA, GPIO_Pin_1)

#define DS18B20_PORT       GPIOB
#define DS18B20_PIN        GPIO_Pin_0
#define OW_LOW()           GPIO_ResetBits(DS18B20_PORT, DS18B20_PIN)
#define OW_RELEASE()       GPIO_SetBits(DS18B20_PORT, DS18B20_PIN)
#define OW_READ()          GPIO_ReadInputDataBit(DS18B20_PORT, DS18B20_PIN)

#define DS18B20_CMD_SKIP_ROM          0xCC
#define DS18B20_CMD_CONVERT_T         0x44
#define DS18B20_CMD_READ_SCRATCHPAD   0xBE

/*==================== P7 告警参数 ====================*/
/* DS18B20 raw 单位 = 1/16 C */
#define TEMP_ALARM_ON_RAW     ((int16_t)(30 * 16))   /* >=30.0C 进入告警 */
#define TEMP_ALARM_OFF_RAW    ((int16_t)(29 * 16))   /* <=29.0C 清除告警 */

/* 主循环约 10ms/片:
 * 50 片约 500ms 一个告警周期，其中前 10 片约 100ms 蜂鸣 */
#define ALARM_PERIOD_TICKS    50U
#define ALARM_ON_TICKS        10U

/*==================== TIM2: 1MHz 微秒时间基准 ====================*/
static void TIM2_Microsecond_Init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    TIM2->CR1 = 0;
    TIM2->PSC = 71;       /* 72MHz / 72 = 1MHz */
    TIM2->ARR = 0xFFFF;
    TIM2->EGR = TIM_EGR_UG;
    TIM2->CNT = 0;
    TIM2->CR1 |= TIM_CR1_CEN;
}

static void Delay_us_TIM2(uint16_t us)
{
    TIM2->CNT = 0;
    while ((uint16_t)TIM2->CNT < us)
        ;
}

/* 仅调度器启动前使用。 */
static void Boot_Delay_ms(uint16_t ms)
{
    while (ms--)
        Delay_us_TIM2(1000);
}

/*==================== GPIO / USART ====================*/
static void GPIO_All_Init(void)
{
    GPIO_InitTypeDef g;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |
                           RCC_APB2Periph_GPIOC, ENABLE);

    /* Preload released BEFORE enabling OD output: coil remains off at boot. */
    RELAY_OFF();
    g.GPIO_Pin = GPIO_Pin_12;
    g.GPIO_Mode = GPIO_Mode_Out_OD;
    g.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOB, &g);

    /* Preload low before enabling output: do not drive the bus at startup. */
    RS485_RECEIVE();
    g.GPIO_Pin = GPIO_Pin_1;
    g.GPIO_Mode = GPIO_Mode_Out_PP;
    g.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOA, &g);

    /* PC13 心跳灯: 推挽输出, 默认熄灭 */
    g.GPIO_Pin   = GPIO_Pin_13;
    g.GPIO_Mode  = GPIO_Mode_Out_PP;
    g.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOC, &g);
    GPIO_SetBits(GPIOC, GPIO_Pin_13);

    /* PA0 按键输入: 上拉输入 —— 【第一次读输入】
     * 不按=1(被上拉), 按下=0(被拉到 GND)。共地才有回路, GND 必须接 */
    g.GPIO_Pin   = GPIO_Pin_0;
    g.GPIO_Mode  = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &g);

    /* PB1 蜂鸣器控制: 推挽输出, 初始化为高 = 不响
     * (低电平触发模块, 上电第一时间压住, 防初始化期间误叫) */
    g.GPIO_Pin   = GPIO_Pin_1;
    g.GPIO_Mode  = GPIO_Mode_Out_PP;
    g.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOB, &g);
    BEEP_OFF();

    /* PB0: DS18B20 1-Wire DATA，开漏输出
     * ResetBits = 主动拉低；SetBits = 释放总线，由模块上拉拉高 */
    g.GPIO_Pin   = DS18B20_PIN;
    g.GPIO_Mode  = GPIO_Mode_Out_OD;
    g.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(DS18B20_PORT, &g);
    OW_RELEASE();

}

static void USART1_Init(void)
{
    USART_InitTypeDef u;
    GPIO_InitTypeDef g;
    NVIC_InitTypeDef nvic;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

    u.USART_BaudRate            = MODBUS_BAUD_RATE;
#if MODBUS_EVEN_PARITY
    /* STM32F1 word length includes parity: 9b + parity gives 8 data bits. */
    u.USART_WordLength          = USART_WordLength_9b;
    u.USART_StopBits            = USART_StopBits_1;
    u.USART_Parity              = USART_Parity_Even;
#else
    u.USART_WordLength          = USART_WordLength_8b;
    u.USART_StopBits            = USART_StopBits_2;
    u.USART_Parity              = USART_Parity_No;
#endif
    u.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    u.USART_Mode                = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(USART1, &u);
	  g.GPIO_Pin   = GPIO_Pin_9;
    g.GPIO_Mode  = GPIO_Mode_AF_PP;
    g.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &g);
    g.GPIO_Pin = GPIO_Pin_10;
    g.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    g.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &g);
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
    nvic.NVIC_IRQChannel = USART1_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 6;
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
    USART_Cmd(USART1, ENABLE);
}

int fputc(int ch, FILE *f)
{
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET)
        ;
    USART_SendData(USART1, (uint8_t)ch);
    return ch;
}

/*==================== 按键: 消抖状态机 ====================*/
/* 每 10ms 调用一次。返回 1 = 检测到一次"完整按下"(已消抖, 长按不重复计)
 * 三状态:
 *   0 空闲   : 见到低电平 -> 进 1 (可能是真按, 也可能是抖动)
 *   1 确认中 : 10ms 后仍是低 -> 确认按下, 触发并进 2
 *              已回高 -> 只是毛刺, 退回 0 (抖动在这里被吃掉)
 *   2 等释放 : 按住不放只算一次, 松手回 0                       */
static uint8_t Key_GetEvent(void)
{
    static uint8_t state = 0;
    uint8_t down = KEY_DOWN();

    switch (state) {
    case 0:
        if (down) state = 1;
        break;
    case 1:
        if (down) { state = 2; return 1; }
        state = 0;
        break;
    case 2:
        if (!down) state = 0;
        break;
    }
    return 0;
}

/* 按键音改为非阻塞(见主循环 beep_ticks):
 * 旧的 Beep(30) 用 Delay_ms 阻塞 30ms, 期间键扫停摆 —— 快速连按会被吃掉。
 * 非阻塞版: 按键事件里只开蜂鸣 + 记一个倒计时, 主循环每 10ms 片里递减, 减到 0 关闭。
 * "开启"和"关闭"分离到不同时刻执行, 中间不占用 CPU —— 这是裸机下消除阻塞的标准手法 */

/*==================== I2C1 (P2 原样) ====================*/
static void I2C1_Init(void)
{
    GPIO_InitTypeDef g;
    I2C_InitTypeDef  i;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    g.GPIO_Pin   = GPIO_Pin_6 | GPIO_Pin_7;
    g.GPIO_Mode  = GPIO_Mode_AF_OD;
    g.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &g);

    I2C_DeInit(I2C1);
    i.I2C_Mode                = I2C_Mode_I2C;
    i.I2C_DutyCycle           = I2C_DutyCycle_2;
    i.I2C_OwnAddress1         = 0x00;
    i.I2C_Ack                 = I2C_Ack_Enable;
    i.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    i.I2C_ClockSpeed          = 400000;
    I2C_Init(I2C1, &i);
    I2C_Cmd(I2C1, ENABLE);
}

/*==================== I2C Master STOP 辅助 ====================*/
/*
 * STM32F1: STOPF 是从机模式的 Stop detection。
 * 主机主动 GenerateSTOP() 后，应等待 CR1.STOP 被硬件清零。
 * 这里保留 timeout，防止真实总线故障时程序永久死锁。
 */
static uint32_t g_i2c_stop_timeout = 0;

static uint8_t I2C_MasterStop(void)
{
    uint32_t to = 100000;

    I2C_GenerateSTOP(I2C1, ENABLE);
    while ((I2C1->CR1 & I2C_CR1_STOP) != 0) {
        if (--to == 0) {
            g_i2c_stop_timeout++;
            return 0;
        }
    }
    return 1;
}

/*==================== INA226 驱动 (P3/P4 原样) ====================*/
static uint8_t ina226_write_reg(uint8_t reg, uint16_t val)
{
    uint32_t to;
    uint8_t  err = 0;

    to = 100000;
    while (I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY) == SET)
        if (--to == 0) return 0xFF;

    I2C_GenerateSTART(I2C1, ENABLE);
    to = 100000;
    while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT) != SUCCESS)
        if (--to == 0) { err = 1; goto stop; }

    I2C_Send7bitAddress(I2C1, INA226_ADDR << 1, I2C_Direction_Transmitter);
    to = 100000;
    while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) != SUCCESS)
        if (--to == 0) { err = 2; goto stop; }

    I2C_SendData(I2C1, reg);
    to = 100000;
    while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED) != SUCCESS)
        if (--to == 0) { err = 3; goto stop; }

    I2C_SendData(I2C1, (uint8_t)(val >> 8));
    to = 100000;
    while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED) != SUCCESS)
        if (--to == 0) { err = 4; goto stop; }

    I2C_SendData(I2C1, (uint8_t)(val & 0xFF));
    to = 100000;
    while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED) != SUCCESS)
        if (--to == 0) { err = 5; goto stop; }

stop:
    /* v4: Master STOP 不等待 STOPF；等待 CR1.STOP 自动清零 */
    I2C_MasterStop();
    if (I2C_GetFlagStatus(I2C1, I2C_FLAG_AF) == SET)
        I2C_ClearFlag(I2C1, I2C_FLAG_AF);
    return err;
}

static uint16_t ina226_read_reg(uint8_t reg)
{
    uint32_t to;
    uint8_t  hi, lo;

    to = 100000;
    while (I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY) == SET)
        if (--to == 0) return 0xFFFF;

    I2C_GenerateSTART(I2C1, ENABLE);
    to = 100000;
    while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT) != SUCCESS)
        if (--to == 0) return 0xFFFF;

    I2C_Send7bitAddress(I2C1, INA226_ADDR << 1, I2C_Direction_Transmitter);
    to = 100000;
    while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) != SUCCESS)
        if (--to == 0) {
            if (I2C_GetFlagStatus(I2C1, I2C_FLAG_AF) == SET)
                I2C_ClearFlag(I2C1, I2C_FLAG_AF);
            I2C_GenerateSTOP(I2C1, ENABLE);
            return 0xFFFE;
        }

    I2C_SendData(I2C1, reg);
    to = 100000;
    while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED) != SUCCESS)
        if (--to == 0) return 0xFFFF;

    I2C_GenerateSTART(I2C1, ENABLE);
    to = 100000;
    while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT) != SUCCESS)
        if (--to == 0) return 0xFFFF;

    I2C_Send7bitAddress(I2C1, INA226_ADDR << 1, I2C_Direction_Receiver);
    to = 100000;
    while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED) != SUCCESS)
        if (--to == 0) return 0xFFFF;

    I2C_AcknowledgeConfig(I2C1, ENABLE);
    to = 100000;
    while (I2C_GetFlagStatus(I2C1, I2C_FLAG_RXNE) == RESET)
        if (--to == 0) return 0xFFFF;
    hi = I2C_ReceiveData(I2C1);

    I2C_AcknowledgeConfig(I2C1, DISABLE);
    to = 100000;
    while (I2C_GetFlagStatus(I2C1, I2C_FLAG_RXNE) == RESET)
        if (--to == 0) return 0xFFFF;
    lo = I2C_ReceiveData(I2C1);

    /* v4: Master STOP 等 CR1.STOP 清零，不等待 Slave-only STOPF */
    I2C_MasterStop();
    I2C_AcknowledgeConfig(I2C1, ENABLE);
    return ((uint16_t)hi << 8) | lo;
}

/* Checked 16-bit read for signed registers: FFFF/FFFE can be valid data.
 * AN2824 two-byte receive sequence; only the short ADDR/ACK and STOP/DR
 * sequences are critical, not the polling waits. Legacy voltage read retained. */
static uint8_t ina226_read_checked(uint8_t reg, uint16_t *value)
{
    uint32_t to;
    volatile uint32_t clear;
    uint8_t hi = 0, lo = 0;
    uint8_t err = 0;
    I2C_NACKPositionConfig(I2C1, I2C_NACKPosition_Current);
    I2C_AcknowledgeConfig(I2C1, ENABLE);
    to = 100000;
    while (I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY) == SET)
        if (--to == 0) return 1;
    I2C_GenerateSTART(I2C1, ENABLE);
    to = 100000;
    while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT) != SUCCESS)
        if (--to == 0) { err = 2; goto failed; }
    I2C_Send7bitAddress(I2C1, INA226_ADDR << 1, I2C_Direction_Transmitter);
    to = 100000;
    while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) != SUCCESS)
        if (--to == 0) { err = 3; goto failed; }
    I2C_SendData(I2C1, reg);
    to = 100000;
    while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED) != SUCCESS)
        if (--to == 0) { err = 4; goto failed; }
    I2C_GenerateSTART(I2C1, ENABLE);
    to = 100000;
    while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT) != SUCCESS)
        if (--to == 0) { err = 5; goto failed; }
    I2C_NACKPositionConfig(I2C1, I2C_NACKPosition_Next);
    I2C_Send7bitAddress(I2C1, INA226_ADDR << 1, I2C_Direction_Receiver);
    to = 100000;
    /* Check ADDR alone: CheckEvent(EV6) would clear ADDR too early. */
    while (I2C_GetFlagStatus(I2C1, I2C_FLAG_ADDR) == RESET)
        if (--to == 0) { err = 6; goto failed; }
    taskENTER_CRITICAL();
    clear = I2C1->SR1;
    clear = I2C1->SR2;
    I2C_AcknowledgeConfig(I2C1, DISABLE);
    taskEXIT_CRITICAL();
    (void)clear;
    to = 100000;
    while (I2C_GetFlagStatus(I2C1, I2C_FLAG_BTF) == RESET)
        if (--to == 0) { err = 7; goto failed; }
    taskENTER_CRITICAL();
    I2C_GenerateSTOP(I2C1, ENABLE);
    hi = I2C_ReceiveData(I2C1);
    lo = I2C_ReceiveData(I2C1);
    taskEXIT_CRITICAL();
    to = 100000;
    while ((I2C1->CR1 & I2C_CR1_STOP) != 0U) {
        if (--to == 0) { g_i2c_stop_timeout++; err = 8; break; }
    }
    I2C_NACKPositionConfig(I2C1, I2C_NACKPosition_Current);
    I2C_AcknowledgeConfig(I2C1, ENABLE);
    if (err == 0U) *value = ((uint16_t)hi << 8) | lo;
    return err;
failed:
    (void)I2C_MasterStop();
    if (I2C_GetFlagStatus(I2C1, I2C_FLAG_AF) == SET)
        I2C_ClearFlag(I2C1, I2C_FLAG_AF);
    I2C_NACKPositionConfig(I2C1, I2C_NACKPosition_Current);
    I2C_AcknowledgeConfig(I2C1, ENABLE);
    return err;
}

/*==================== 1-Wire 基础时序 ====================*/

/*
 * Reset + Presence
 *
 * 返回:
 *   0 = Presence 成功
 *   1 = 没检测到器件
 *   2 = Reset 前总线已经被拉低
 */
static uint8_t OW_ResetAndPresence(void)
{
    uint8_t presence;

    OW_RELEASE();
    Delay_us_TIM2(10);

    if (OW_READ() == Bit_RESET)
        return 2;

    OW_LOW();
    Delay_us_TIM2(500);

    taskENTER_CRITICAL();
    OW_RELEASE();
    Delay_us_TIM2(70);
    presence = (OW_READ() == Bit_RESET) ? 1U : 0U;
    taskEXIT_CRITICAL();

    Delay_us_TIM2(430);

    return presence ? 0U : 1U;
}

/*
 * 写 1 bit
 *
 * 1-Wire 每个时隙至少约 60us。
 *
 * 写 1:
 *   拉低很短时间 -> 快速释放
 *
 * 写 0:
 *   大部分时隙保持低电平
 */
static void OW_WriteBit(uint8_t bit)
{
    taskENTER_CRITICAL();

    if (bit) {
        OW_LOW();
        Delay_us_TIM2(6);
        OW_RELEASE();
        Delay_us_TIM2(64);
    }
    else {
        OW_LOW();
        Delay_us_TIM2(60);
        OW_RELEASE();
        Delay_us_TIM2(10);
    }

    taskEXIT_CRITICAL();
}

/*
 * 读 1 bit
 *
 * Master 先用一个很短的低脉冲开启读时隙，
 * 然后释放 DATA，在时隙前段采样。
 */
static uint8_t OW_ReadBit(void)
{
    uint8_t bit;

    taskENTER_CRITICAL();

    OW_LOW();
    Delay_us_TIM2(3);
    OW_RELEASE();
    Delay_us_TIM2(10);

    bit = (OW_READ() == Bit_SET) ? 1U : 0U;

    Delay_us_TIM2(53);

    taskEXIT_CRITICAL();

    return bit;
}

/* DS18B20 按 LSB first 发送/接收字节 */
static void OW_WriteByte(uint8_t data)
{
    uint8_t i;

    for (i = 0; i < 8; i++) {
        OW_WriteBit(data & 0x01);
        data >>= 1;
    }
}

static uint8_t OW_ReadByte(void)
{
    uint8_t i;
    uint8_t data = 0;

    for (i = 0; i < 8; i++) {
        if (OW_ReadBit())
            data |= (1U << i);
    }

    return data;
}

/*==================== DS18B20 高层操作 ====================*/

/*
 * 启动一次温度转换
 *
 * 单总线上现在只有一个 DS18B20，所以使用 Skip ROM (0xCC)。
 */
static uint8_t DS18B20_StartConversion(void)
{
    uint8_t r;

    r = OW_ResetAndPresence();
    if (r != 0)
        return r;

    OW_WriteByte(DS18B20_CMD_SKIP_ROM);
    OW_WriteByte(DS18B20_CMD_CONVERT_T);

    return 0;
}

/*
 * 读取完整 9-byte Scratchpad
 *
 * byte 0: Temperature LSB
 * byte 1: Temperature MSB
 * ...
 * byte 8: CRC
 */
static uint8_t DS18B20_ReadScratchpad(uint8_t data[9])
{
    uint8_t i;
    uint8_t r;

    r = OW_ResetAndPresence();
    if (r != 0)
        return r;

    OW_WriteByte(DS18B20_CMD_SKIP_ROM);
    OW_WriteByte(DS18B20_CMD_READ_SCRATCHPAD);

    for (i = 0; i < 9; i++)
        data[i] = OW_ReadByte();

    return 0;
}

/*
 * Dallas/Maxim 1-Wire CRC-8
 * 多项式 x^8 + x^5 + x^4 + 1
 * LSB-first 实现中使用 0x8C。
 */
static uint8_t DS18B20_CRC8(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0;
    uint8_t i, j;
    uint8_t mix;

    for (i = 0; i < len; i++) {
        uint8_t inbyte = data[i];

        for (j = 0; j < 8; j++) {
            mix = (crc ^ inbyte) & 0x01;
            crc >>= 1;

            if (mix)
                crc ^= 0x8C;

            inbyte >>= 1;
        }
    }

    return crc;
}


/*==================== OLED 驱动 (P4 原样) ====================*/
static uint8_t oled_ram[8][128];
static uint8_t oled_sent[8][128];

static void oled_start_write(uint8_t ctrl)
{
    uint32_t to;

    to = 100000;
    while (I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY) == SET)
        if (--to == 0) return;

    I2C_GenerateSTART(I2C1, ENABLE);
    to = 100000;
    while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT) != SUCCESS)
        if (--to == 0) return;

    I2C_Send7bitAddress(I2C1, OLED_ADDR << 1, I2C_Direction_Transmitter);
    to = 100000;
    while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) != SUCCESS)
        if (--to == 0) { I2C_GenerateSTOP(I2C1, ENABLE); return; }

    I2C_SendData(I2C1, ctrl);
    to = 100000;
    while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED) != SUCCESS)
        if (--to == 0) return;
}

static void oled_write_byte(uint8_t b)
{
    uint32_t to = 100000;

    I2C_SendData(I2C1, b);
    while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED) != SUCCESS)
        if (--to == 0) return;
}

static void oled_stop(void)
{
    /* v4 关键修复:
     * STOPF 是 Slave mode 的停止检测标志，Master GenerateSTOP 后不会按这里
     * 期待的方式置位。旧 v3 因此每次 STOP 都可能把 100000 次 timeout 跑满。
     * Master 正确的完成条件之一是 CR1.STOP 被硬件自动清零。 */
    I2C_MasterStop();
}

static void OLED_WriteCmd(uint8_t cmd)
{
    oled_start_write(0x00);
    oled_write_byte(cmd);
    oled_stop();
}

static void OLED_Init(void)
{
    Boot_Delay_ms(120);

    OLED_WriteCmd(0xAE);
    OLED_WriteCmd(0xD5); OLED_WriteCmd(0x80);
    OLED_WriteCmd(0xA8); OLED_WriteCmd(0x3F);
    OLED_WriteCmd(0xD3); OLED_WriteCmd(0x00);
    OLED_WriteCmd(0x40);
    OLED_WriteCmd(0x8D); OLED_WriteCmd(0x14);
    OLED_WriteCmd(0x20); OLED_WriteCmd(0x02);
    OLED_WriteCmd(0xA1);
    OLED_WriteCmd(0xC8);
    OLED_WriteCmd(0xDA); OLED_WriteCmd(0x12);
    OLED_WriteCmd(0x81); OLED_WriteCmd(0xCF);
    OLED_WriteCmd(0xD9); OLED_WriteCmd(0xF1);
    OLED_WriteCmd(0xDB); OLED_WriteCmd(0x40);
    OLED_WriteCmd(0xA4);
    OLED_WriteCmd(0xA6);
    OLED_WriteCmd(0xAF);
}

static void OLED_Refresh(void)
{
    uint8_t p, x, dirty;

    for (p = 0; p < 8; p++) {
        dirty = 0;
        for (x = 0; x < 128; x++) {
            if (oled_ram[p][x] != oled_sent[p][x]) { dirty = 1; break; }
        }
        if (!dirty)
            continue;

        /* 【v3 优化】页址+列址 3 个命令合并为 1 笔事务(Co=0 时后续字节全是命令),
         * 每页从 4 笔事务减到 2 笔 —— 事务是有开销的(START/地址/STOP), 能省则省 */
        oled_start_write(0x00);
        oled_write_byte(0xB0 | p);
        oled_write_byte(0x00);
        oled_write_byte(0x10);
        oled_stop();

        oled_start_write(0x40);
        for (x = 0; x < 128; x++) {
            oled_write_byte(oled_ram[p][x]);
            oled_sent[p][x] = oled_ram[p][x];
        }
        oled_stop();
    }
}

/*==================== 8x16 字库 (Consolas 13px, P4 原样) ====================*/
static const unsigned char F8X16[][16] = {
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,  /* ' ' 0x20 */
    0x00,0x00,0x00,0xF0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x1B,0x18,0x00,0x00,0x00,  /* '!' 0x21 */
    0x00,0x70,0x70,0x00,0x70,0x70,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,  /* '"' 0x22 */
    0x80,0x80,0xE0,0x80,0x80,0xE0,0x80,0x00,0x04,0x1E,0x05,0x04,0x1E,0x05,0x04,0x00,  /* '#' 0x23 */
    0x00,0xC0,0x20,0xF0,0x20,0x00,0x00,0x00,0x00,0x10,0x79,0x17,0x12,0x0C,0x00,0x00,  /* '$' 0x24 */
    0xF0,0x90,0xF0,0x00,0xC0,0x20,0x10,0x00,0x10,0x08,0x06,0x01,0x1E,0x12,0x1E,0x00,  /* '%' 0x25 */
    0x00,0xE0,0x10,0x90,0xE0,0x00,0x00,0x00,0x0C,0x12,0x11,0x17,0x1C,0x1E,0x10,0x00,  /* '&' 0x26 */
    0x00,0x00,0x00,0x70,0x70,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,  /* ''' 0x27 */
    0x00,0x00,0x80,0x60,0x10,0x00,0x00,0x00,0x00,0x00,0x1F,0x60,0x80,0x00,0x00,0x00,  /* '(' 0x28 */
    0x00,0x00,0x10,0x60,0x80,0x00,0x00,0x00,0x00,0x00,0x80,0x60,0x1F,0x00,0x00,0x00,  /* ')' 0x29 */
    0x00,0xA0,0x40,0xF0,0x40,0xA0,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x00,  /* '*' 0x2A */
    0x00,0x00,0x00,0xC0,0x00,0x00,0x00,0x00,0x02,0x02,0x02,0x1F,0x02,0x02,0x02,0x00,  /* '+' 0x2B */
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x40,0x78,0x38,0x00,0x00,0x00,  /* ',' 0x2C */
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x02,0x02,0x02,0x02,0x00,0x00,  /* '-' 0x2D */
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x18,0x18,0x00,0x00,0x00,  /* '.' 0x2E */
    0x00,0x00,0x00,0x00,0xE0,0x10,0x00,0x00,0x00,0x20,0x1C,0x03,0x00,0x00,0x00,0x00,  /* '/' 0x2F */
    0x00,0x80,0x60,0x20,0x20,0xE0,0xC0,0x00,0x00,0x0F,0x1C,0x12,0x11,0x18,0x07,0x00,  /* '0' 0x30 */
    0x00,0x40,0x20,0xE0,0x00,0x00,0x00,0x00,0x00,0x10,0x10,0x1F,0x10,0x10,0x00,0x00,  /* '1' 0x31 */
    0x00,0x40,0x20,0x20,0x20,0xC0,0x00,0x00,0x00,0x10,0x18,0x14,0x12,0x11,0x00,0x00,  /* '2' 0x32 */
    0x00,0x20,0x20,0x20,0x20,0xC0,0x00,0x00,0x00,0x10,0x11,0x11,0x11,0x0E,0x00,0x00,  /* '3' 0x33 */
    0x00,0x00,0xC0,0x60,0xE0,0x00,0x00,0x00,0x06,0x05,0x04,0x04,0x1F,0x04,0x04,0x00,  /* '4' 0x34 */
    0x00,0xE0,0x20,0x20,0x20,0x20,0x00,0x00,0x00,0x11,0x11,0x11,0x19,0x0E,0x00,0x00,  /* '5' 0x35 */
    0x00,0x80,0x40,0x20,0x20,0x20,0x00,0x00,0x00,0x0F,0x12,0x11,0x11,0x11,0x0E,0x00,  /* '6' 0x36 */
    0x00,0x20,0x20,0x20,0x20,0xA0,0x60,0x00,0x00,0x00,0x10,0x1C,0x07,0x01,0x00,0x00,  /* '7' 0x37 */
    0x00,0xC0,0xA0,0x20,0x20,0xC0,0x00,0x00,0x00,0x0E,0x13,0x11,0x13,0x0E,0x00,0x00,  /* '8' 0x38 */
    0x00,0xC0,0x20,0x20,0x20,0x20,0xC0,0x00,0x00,0x01,0x12,0x12,0x12,0x09,0x07,0x00,  /* '9' 0x39 */
    0x00,0x00,0x00,0x80,0x80,0x00,0x00,0x00,0x00,0x00,0x00,0x19,0x19,0x00,0x00,0x00,  /* ':' 0x3A */
    0x00,0x00,0x00,0x80,0x80,0x00,0x00,0x00,0x00,0x00,0x40,0x79,0x39,0x00,0x00,0x00,  /* ';' 0x3B */
    0x00,0x00,0x00,0x80,0x80,0x40,0x00,0x00,0x00,0x02,0x07,0x0D,0x08,0x10,0x00,0x00,  /* '<' 0x3C */
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x05,0x05,0x05,0x05,0x05,0x00,0x00,  /* '=' 0x3D */
    0x00,0x40,0x80,0x80,0x00,0x00,0x00,0x00,0x00,0x10,0x08,0x0D,0x07,0x02,0x00,0x00,  /* '>' 0x3E */
    0x00,0x00,0x10,0x10,0x20,0xE0,0x00,0x00,0x00,0x00,0x1B,0x19,0x01,0x00,0x00,0x00,  /* '?' 0x3F */
    0x00,0xE0,0x30,0x90,0x90,0x30,0xC0,0x00,0x3F,0x40,0x9F,0x90,0x9F,0x10,0x0F,0x00,  /* '@' 0x40 */
    0x00,0x00,0xC0,0x20,0xE0,0x00,0x00,0x00,0x00,0x1E,0x07,0x04,0x04,0x07,0x1C,0x00,  /* 'A' 0x41 */
    0x00,0xE0,0x20,0x20,0x20,0xC0,0x00,0x00,0x00,0x1F,0x11,0x11,0x11,0x0E,0x00,0x00,  /* 'B' 0x42 */
    0x00,0x80,0x40,0x20,0x20,0x20,0x00,0x00,0x00,0x0F,0x18,0x10,0x10,0x10,0x00,0x00,  /* 'C' 0x43 */
    0x00,0xE0,0x20,0x20,0x20,0x40,0x80,0x00,0x00,0x1F,0x10,0x10,0x10,0x08,0x07,0x00,  /* 'D' 0x44 */
    0x00,0xE0,0x20,0x20,0x20,0x20,0x00,0x00,0x00,0x1F,0x11,0x11,0x11,0x11,0x00,0x00,  /* 'E' 0x45 */
    0x00,0xE0,0x20,0x20,0x20,0x20,0x00,0x00,0x00,0x1F,0x01,0x01,0x01,0x01,0x00,0x00,  /* 'F' 0x46 */
    0x00,0x80,0x40,0x20,0x20,0x20,0x20,0x00,0x00,0x07,0x08,0x10,0x11,0x11,0x1F,0x00,  /* 'G' 0x47 */
    0x00,0xE0,0x00,0x00,0x00,0xE0,0x00,0x00,0x00,0x1F,0x01,0x01,0x01,0x1F,0x00,0x00,  /* 'H' 0x48 */
    0x00,0x20,0x20,0xE0,0x20,0x20,0x00,0x00,0x00,0x10,0x10,0x1F,0x10,0x10,0x00,0x00,  /* 'I' 0x49 */
    0x00,0x00,0x20,0x20,0x20,0xE0,0x00,0x00,0x00,0x00,0x10,0x10,0x10,0x0F,0x00,0x00,  /* 'J' 0x4A */
    0x00,0xE0,0x00,0x80,0x40,0x20,0x00,0x00,0x00,0x1F,0x02,0x05,0x08,0x10,0x00,0x00,  /* 'K' 0x4B */
    0x00,0xE0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x1F,0x10,0x10,0x10,0x10,0x00,0x00,  /* 'L' 0x4C */
    0x00,0xE0,0xC0,0x00,0x80,0xE0,0x00,0x00,0x18,0x1F,0x01,0x02,0x01,0x1F,0x1F,0x00,  /* 'M' 0x4D */
    0x00,0xE0,0xE0,0x00,0x00,0xE0,0x00,0x00,0x00,0x1F,0x00,0x03,0x1C,0x1F,0x00,0x00,  /* 'N' 0x4E */
    0x80,0x40,0x20,0x20,0x20,0x60,0x80,0x00,0x07,0x18,0x10,0x10,0x10,0x08,0x07,0x00,  /* 'O' 0x4F */
    0x00,0xE0,0x20,0x20,0x20,0xC0,0x00,0x00,0x00,0x1F,0x02,0x02,0x02,0x01,0x00,0x00,  /* 'P' 0x50 */
    0x00,0x80,0x60,0x20,0x20,0x60,0xC0,0x00,0x00,0x0F,0x18,0x30,0x50,0x58,0x47,0x00,  /* 'Q' 0x51 */
    0x00,0xE0,0x20,0x20,0x20,0xC0,0x00,0x00,0x00,0x1F,0x02,0x02,0x06,0x1D,0x10,0x00,  /* 'R' 0x52 */
    0x00,0xC0,0x20,0x20,0x20,0x00,0x00,0x00,0x00,0x10,0x11,0x11,0x12,0x0C,0x00,0x00,  /* 'S' 0x53 */
    0x00,0x20,0x20,0xE0,0x20,0x20,0x00,0x00,0x00,0x00,0x00,0x1F,0x00,0x00,0x00,0x00,  /* 'T' 0x54 */
    0x00,0xE0,0x00,0x00,0x00,0x00,0xE0,0x00,0x00,0x0F,0x10,0x10,0x10,0x10,0x0F,0x00,  /* 'U' 0x55 */
    0xE0,0x80,0x00,0x00,0x00,0x80,0x60,0x00,0x00,0x03,0x1E,0x10,0x0E,0x03,0x00,0x00,  /* 'V' 0x56 */
    0x00,0xE0,0x00,0x00,0x00,0xE0,0x00,0x00,0x00,0x1F,0x18,0x07,0x18,0x1F,0x00,0x00,  /* 'W' 0x57 */
    0x20,0x60,0x80,0x00,0x80,0x60,0x20,0x00,0x10,0x18,0x07,0x03,0x07,0x18,0x10,0x00,  /* 'X' 0x58 */
    0x20,0xC0,0x80,0x00,0x80,0xC0,0x20,0x00,0x00,0x00,0x01,0x1E,0x01,0x00,0x00,0x00,  /* 'Y' 0x59 */
    0x00,0x20,0x20,0x20,0x20,0xE0,0x20,0x00,0x00,0x10,0x1C,0x12,0x11,0x10,0x00,0x00,  /* 'Z' 0x5A */
    0x00,0x00,0xF0,0x10,0x10,0x00,0x00,0x00,0x00,0x00,0xFF,0x80,0x80,0x00,0x00,0x00,  /* '[' 0x5B */
    0x00,0x10,0xE0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x03,0x1C,0x20,0x00,0x00,  /* '\' 0x5C */
    0x00,0x00,0x10,0x10,0xF0,0x00,0x00,0x00,0x00,0x00,0x80,0x80,0xFF,0x00,0x00,0x00,  /* ']' 0x5D */
    0x00,0x80,0x40,0x20,0x40,0x80,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x01,0x00,0x00,  /* '^' 0x5E */
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x00,  /* '_' 0x5F */
    0x00,0x00,0x10,0x30,0x20,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,  /* '`' 0x60 */
    0x00,0x00,0x80,0x80,0x80,0x00,0x00,0x00,0x00,0x0C,0x12,0x12,0x1A,0x1F,0x00,0x00,  /* 'a' 0x61 */
    0x00,0xF0,0x80,0x80,0x80,0x00,0x00,0x00,0x00,0x1F,0x11,0x10,0x10,0x0F,0x00,0x00,  /* 'b' 0x62 */
    0x00,0x00,0x80,0x80,0x80,0x80,0x00,0x00,0x00,0x0F,0x19,0x10,0x10,0x10,0x00,0x00,  /* 'c' 0x63 */
    0x00,0x00,0x80,0x80,0x80,0xF0,0x00,0x00,0x00,0x0F,0x10,0x10,0x08,0x1F,0x00,0x00,  /* 'd' 0x64 */
    0x00,0x00,0x80,0x80,0x80,0x80,0x00,0x00,0x00,0x0F,0x1A,0x12,0x12,0x12,0x13,0x00,  /* 'e' 0x65 */
    0x00,0x80,0x80,0xE0,0x90,0x90,0x10,0x00,0x00,0x00,0x00,0x1F,0x00,0x00,0x00,0x00,  /* 'f' 0x66 */
    0x00,0x00,0x80,0x80,0x80,0x80,0x80,0x00,0x00,0x7F,0x94,0x94,0x94,0x93,0x60,0x00,  /* 'g' 0x67 */
    0x00,0xF0,0x00,0x80,0x80,0x00,0x00,0x00,0x00,0x1F,0x01,0x00,0x00,0x1F,0x00,0x00,  /* 'h' 0x68 */
    0x00,0x80,0x80,0xB0,0x30,0x00,0x00,0x00,0x00,0x10,0x10,0x1F,0x10,0x10,0x00,0x00,  /* 'i' 0x69 */
    0x00,0x00,0x80,0x80,0xB0,0xB0,0x00,0x00,0x00,0x00,0x80,0x80,0x80,0x7F,0x00,0x00,  /* 'j' 0x6A */
    0x00,0xF0,0x00,0x00,0x80,0x80,0x00,0x00,0x00,0x1F,0x02,0x05,0x09,0x10,0x00,0x00,  /* 'k' 0x6B */
    0x00,0x10,0x10,0xF0,0x00,0x00,0x00,0x00,0x00,0x10,0x10,0x1F,0x10,0x10,0x00,0x00,  /* 'l' 0x6C */
    0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x00,0x1F,0x01,0x00,0x1F,0x01,0x00,0x1F,0x00,  /* 'm' 0x6D */
    0x00,0x80,0x80,0x80,0x80,0x00,0x00,0x00,0x00,0x1F,0x01,0x00,0x00,0x1F,0x00,0x00,  /* 'n' 0x6E */
    0x00,0x00,0x80,0x80,0x80,0x80,0x00,0x00,0x00,0x0F,0x19,0x10,0x10,0x19,0x0F,0x00,  /* 'o' 0x6F */
    0x00,0x80,0x80,0x80,0x80,0x00,0x00,0x00,0x00,0xFF,0x11,0x10,0x10,0x0F,0x00,0x00,  /* 'p' 0x70 */
    0x00,0x00,0x80,0x80,0x80,0x80,0x00,0x00,0x00,0x0F,0x10,0x10,0x08,0xFF,0x00,0x00,  /* 'q' 0x71 */
    0x00,0x80,0x00,0x80,0x80,0x80,0x00,0x00,0x00,0x1F,0x01,0x00,0x00,0x01,0x00,0x00,  /* 'r' 0x72 */
    0x00,0x00,0x80,0x80,0x80,0x80,0x00,0x00,0x00,0x13,0x12,0x12,0x14,0x0C,0x00,0x00,  /* 's' 0x73 */
    0x00,0x80,0x80,0xE0,0x80,0x80,0x80,0x00,0x00,0x00,0x00,0x1F,0x10,0x10,0x10,0x00,  /* 't' 0x74 */
    0x00,0x80,0x00,0x00,0x00,0x80,0x00,0x00,0x00,0x0F,0x10,0x10,0x18,0x1F,0x00,0x00,  /* 'u' 0x75 */
    0x00,0x80,0x00,0x00,0x00,0x80,0x00,0x00,0x00,0x03,0x0E,0x10,0x0E,0x03,0x00,0x00,  /* 'v' 0x76 */
    0x00,0x80,0x00,0x00,0x00,0x80,0x00,0x00,0x00,0x0F,0x18,0x07,0x0C,0x1F,0x00,0x00,  /* 'w' 0x77 */
    0x00,0x80,0x80,0x00,0x00,0x80,0x80,0x00,0x10,0x18,0x0D,0x02,0x07,0x19,0x10,0x00,  /* 'x' 0x78 */
    0x00,0x00,0x80,0x00,0x00,0x00,0x80,0x00,0x00,0x80,0x83,0xCE,0x30,0x0E,0x01,0x00,  /* 'y' 0x79 */
    0x00,0x80,0x80,0x80,0x80,0x80,0x00,0x00,0x00,0x10,0x18,0x16,0x11,0x10,0x00,0x00,  /* 'z' 0x7A */
    0x00,0x00,0x00,0xE0,0x10,0x10,0x00,0x00,0x00,0x02,0x02,0x7D,0x80,0x80,0x00,0x00,  /* '{' 0x7B */
    0x00,0x00,0x00,0xF8,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0x00,0x00,0x00,0x00,  /* '|' 0x7C */
    0x00,0x10,0x10,0xE0,0x00,0x00,0x00,0x00,0x00,0x80,0x80,0x7D,0x02,0x02,0x00,0x00,  /* '}' 0x7D */
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x06,0x01,0x03,0x06,0x04,0x03,0x00,  /* '~' 0x7E */
};

static void OLED_ShowChar(uint8_t x, uint8_t page, char c)
{
    uint8_t i;
    const unsigned char *glyph = F8X16[c - ' '];

    for (i = 0; i < 8; i++) {
        oled_ram[page][x + i]     = glyph[i];
        oled_ram[page + 1][x + i] = glyph[i + 8];
    }
}

static void OLED_ShowString(uint8_t x, uint8_t page, const char *s)
{
    while (*s) {
        OLED_ShowChar(x, page, *s++);
        x += 8;
    }
}

/* 清除一整行 8x16 文本区域。
 * 一行 16px 高，因此对应 SSD1306 的两个 page。
 *
 * 为什么需要它:
 * 增量刷新模式下，短字符串覆盖长字符串时，旧字符串尾部不会自动消失。
 * 例如:
 *   "ALARM: HIGH TEMP"  -> "STATE: NORMAL"
 * 后者更短，若不先清行，末尾 "EMP" 会残留。
 */
static void OLED_ClearTextLine(uint8_t page)
{
    if (page > 6)
        return;

    memset(oled_ram[page],     0, 128);
    memset(oled_ram[page + 1], 0, 128);
}


/*==================== M3-P4 RTOS 数据模型 ====================*/
typedef struct {
    int32_t current_ua;
    int32_t power_uw;
    int16_t shunt_raw;
    int16_t current_raw;
    uint16_t power_raw;
    uint16_t cal_read;
    uint16_t mask_start;
    uint16_t mask_end;
    uint16_t flags;
} ElectricalSnapshot;

typedef struct {
    uint32_t sequence;
    TickType_t tick;
    uint16_t ina_raw;
    uint16_t bus_mv;
    uint16_t min_mv;
    uint16_t max_mv;
    int16_t  temp_raw;
    uint8_t  temp_status;
    uint32_t i2c_stop_timeout;
    ElectricalSnapshot electrical;
} SensorSnapshot;

typedef struct {
    uint8_t page;
    uint16_t keys;
} UiState;

typedef struct {
    uint8_t active;
    uint8_t relay_on;
    uint8_t relay_reason;
} AlarmState;

typedef struct {
		SensorSnapshot sensor;
		uint8_t alarm_active;
    uint8_t relay_on;
    uint8_t relay_reason;
} MonitorSnapshot;

typedef struct {
		uint16_t heap_now;
		uint16_t heap_min;
		uint16_t stack[7];
} RtosDiagnostics;

typedef enum {
    BUZZER_CMD_KEY_BEEP = 1,
    BUZZER_CMD_ALARM_ON,
    BUZZER_CMD_ALARM_OFF
} BuzzerCommand;

static QueueHandle_t g_q_sensor_display = NULL;
static QueueHandle_t g_q_sensor_alarm   = NULL;
static QueueHandle_t g_q_sensor_uart    = NULL;
static QueueHandle_t g_q_ui_state       = NULL;
static QueueHandle_t g_q_alarm_state    = NULL;
static QueueHandle_t g_q_buzzer_cmd     = NULL;
static QueueHandle_t g_q_monitor_modbus = NULL;
static QueueHandle_t g_rx_queue         = NULL;

static SemaphoreHandle_t g_i2c_mutex    = NULL;

static TaskHandle_t g_task_sample  = NULL;
static TaskHandle_t g_task_display = NULL;
static TaskHandle_t g_task_input   = NULL;
static TaskHandle_t g_task_alarm   = NULL;
static TaskHandle_t g_task_buzzer  = NULL;
static TaskHandle_t g_task_uart    = NULL;
static TaskHandle_t g_task_modbus  = NULL;

static volatile uint32_t g_rx_error_count = 0;
/* Temporary OLED diagnostics: observation only, no protocol changes. */
static volatile uint32_t g_diag_rx_bytes = 0;
static volatile uint32_t g_diag_frames = 0;
static volatile uint32_t g_diag_tx_start = 0;
static volatile uint32_t g_diag_tx_done = 0;
static volatile uint16_t g_diag_last_len = 0;
static AlarmState g_relay_view; /* copied under a short critical section */
static RtosDiagnostics g_diagnostics;
static uint8_t g_ina226_id_ok = 0;
static uint8_t g_ina226_cal_ok = 0;

/* ELECTRICAL_CONVERSION_BEGIN */
static void Electrical_Convert(ElectricalSnapshot *e, uint16_t bus_raw,
                               uint8_t reads_ok, uint8_t boot_ok)
{
    e->flags = 0;
    e->current_ua = 0;
    e->power_uw = 0;
    if (reads_ok) {
        e->flags |= 1U;
        if (e->cal_read == INA226_CAL_VALUE) e->flags |= 2U;
        if (((e->mask_start | e->mask_end) & 4U) == 0U) e->flags |= 8U;
        if (bus_raw <= 28800U && e->shunt_raw != (int16_t)0x8000 &&
            e->shunt_raw != 32767 && e->current_raw != (int16_t)0x8000 &&
            e->current_raw != 32767) e->flags |= 16U;
        if ((e->mask_start & 8U) != 0U) e->flags |= 32U;
        if ((e->mask_end & 8U) == 0U) e->flags |= 64U;
    }
    if (boot_ok) e->flags |= 4U;
    if ((e->flags & ELECTRICAL_VALID_MASK) == ELECTRICAL_VALID_MASK) {
        e->current_ua = (int32_t)e->current_raw * INA226_CURRENT_LSB_UA;
        /* bus_raw * 1.25mV * current_uA -> signed microwatts. */
        e->power_uw = (int32_t)(((int64_t)bus_raw * e->current_ua) / 800);
    }
}
/* ELECTRICAL_CONVERSION_END */

/* Caller owns the shared I2C mutex. No automatic recalibration/retry loop. */
static void Electrical_Sample(ElectricalSnapshot *e)
{
    uint16_t shunt = 0, current = 0, bus = 0;
    uint8_t ok = 1;
    memset(e, 0, sizeof(*e));
    if (ina226_read_checked(0x06, &e->mask_start) != 0U) ok = 0;
    if (ina226_read_checked(0x01, &shunt) != 0U) ok = 0;
    if (ina226_read_checked(0x02, &bus) != 0U) ok = 0;
    if (ina226_read_checked(0x04, &current) != 0U) ok = 0;
    if (ina226_read_checked(0x03, &e->power_raw) != 0U) ok = 0;
    if (ina226_read_checked(0x05, &e->cal_read) != 0U) ok = 0;
    if (ina226_read_checked(0x06, &e->mask_end) != 0U) ok = 0;
    e->shunt_raw = (int16_t)shunt;
    e->current_raw = (int16_t)current;
    Electrical_Convert(e, bus, ok, (uint8_t)(g_ina226_cal_ok && g_ina226_id_ok));
}

/* RTU_RX_CORE_BEGIN: host-tested framing state machine, not tied to SysTick. */
#define RTU_T15_US 750U
#define RTU_T35_US 1750U
#define RTU_CHAR_US ((11000000UL + MODBUS_BAUD_RATE - 1UL) / MODBUS_BAUD_RATE)
/* RXNE timestamps are character-completion timestamps, not start timestamps.
 * One character of guard avoids closing a frame with a byte still in flight.
 * IRQ latency still limits accuracy near boundaries; scope validation pending. */
#define RTU_INTERBYTE_LIMIT (RTU_T15_US + RTU_CHAR_US)
#define RTU_END_US (RTU_T35_US + RTU_CHAR_US)
#define RTU_SLOT_COUNT 3U
#define RTU_ADU_MAX 256U
#define RTU_NONE 255U
#define RTU_FREE 0U
#define RTU_FILLING 1U
#define RTU_READY 2U
typedef struct {
    uint8_t data[RTU_ADU_MAX];
    uint16_t len;
    uint8_t invalid;
    volatile uint8_t state;
} RtuFrame;
static RtuFrame g_rtu_frames[RTU_SLOT_COUNT];
static volatile uint8_t g_rtu_slot = RTU_NONE;
static volatile uint8_t g_rtu_guard = 1;
static volatile uint8_t g_rtu_armed = 0;
static volatile uint8_t g_rtu_dropping = 0;
static volatile uint8_t g_rtu_tx = 0;
static volatile uint16_t g_rtu_last = 0;
static void Rtu_ArmTimer(uint16_t now);

static void Rtu_Finish(BaseType_t *wake)
{
    uint8_t slot = g_rtu_slot;
    g_rtu_slot = RTU_NONE;
    g_rtu_armed = 0;
    g_rtu_guard = 0;
    g_rtu_dropping = 0;
    if (slot != RTU_NONE) {
        if (g_rtu_frames[slot].invalid) {
            g_rtu_frames[slot].state = RTU_FREE;
        } else {
            g_rtu_frames[slot].state = RTU_READY;
            if (xQueueSendFromISR(g_rx_queue, &slot, wake) != pdPASS) {
                g_rtu_frames[slot].state = RTU_FREE;
                g_rx_error_count++;
            }
        }
    }
}

static void Rtu_ReceiveByte(uint16_t now, uint8_t byte, uint8_t bad,
                            BaseType_t *wake)
{
    uint16_t gap = (uint16_t)(now - g_rtu_last);
    uint8_t i;
    RtuFrame *frame;
    if (g_rtu_tx) return;
    if (g_rtu_armed && gap >= RTU_END_US) Rtu_Finish(wake);
    if (g_rtu_guard) {
        if (!g_rtu_dropping) g_rx_error_count++;
        g_rtu_dropping = 1;
        Rtu_ArmTimer(now);
        return;
    }
    if (g_rtu_slot != RTU_NONE) {
        frame = &g_rtu_frames[g_rtu_slot];
        if (gap > RTU_INTERBYTE_LIMIT && !frame->invalid) {
            frame->invalid = 1;
            g_rx_error_count++;
        }
    } else if (!g_rtu_dropping) {
        for (i = 0; i < RTU_SLOT_COUNT; i++) {
            if (g_rtu_frames[i].state == RTU_FREE) {
                g_rtu_slot = i;
                g_rtu_frames[i].len = 0;
                g_rtu_frames[i].invalid = 0;
                g_rtu_frames[i].state = RTU_FILLING;
                break;
            }
        }
        if (g_rtu_slot == RTU_NONE) {
            g_rtu_dropping = 1;
            g_rx_error_count++;
        }
    }
    if (g_rtu_slot != RTU_NONE) {
        frame = &g_rtu_frames[g_rtu_slot];
        if (bad) {
            frame->invalid = 1;
            g_rx_error_count++;
        }
        if (frame->len < RTU_ADU_MAX) frame->data[frame->len++] = byte;
        else if (!frame->invalid) {
            frame->invalid = 1;
            g_rx_error_count++;
        }
    }
    Rtu_ArmTimer(now);
}
/* RTU_RX_CORE_END */

static void Rtu_ArmTimer(uint16_t now)
{
    TIM3->DIER &= ~TIM_DIER_CC1IE;
    g_rtu_last = now;
    g_rtu_armed = 1;
    TIM3->CCR1 = (uint16_t)(now + RTU_END_US);
    TIM3->SR = (uint16_t)~TIM_SR_CC1IF;
    TIM3->DIER |= TIM_DIER_CC1IE;
}

static void Rtu_ServiceUart(BaseType_t *wake)
{
    uint32_t sr = USART1->SR;
    uint16_t now;
    uint8_t byte;
    uint8_t bad;
    if ((sr & (USART_SR_RXNE | USART_SR_ORE | USART_SR_NE | USART_SR_FE | USART_SR_PE)) != 0U) {
        now = (uint16_t)TIM3->CNT;
        byte = (uint8_t)USART1->DR;
        if ((sr & USART_SR_RXNE) != 0U) g_diag_rx_bytes++;
        bad = ((sr & (USART_SR_ORE | USART_SR_NE | USART_SR_FE | USART_SR_PE)) != 0U);
        Rtu_ReceiveByte(now, byte, bad, wake);
    }
}

static const uint8_t *g_rtu_tx_data;
static uint16_t g_rtu_tx_len;
static uint16_t g_rtu_tx_pos;

/* Interrupt-driven TX prevents a task switch from inserting a long byte gap. */
static void Rtu_ServiceTx(void)
{
    if (g_rtu_tx && (USART1->CR1 & USART_CR1_TXEIE) != 0U &&
        (USART1->SR & USART_SR_TXE) != 0U) {
        if (g_rtu_tx_pos < g_rtu_tx_len) {
            USART_SendData(USART1, g_rtu_tx_data[g_rtu_tx_pos++]);
        } else {
            USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
            USART_ITConfig(USART1, USART_IT_TC, ENABLE);
        }
    }
    if (g_rtu_tx && (USART1->CR1 & USART_CR1_TCIE) != 0U &&
        (USART1->SR & USART_SR_TC) != 0U) {
        USART_ITConfig(USART1, USART_IT_TC, DISABLE);
        (void)USART1->SR;
        (void)USART1->DR;
        g_rtu_guard = 1;
        /* Guard after known TX completion is t3.5, not t3.5 + RX character. */
        Rtu_ArmTimer((uint16_t)(TIM3->CNT - RTU_CHAR_US));
        RS485_RECEIVE();
        USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
        g_diag_tx_done++;
        g_rtu_tx = 0;
    }
}

void USART1_IRQHandler(void)
{
    BaseType_t wake = pdFALSE;
    Rtu_ServiceUart(&wake);
    Rtu_ServiceTx();
    portYIELD_FROM_ISR(wake);
}

void TIM3_IRQHandler(void)
{
    BaseType_t wake = pdFALSE;
    uint16_t now;
    if ((TIM3->SR & TIM_SR_CC1IF) != 0U) {
        TIM3->DIER &= ~TIM_DIER_CC1IE;
        TIM3->SR = (uint16_t)~TIM_SR_CC1IF;
        /* Same priority as USART1: no nesting; pending RXNE is handled first. */
        Rtu_ServiceUart(&wake);
        now = (uint16_t)TIM3->CNT;
        if (g_rtu_armed && (uint16_t)(now - g_rtu_last) >= RTU_END_US)
            Rtu_Finish(&wake);
        else if (g_rtu_armed) TIM3->DIER |= TIM_DIER_CC1IE;
    }
    portYIELD_FROM_ISR(wake);
}

/* Called from the Modbus task, after queues and scheduler exist. */
static void Rtu_TimerStart(void)
{
    NVIC_InitTypeDef nvic;
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    TIM3->CR1 = 0;
    TIM3->DIER = 0;
    TIM3->PSC = 71;
    TIM3->ARR = 0xFFFF;
    TIM3->CCMR1 = 0;
    TIM3->CCER = 0;
    TIM3->EGR = TIM_EGR_UG;
    TIM3->SR = 0;
    TIM3->CNT = 0;
    nvic.NVIC_IRQChannel = TIM3_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 6;
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
    Rtu_ArmTimer(0);
    TIM3->CR1 = TIM_CR1_CEN;
}
/* CRC-16/MODBUS: init FFFF, reflected polynomial A001, no final XOR.
 * Wire order: low byte, then high byte. */
static uint16_t Modbus_CRC16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFFU;
    uint16_t i;
    uint8_t bit;
    for (i = 0; i < len; i++) {
        crc ^= data[i];
        for (bit = 0; bit < 8U; bit++) {
            if ((crc & 1U) != 0U)
                crc = (uint16_t)((crc >> 1) ^ 0xA001U);
            else
                crc = (uint16_t)(crc >> 1);
        }
    }
    return crc;
}

/* Read-only test slave. Addresses are zero-based. */
#define MODBUS_SLAVE_ADDRESS 1U

typedef struct {
    uint8_t address;
    uint8_t function;
    uint16_t start;
    uint16_t quantity;
} ModbusReadRequest;

typedef enum {
    MB_PARSE_OK = 0, MB_PARSE_OVERFLOW, MB_PARSE_SHORT, MB_PARSE_CRC,
    MB_PARSE_OTHER_ADDRESS, MB_PARSE_BROADCAST_READ, MB_PARSE_FUNCTION,
    MB_PARSE_LENGTH, MB_PARSE_QUANTITY, MB_PARSE_RANGE
} ModbusParseResult;

static ModbusParseResult Modbus_ParseReadRequest(const uint8_t *frame,
                                                 uint16_t len,
                                                 uint8_t overflow,
                                                 ModbusReadRequest *request)
{
    uint16_t received;
    uint16_t start;
    uint16_t quantity;
    if (overflow) return MB_PARSE_OVERFLOW;
    if (len < 4U) return MB_PARSE_SHORT;
    received = (uint16_t)((uint16_t)frame[len - 2U] |
                         ((uint16_t)frame[len - 1U] << 8));
    if (Modbus_CRC16(frame, (uint16_t)(len - 2U)) != received)
        return MB_PARSE_CRC;
    if (frame[0] == 0U) return MB_PARSE_BROADCAST_READ;
    if (frame[0] != MODBUS_SLAVE_ADDRESS) return MB_PARSE_OTHER_ADDRESS;
    if (frame[1] != 0x03U) return MB_PARSE_FUNCTION;
    if (len != 8U) return MB_PARSE_LENGTH;
    start = (uint16_t)(((uint16_t)frame[2] << 8) | frame[3]);
    quantity = (uint16_t)(((uint16_t)frame[4] << 8) | frame[5]);
    if ((quantity == 0U) || (quantity > 125U)) return MB_PARSE_QUANTITY;
    if ((uint32_t)start + (uint32_t)quantity > 65536UL)
        return MB_PARSE_RANGE;
    request->address = frame[0];
    request->function = frame[1];
    request->start = start;
    request->quantity = quantity;
    return MB_PARSE_OK;
}

#define MODBUS_REGISTER_COUNT 36U
#define MODBUS_RESPONSE_CAPACITY 77U
static uint16_t Modbus_BuildResponse(const uint8_t *frame, uint16_t len,
                                     uint8_t invalid, const uint16_t *registers, uint8_t *response,
                                     uint16_t capacity)
{
    ModbusReadRequest request;
    ModbusParseResult result;
    uint8_t exception = 0;
    uint16_t count;
    uint16_t i;
    uint16_t value;
    uint16_t crc;
    result = Modbus_ParseReadRequest(frame, len, invalid, &request);
    switch (result) {
    case MB_PARSE_OK:
        if ((uint32_t)request.start + request.quantity > MODBUS_REGISTER_COUNT)
            exception = 0x02U;
        break;
    case MB_PARSE_FUNCTION: exception = 0x01U; break;
    case MB_PARSE_QUANTITY: exception = 0x03U; break;
    case MB_PARSE_RANGE: exception = 0x02U; break;
    default: return 0; /* CRC/length/errors, other addresses and broadcast: silence. */
    }
    if (exception != 0U) {
        if (capacity < 5U) return 0;
        response[0] = MODBUS_SLAVE_ADDRESS;
        response[1] = (uint8_t)(frame[1] | 0x80U);
        response[2] = exception;
        count = 3U;
    } else {
        count = (uint16_t)(3U + 2U * request.quantity);
        if (capacity < count + 2U) return 0;
        response[0] = MODBUS_SLAVE_ADDRESS;
        response[1] = 0x03U;
        response[2] = (uint8_t)(2U * request.quantity);
        for (i = 0; i < request.quantity; i++) {
            value = registers[request.start + i];
            response[3U + 2U*i] = (uint8_t)(value >> 8);
            response[4U + 2U*i] = (uint8_t)value;
        }
    }
    crc = Modbus_CRC16(response, count);
    response[count++] = (uint8_t)crc;
    response[count++] = (uint8_t)(crc >> 8);
    return count;
}


/*==================== 页面绘制 ====================*/
/* 每页 4 行文字(每行占 2 个显示页: 0-1, 2-3, 4-5, 6-7)
 * 【v2 修复】只在换页时清显存 —— 旧版每轮调用都 memset 清空 8 页,
 * 导致影子帧每轮全部判脏, 增量刷新退化成每 250ms 一次整帧全量发送(~30ms 阻塞)。
 * 清屏保留旧显存后, 平时只有数字变化的页判脏, 单轮刷新开销降到 ~3ms */
static void Draw_Page(uint8_t page, uint16_t mv, uint16_t raw,
                      uint16_t min_mv, uint16_t max_mv,
                      uint32_t uptime_s, uint16_t keys,
                      int16_t temp_raw, uint8_t temp_status,
                      uint8_t temp_alarm)
{
    char buf[20];
    static uint8_t drawn_page = 0xFF;
    uint16_t mag, whole, frac4;
    AlarmState relay;

    if (page != drawn_page) {
        memset(oled_ram, 0, sizeof(oled_ram));
        drawn_page = page;
    }

    switch (page) {
    case 0:
        OLED_ShowString(0, 0, "PAGE1 VOLTAGE");
        sprintf(buf, "BUS: %2d.%03d V", mv / 1000, mv % 1000);
        OLED_ShowString(0, 2, buf);
        sprintf(buf, "RAW: %5d", raw);
        OLED_ShowString(0, 4, buf);
        OLED_ShowString(0, 6, "KEY: NEXT 1/4");
        break;

    case 1:
        OLED_ShowString(0, 0, "PAGE2 STATS");
        sprintf(buf, "MIN: %2d.%03d V", min_mv / 1000, min_mv % 1000);
        OLED_ShowString(0, 2, buf);
        sprintf(buf, "MAX: %2d.%03d V", max_mv / 1000, max_mv % 1000);
        OLED_ShowString(0, 4, buf);
        OLED_ShowString(0, 6, "KEY: NEXT 2/4");
        break;

    case 2:
        OLED_ClearTextLine(0);
        OLED_ClearTextLine(2);
        OLED_ClearTextLine(4);
        OLED_ClearTextLine(6);
        sprintf(buf, "RS485 D%d R%d",
                GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_1),
                GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_10));
        OLED_ShowString(0, 0, buf);
        sprintf(buf, "B%05u E%05u",
                (unsigned)(g_diag_rx_bytes % 100000UL),
                (unsigned)(g_rx_error_count % 100000UL));
        OLED_ShowString(0, 2, buf);
        sprintf(buf, "F%05u L%03u",
                (unsigned)(g_diag_frames % 100000UL),
                (unsigned)g_diag_last_len);
        OLED_ShowString(0, 4, buf);
        sprintf(buf, "T%05u C%05u",
                (unsigned)(g_diag_tx_start % 100000UL),
                (unsigned)(g_diag_tx_done % 100000UL));
        OLED_ShowString(0, 6, buf);
        break;

    case 3:
        OLED_ShowString(0, 0, "PAGE4 TEMP");

        /* P7 OLED 修复:
         * PAGE4 的温度行和状态行长度会随状态变化。
         * 每次重画前先清这两行，避免长字符串切换成短字符串后残字。 */
        OLED_ClearTextLine(2);
        OLED_ClearTextLine(4);

        if (temp_status == 1) {
            if (temp_raw < 0) {
                mag = (uint16_t)(-temp_raw);
                whole = mag / 16;
                frac4 = (mag % 16) * 625;
                sprintf(buf, "TEMP:-%d.%04d C", whole, frac4);
            } else {
                mag = (uint16_t)temp_raw;
                whole = mag / 16;
                frac4 = (mag % 16) * 625;
                sprintf(buf, "TEMP: %d.%04d C", whole, frac4);
            }
            OLED_ShowString(0, 2, buf);

            if (temp_alarm)
                OLED_ShowString(0, 4, "ALARM: HIGH TEMP");
            else
                OLED_ShowString(0, 4, "STATE: NORMAL");
        }
        else if (temp_status == 2) {
            OLED_ShowString(0, 2, "TEMP: NOT FOUND");
            OLED_ShowString(0, 4, "STATE: SENSORERR");
        }
        else if (temp_status == 3) {
            OLED_ShowString(0, 2, "TEMP: CRC ERROR");
            OLED_ShowString(0, 4, "STATE: CRC FAIL");
        }
        else {
            OLED_ShowString(0, 2, "TEMP: WAIT...");
            OLED_ShowString(0, 4, "STATE: WAIT");
        }

        taskENTER_CRITICAL(); relay = g_relay_view; taskEXIT_CRITICAL();
        OLED_ClearTextLine(6);
        if (relay.relay_on) OLED_ShowString(0, 6, "RELAY: ON CMD");
        else if (relay.relay_reason == RELAY_HOT) OLED_ShowString(0, 6, "OFF: HIGH TEMP");
        else if (relay.relay_reason == RELAY_SENSOR) OLED_ShowString(0, 6, "OFF: SENSOR ERR");
        else if (relay.relay_reason == RELAY_STALE) OLED_ShowString(0, 6, "OFF: DATA STALE");
        else OLED_ShowString(0, 6, "OFF: WAIT TEMP");
        break;
    }

    OLED_Refresh();
}

/*==================== FreeRTOS engineering hooks ====================*/
/*
 * Stack overflow:
 *   configCHECK_FOR_STACK_OVERFLOW = 2
 *
 * malloc failure:
 *   configUSE_MALLOC_FAILED_HOOK = 1
 *
 * 进入任一 hook 都说明 M3 工程验收失败。
 * 这里不继续运行系统：PC13 常亮 + 蜂鸣器常响，便于一眼识别。
 */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)xTask;
    (void)pcTaskName;

    RELAY_OFF();
    taskDISABLE_INTERRUPTS();
    GPIO_ResetBits(GPIOC, GPIO_Pin_13);
    BEEP_ON();

    for (;;) {
    }
}

void vApplicationMallocFailedHook(void)
{
    RELAY_OFF();
    taskDISABLE_INTERRUPTS();
    GPIO_ResetBits(GPIOC, GPIO_Pin_13);
    BEEP_ON();

    for (;;) {
    }
}

/*==================== task_sample ====================*/
static void Task_Sample(void *pvParameters)
{
    TickType_t last_wake;
    TickType_t ds_start_tick = 0;
    SensorSnapshot snap;
    uint8_t ds_scratch[9];
    uint8_t ds_result;
    uint8_t ds_crc;
    uint8_t ds_converting = 0;

    (void)pvParameters;

    memset(&snap, 0, sizeof(snap));
    snap.min_mv = 0xFFFF;

    ds_result = DS18B20_StartConversion();
    if (ds_result == 0) {
        ds_converting = 1;
        ds_start_tick = xTaskGetTickCount();
        snap.temp_status = 0;
    } else {
        snap.temp_status = 2;
        ds_start_tick = xTaskGetTickCount();
    }

    last_wake = xTaskGetTickCount();

    for (;;) {
        snap.sequence++;

        xSemaphoreTake(g_i2c_mutex, portMAX_DELAY);
        snap.ina_raw = ina226_read_reg(0x02);
        Electrical_Sample(&snap.electrical);
        xSemaphoreGive(g_i2c_mutex);

        if (snap.ina_raw < 0xFFFEU) {
            snap.bus_mv = snap.ina_raw * 125U / 100U;
            if (snap.bus_mv < snap.min_mv) snap.min_mv = snap.bus_mv;
            if (snap.bus_mv > snap.max_mv) snap.max_mv = snap.bus_mv;
        }

        if (ds_converting) {
            if ((TickType_t)(xTaskGetTickCount() - ds_start_tick) >=
                pdMS_TO_TICKS(800)) {

                ds_result = DS18B20_ReadScratchpad(ds_scratch);

                if (ds_result == 0) {
                    ds_crc = DS18B20_CRC8(ds_scratch, 8);

                    if (ds_crc == ds_scratch[8]) {
                        snap.temp_raw =
                            (int16_t)(((uint16_t)ds_scratch[1] << 8) |
                                      ds_scratch[0]);
                        snap.temp_status = 1;
                    } else {
                        snap.temp_status = 3;
                    }
                } else {
                    snap.temp_status = 2;
                }

                ds_result = DS18B20_StartConversion();
                if (ds_result == 0) {
                    ds_converting = 1;
                    ds_start_tick = xTaskGetTickCount();
                } else {
                    ds_converting = 0;
                    snap.temp_status = 2;
                    ds_start_tick = xTaskGetTickCount();
                }
            }
        } else {
            if ((TickType_t)(xTaskGetTickCount() - ds_start_tick) >=
                pdMS_TO_TICKS(1000)) {

                ds_result = DS18B20_StartConversion();

                if (ds_result == 0) {
                    ds_converting = 1;
                    ds_start_tick = xTaskGetTickCount();
                    snap.temp_status = 0;
                } else {
                    snap.temp_status = 2;
                    ds_start_tick = xTaskGetTickCount();
                }
            }
        }

        snap.tick = xTaskGetTickCount();
        snap.i2c_stop_timeout = g_i2c_stop_timeout;

        /* Queue 不是广播；每个消费者一只 1-slot mailbox。 */
        xQueueOverwrite(g_q_sensor_display, &snap);
        xQueueOverwrite(g_q_sensor_alarm,   &snap);
        xQueueOverwrite(g_q_sensor_uart,    &snap);

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(250));
    }
}

/*==================== task_input ====================*/
static void Task_Input(void *pvParameters)
{
    TickType_t last_wake;
    UiState ui;
    BuzzerCommand cmd;

    (void)pvParameters;

    ui.page = 0;
    ui.keys = 0;
    xQueueOverwrite(g_q_ui_state, &ui);

    last_wake = xTaskGetTickCount();

    for (;;) {
        if (Key_GetEvent()) {
            ui.keys++;
            ui.page = (ui.page + 1U) % 4U;
            xQueueOverwrite(g_q_ui_state, &ui);

            cmd = BUZZER_CMD_KEY_BEEP;
            xQueueSend(g_q_buzzer_cmd, &cmd, 0);
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(10));
    }
}

/* RELAY_POLICY_BEGIN: host-tested decisions, no hardware accesses. */
static void Relay_Evaluate(AlarmState *state, const SensorSnapshot *snap,
                           uint8_t have_sample, TickType_t now)
{
    state->relay_on = 0;
    state->relay_reason = RELAY_WAIT;
    if (!have_sample) return;
    if ((TickType_t)(now - snap->tick) > pdMS_TO_TICKS(RELAY_MAX_AGE_MS)) {
        state->relay_reason = RELAY_STALE;
        return;
    }
    if (snap->temp_status != 1U) {
        state->relay_reason = RELAY_SENSOR;
        return;
    }
    if (snap->temp_raw >= TEMP_ALARM_ON_RAW) state->active = 1;
    else if (snap->temp_raw <= TEMP_ALARM_OFF_RAW) state->active = 0;
    if (state->active) state->relay_reason = RELAY_HOT;
    else {
        state->relay_reason = RELAY_NORMAL;
        state->relay_on = 1;
    }
}
/* RELAY_POLICY_END */

/*==================== task_alarm ====================*/
static void Task_Alarm(void *pvParameters)
{
    SensorSnapshot snap;
    AlarmState alarm;
    MonitorSnapshot monitor;
    BuzzerCommand cmd;
    uint8_t previous_alarm;
    uint8_t have_sample = 0;

    (void)pvParameters;
    memset(&alarm, 0, sizeof(alarm));
    memset(&snap, 0, sizeof(snap));
    memset(&monitor, 0, sizeof(monitor));
    RELAY_OFF();
    xQueueOverwrite(g_q_alarm_state, &alarm);
    for (;;) {
        if (xQueueReceive(g_q_sensor_alarm, &snap, pdMS_TO_TICKS(100)) == pdPASS)
            have_sample = 1;
        previous_alarm = alarm.active;
        Relay_Evaluate(&alarm, &snap, have_sample, xTaskGetTickCount());
        if (alarm.relay_on) RELAY_ON(); else RELAY_OFF();
        taskENTER_CRITICAL(); g_relay_view = alarm; taskEXIT_CRITICAL();
        xQueueOverwrite(g_q_alarm_state, &alarm);
        if (previous_alarm != alarm.active) {
            cmd = alarm.active ? BUZZER_CMD_ALARM_ON : BUZZER_CMD_ALARM_OFF;
            xQueueSend(g_q_buzzer_cmd, &cmd, 0);
        }
        if (have_sample) {
            /* Includes OFF command after sample timeout, even without new data. */
            monitor.sensor = snap;
            monitor.alarm_active = alarm.active;
            monitor.relay_on = alarm.relay_on;
            monitor.relay_reason = alarm.relay_reason;
            xQueueOverwrite(g_q_monitor_modbus, &monitor);
        }
    }
}

/*==================== task_buzzer ====================*/
static void Task_Buzzer(void *pvParameters)
{
    TickType_t last_wake;
    BuzzerCommand cmd;
    uint8_t key_beep_ticks = 0;
    uint8_t alarm_active = 0;
    uint32_t phase = 0;

    (void)pvParameters;
    last_wake = xTaskGetTickCount();

    for (;;) {
        while (xQueueReceive(g_q_buzzer_cmd, &cmd, 0) == pdPASS) {
            if (cmd == BUZZER_CMD_KEY_BEEP) {
                key_beep_ticks = 5;
            } else if (cmd == BUZZER_CMD_ALARM_ON) {
                alarm_active = 1;
                phase = 0;
            } else if (cmd == BUZZER_CMD_ALARM_OFF) {
                alarm_active = 0;
                phase = 0;
            }
        }

        if (key_beep_ticks ||
            (alarm_active &&
             ((phase % ALARM_PERIOD_TICKS) < ALARM_ON_TICKS))) {
            BEEP_ON();
        } else {
            BEEP_OFF();
        }

        if (key_beep_ticks) key_beep_ticks--;

        if (alarm_active) phase++;
        else phase = 0;

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(10));
    }
}

/*==================== task_display ====================*/
static void Task_Display(void *pvParameters)
{
    TickType_t last_wake;
    SensorSnapshot snap;
    UiState ui;
    AlarmState alarm;
    uint8_t have_sensor = 0;

    (void)pvParameters;

    memset(&snap, 0, sizeof(snap));
    snap.min_mv = 0xFFFF;
    ui.page = 0;
    ui.keys = 0;
    alarm.active = 0;

    last_wake = xTaskGetTickCount();

    for (;;) {
        if (xQueuePeek(g_q_sensor_display, &snap, 0) == pdPASS)
            have_sensor = 1;

        xQueuePeek(g_q_ui_state, &ui, 0);
        xQueuePeek(g_q_alarm_state, &alarm, 0);

        if (have_sensor) {
            xSemaphoreTake(g_i2c_mutex, portMAX_DELAY);

            Draw_Page(ui.page,
                      snap.bus_mv,
                      snap.ina_raw,
                      snap.min_mv,
                      snap.max_mv,
                      (uint32_t)(snap.tick / configTICK_RATE_HZ),
                      ui.keys,
                      snap.temp_raw,
                      snap.temp_status,
                      alarm.active);

            xSemaphoreGive(g_i2c_mutex);
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(100));
    }
}

/* Original UART task: heartbeat/diagnostics only, no serial output. */
static void Task_UART(void *pvParameters)
{
    TickType_t last_wake = xTaskGetTickCount();
    uint8_t hb = 0;
    RtosDiagnostics diag;
    (void)pvParameters;
    for (;;) {
        hb = !hb;
        if (hb) GPIO_ResetBits(GPIOC, GPIO_Pin_13); else GPIO_SetBits(GPIOC, GPIO_Pin_13);
        diag.heap_now = (uint16_t)xPortGetFreeHeapSize();
        diag.heap_min = (uint16_t)xPortGetMinimumEverFreeHeapSize();
        diag.stack[0] = (uint16_t)uxTaskGetStackHighWaterMark(g_task_sample);
        diag.stack[1] = (uint16_t)uxTaskGetStackHighWaterMark(g_task_display);
        diag.stack[2] = (uint16_t)uxTaskGetStackHighWaterMark(g_task_input);
        diag.stack[3] = (uint16_t)uxTaskGetStackHighWaterMark(g_task_alarm);
        diag.stack[4] = (uint16_t)uxTaskGetStackHighWaterMark(g_task_buzzer);
        diag.stack[5] = (uint16_t)uxTaskGetStackHighWaterMark(g_task_uart);
        diag.stack[6] = (uint16_t)uxTaskGetStackHighWaterMark(g_task_modbus);
        taskENTER_CRITICAL(); g_diagnostics = diag; taskEXIT_CRITICAL();
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(500));
    }
}

static void Modbus_MakeRegisters(const MonitorSnapshot *monitor,
                                 uint8_t have_monitor, const UiState *ui,
                                 const RtosDiagnostics *diag, TickType_t now,
                                 uint32_t rx_errors, uint16_t *registers)
{
    const SensorSnapshot *snap = &monitor->sensor;
    uint16_t i;
    uint16_t status = 0;
    memset(registers, 0, MODBUS_REGISTER_COUNT * sizeof(uint16_t));
    if (have_monitor) {
        status |= 1U;
        if (snap->ina_raw < 0xFFFEU) status |= 2U;
        if (snap->temp_status == 1U) status |= 4U;
        if ((TickType_t)(now - snap->tick) <= pdMS_TO_TICKS(1500)) status |= 8U;
        status |= (uint16_t)((monitor->relay_on ? 0x20U : 0U) |
                             ((uint16_t)(monitor->relay_reason & 7U) << 6));
        registers[0] = snap->bus_mv;
        registers[1] = (uint16_t)snap->temp_raw;
        registers[2] = monitor->alarm_active;
        registers[3] = snap->temp_status;
        registers[5] = (uint16_t)snap->sequence;
        registers[6] = (uint16_t)(snap->sequence >> 16);
        registers[7] = (uint16_t)snap->i2c_stop_timeout;
        registers[8] = (uint16_t)(snap->i2c_stop_timeout >> 16);
        registers[22] = snap->electrical.flags;
        registers[23] = (uint16_t)snap->electrical.shunt_raw;
        registers[24] = (uint16_t)snap->electrical.current_raw;
        registers[25] = snap->electrical.power_raw;
        registers[26] = snap->electrical.cal_read;
        registers[27] = snap->electrical.mask_start;
        registers[28] = snap->electrical.mask_end;
        registers[29] = (uint16_t)snap->electrical.current_ua;
        registers[30] = (uint16_t)((uint32_t)snap->electrical.current_ua >> 16);
        registers[31] = (uint16_t)snap->electrical.power_uw;
        registers[32] = (uint16_t)((uint32_t)snap->electrical.power_uw >> 16);
    }
    if (g_ina226_id_ok) status |= 16U;
    registers[4] = (uint16_t)(status | 0x8000U); /* relay command fields present */
    registers[9] = diag->heap_now;
    registers[10] = diag->heap_min;
    for (i = 0; i < 7U; i++) registers[11U+i] = diag->stack[i];
    registers[18] = ui->keys;
    registers[19] = (uint16_t)(ui->page + 1U);
    registers[20] = (uint16_t)rx_errors;
    registers[21] = (uint16_t)(rx_errors >> 16);
    registers[33] = INA226_CAL_VALUE;
    registers[34] = INA226_SHUNT_UOHM;
    registers[35] = 0x0502U; /* same 36 regs; status adds relay command fields */
}

/* Send only complete responses; keep DE high until the last stop bit leaves. */
static void RS485_SendFrame(const uint8_t *data, uint16_t len)
{
    if (len == 0U) return;

    /* /RE is high during TX. Discard receiver noise while RO is disabled. */
    taskENTER_CRITICAL();
    /* Never start a delayed response over newly arriving bus traffic. */
    if (g_rtu_armed || g_rtu_guard || g_rtu_slot != RTU_NONE ||
        (USART1->SR & USART_SR_RXNE) != 0U ||
        uxQueueMessagesWaiting(g_rx_queue) != 0U) {
        g_rx_error_count++;
        taskEXIT_CRITICAL();
        return;
    }
    g_diag_tx_start++;
    g_rtu_tx = 1;
    g_rtu_tx_data = data;
    g_rtu_tx_len = len;
    g_rtu_tx_pos = 0;
    TIM3->DIER &= ~TIM_DIER_CC1IE;
    USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);
    RS485_TRANSMIT();
    USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
    taskEXIT_CRITICAL();
    /* Response buffer belongs to this task: keep it alive until TC IRQ. */
    while (g_rtu_tx) vTaskDelay(pdMS_TO_TICKS(1));
}

/* Frames are completed in TIM3/USART IRQs, never by a 1ms task timeout. */
static void Task_Modbus(void *pvParameters)
{
    uint8_t response[MODBUS_RESPONSE_CAPACITY];
    uint16_t registers[MODBUS_REGISTER_COUNT];
    MonitorSnapshot monitor;
    UiState ui;
    RtosDiagnostics diag;
    uint16_t n;
    uint8_t slot;
    uint8_t have_monitor;
    (void)pvParameters;
    memset(&monitor, 0, sizeof(monitor));
    memset(&ui, 0, sizeof(ui));
    (void)USART1->SR; (void)USART1->DR;
    Rtu_TimerStart();
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    for (;;) {
        if (xQueueReceive(g_rx_queue, &slot, portMAX_DELAY) == pdPASS) {
            g_diag_frames++;
            g_diag_last_len = g_rtu_frames[slot].len;
            have_monitor = (xQueuePeek(g_q_monitor_modbus, &monitor, 0) == pdPASS);
            xQueuePeek(g_q_ui_state, &ui, 0);
            taskENTER_CRITICAL(); diag = g_diagnostics; taskEXIT_CRITICAL();
            Modbus_MakeRegisters(&monitor, have_monitor, &ui, &diag,
                                 xTaskGetTickCount(), g_rx_error_count, registers);
            n = Modbus_BuildResponse(g_rtu_frames[slot].data,
                                    g_rtu_frames[slot].len,
                                    g_rtu_frames[slot].invalid,
                                    registers, response, sizeof(response));
            taskENTER_CRITICAL();
            g_rtu_frames[slot].state = RTU_FREE;
            taskEXIT_CRITICAL();
            RS485_SendFrame(response, n);
        }
    }
}

/*==================== main ====================*/
int main(void)
{
    BaseType_t ok_sample, ok_display, ok_input, ok_alarm;
    BaseType_t ok_buzzer, ok_uart, ok_modbus;
    uint16_t reg_fe, reg_ff;
    uint16_t cal_check = 0;
    uint8_t config_error, cal_error;
    UiState ui0;
    AlarmState alarm0;
    GPIO_All_Init();
    USART1_Init();
    I2C1_Init();
    TIM2_Microsecond_Init();
    Boot_Delay_ms(100);
    reg_fe = ina226_read_reg(0xFE);
    reg_ff = ina226_read_reg(0xFF);
    g_ina226_id_ok = ((reg_fe == 0x5449U) && (reg_ff == 0x2260U));
    config_error = ina226_write_reg(0x00, 0x4727);
    cal_error = ina226_write_reg(0x05, INA226_CAL_VALUE);
    if (ina226_read_checked(0x05, &cal_check) == 0U &&
        config_error == 0U && cal_error == 0U && cal_check == INA226_CAL_VALUE)
        g_ina226_cal_ok = 1;
    OLED_Init();
    g_i2c_mutex = xSemaphoreCreateMutex();
    g_q_monitor_modbus = xQueueCreate(1, sizeof(MonitorSnapshot));
    /* Queue only slot indices; frame payloads have static storage. */
    g_rx_queue = xQueueCreate(RTU_SLOT_COUNT, sizeof(uint8_t));
    g_q_sensor_display = xQueueCreate(1, sizeof(SensorSnapshot));
    g_q_sensor_alarm = xQueueCreate(1, sizeof(SensorSnapshot));
    g_q_sensor_uart = xQueueCreate(1, sizeof(SensorSnapshot));
    g_q_ui_state = xQueueCreate(1, sizeof(UiState));
    g_q_alarm_state = xQueueCreate(1, sizeof(AlarmState));
    g_q_buzzer_cmd = xQueueCreate(8, sizeof(BuzzerCommand));
    if ((g_i2c_mutex == NULL) || (g_q_monitor_modbus == NULL) ||
        (g_rx_queue == NULL) || (g_q_sensor_display == NULL) ||
        (g_q_sensor_alarm == NULL) || (g_q_sensor_uart == NULL) ||
        (g_q_ui_state == NULL) || (g_q_alarm_state == NULL) ||
        (g_q_buzzer_cmd == NULL)) vApplicationMallocFailedHook();
    ui0.page = 0; ui0.keys = 0; memset(&alarm0, 0, sizeof(alarm0));
    xQueueOverwrite(g_q_ui_state, &ui0);
    xQueueOverwrite(g_q_alarm_state, &alarm0);
    ok_sample = xTaskCreate(Task_Sample, "sample", 220, NULL, 2, &g_task_sample);
    ok_display = xTaskCreate(Task_Display, "display", 256, NULL, 1, &g_task_display);
    ok_input = xTaskCreate(Task_Input, "input", 128, NULL, 3, &g_task_input);
    ok_alarm = xTaskCreate(Task_Alarm, "alarm", 160, NULL, 2, &g_task_alarm);
    ok_buzzer = xTaskCreate(Task_Buzzer, "buzzer", 128, NULL, 3, &g_task_buzzer);
    ok_uart = xTaskCreate(Task_UART, "uart", 128, NULL, 1, &g_task_uart);
    ok_modbus = xTaskCreate(Task_Modbus, "modbus", 256, NULL, 2, &g_task_modbus);
    if ((ok_sample != pdPASS) || (ok_display != pdPASS) ||
        (ok_input != pdPASS) || (ok_alarm != pdPASS) ||
        (ok_buzzer != pdPASS) || (ok_uart != pdPASS) ||
        (ok_modbus != pdPASS)) vApplicationMallocFailedHook();
    vTaskStartScheduler();
    vApplicationMallocFailedHook();
    for (;;) ;
}
