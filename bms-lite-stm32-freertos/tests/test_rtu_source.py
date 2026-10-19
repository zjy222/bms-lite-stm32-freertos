"""Compile the actual RTU core/protocol/TX source with host hardware stubs.

Run: python tests/test_rtu_source.py [path-to-main.c]
This verifies software state transitions, not UART wire timing or IRQ latency.
"""
from pathlib import Path
import subprocess
import sys
import tempfile

source = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(__file__).resolve().parents[1] / 'firmware' / 'main.c'
text = source.read_text(encoding='utf-8-sig')
core = text.split('/* RTU_RX_CORE_BEGIN:', 1)[1].split('/* RTU_RX_CORE_END */', 1)[0]
core = core.split('*/', 1)[1]
protocol = text[text.index('static uint16_t Modbus_CRC16('):text.index('/*==================== 页面绘制')]
tx = text[text.index('static const uint8_t *g_rtu_tx_data;'):text.index('void USART1_IRQHandler(void)')]
start = text.index('static void RS485_SendFrame(')
end = text.index('/* Frames are completed', start)
send = text[start:end]
prefix = r'''
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
typedef int BaseType_t;
#define pdPASS 1
#define MODBUS_BAUD_RATE 115200UL
#define MODBUS_SLAVE_ADDRESS 1U
static void *g_rx_queue;
static volatile uint32_t g_rx_error_count;
static uint8_t queue_data[3];
static unsigned queue_count;
static int xQueueSendFromISR(void *q, const void *item, BaseType_t *wake)
{
    (void)q; (void)wake;
    if (queue_count == 3U) return 0;
    queue_data[queue_count++] = *(const uint8_t *)item;
    return pdPASS;
}
'''
hardware = r'''
static void Rtu_ArmTimer(uint16_t now)
{
    g_rtu_last = now;
    g_rtu_armed = 1;
}
typedef struct { uint32_t CR1, SR, DR; } MockUart;
typedef struct { uint32_t CNT, DIER; } MockTimer;
static MockUart uart;
static MockTimer timer;
#define USART1 (&uart)
#define TIM3 (&timer)
#define USART_CR1_TXEIE 128U
#define USART_CR1_TCIE 64U
#define USART_SR_TXE 128U
#define USART_SR_TC 64U
#define USART_SR_RXNE 32U
#define USART_IT_TXE 128U
#define USART_IT_TC 64U
#define USART_IT_RXNE 32U
#define TIM_DIER_CC1IE 2U
#define ENABLE 1
#define DISABLE 0
static unsigned de, output_len, critical;
static uint8_t output[256];
#define RS485_RECEIVE() (de = 0)
#define RS485_TRANSMIT() (de = 1)
#define taskENTER_CRITICAL() (critical++)
#define taskEXIT_CRITICAL() (critical--)
#define pdMS_TO_TICKS(x) (x)
static void USART_ITConfig(MockUart *u, uint32_t flag, unsigned enable)
{
    if (enable) u->CR1 |= flag; else u->CR1 &= ~flag;
}
static void USART_SendData(MockUart *u, uint16_t byte)
{
    assert(de == 1U);
    output[output_len++] = (uint8_t)byte;
    u->SR &= ~(USART_SR_TC | USART_SR_TXE);
}
static unsigned uxQueueMessagesWaiting(void *q) { (void)q; return queue_count; }
static void vTaskDelay(unsigned ticks);
'''
cases = r'''
static void vTaskDelay(unsigned ticks)
{
    assert(ticks == 1U && critical == 0U);
    if (uart.CR1 & USART_CR1_TXEIE) uart.SR |= USART_SR_TXE;
    else uart.SR |= USART_SR_TC;
    Rtu_ServiceTx();
}
static BaseType_t wake;
static unsigned tests;
static void reset(void)
{
    memset(g_rtu_frames, 0, sizeof(g_rtu_frames));
    g_rtu_slot = RTU_NONE; g_rtu_guard = 0; g_rtu_armed = 0;
    g_rtu_dropping = 0; g_rtu_tx = 0; g_rtu_last = 0;
    g_rx_error_count = 0; queue_count = 0; wake = 0;
    memset(&uart, 0, sizeof(uart)); memset(&timer, 0, sizeof(timer));
    de = 0; output_len = 0; critical = 0;
    uart.SR = USART_SR_TXE | USART_SR_TC;
}
static void expire(uint16_t now)
{
    if (g_rtu_armed && (uint16_t)(now - g_rtu_last) >= RTU_END_US)
        Rtu_Finish(&wake);
}
static uint16_t feed(const uint8_t *data, unsigned len, uint16_t first)
{
    unsigned i;
    for (i = 0; i < len; i++) Rtu_ReceiveByte((uint16_t)(first + i*RTU_CHAR_US), data[i], 0, &wake);
    return (uint16_t)(first + (len - 1U)*RTU_CHAR_US);
}
static RtuFrame *published(unsigned index) { return &g_rtu_frames[queue_data[index]]; }
int main(void)
{
    const uint8_t request[8] = {1,3,0,0,0,4,0x44,0x09};
    uint8_t response[49], overflow[257];
    uint16_t registers[MODBUS_REGISTER_COUNT];
    uint16_t last, n;
    unsigned i;
    RtuFrame *frame;
    memset(registers, 0, sizeof(registers));
    registers[0] = 3320; registers[1] = 401; registers[3] = 1;

    reset(); last = feed(request, 8, 100);
    expire((uint16_t)(last + RTU_END_US - 1)); assert(queue_count == 0);
    expire((uint16_t)(last + RTU_END_US)); assert(queue_count == 1);
    frame = published(0); assert(frame->len == 8 && !memcmp(frame->data, request, 8));
    n = Modbus_BuildResponse(frame->data, frame->len, frame->invalid, registers, response, 49);
    assert(n == 13 && response[0] == 1 && response[2] == 8); tests++;

    reset(); Rtu_ReceiveByte(100, 1, 0, &wake);
    Rtu_ReceiveByte((uint16_t)(100 + RTU_INTERBYTE_LIMIT), 2, 0, &wake);
    expire((uint16_t)(g_rtu_last + RTU_END_US)); assert(queue_count == 1 && !g_rx_error_count); tests++;

    reset(); Rtu_ReceiveByte(100, 1, 0, &wake);
    Rtu_ReceiveByte((uint16_t)(101 + RTU_INTERBYTE_LIMIT), 2, 0, &wake);
    expire((uint16_t)(g_rtu_last + RTU_END_US)); assert(queue_count == 0 && g_rx_error_count == 1);
    last = feed(request, 8, (uint16_t)(g_rtu_last + RTU_END_US + 100));
    expire((uint16_t)(last + RTU_END_US)); assert(queue_count == 1); tests++;

    reset(); last = feed(request, 8, 100);
    last = feed(request, 8, (uint16_t)(last + RTU_END_US));
    assert(queue_count == 1); expire((uint16_t)(last + RTU_END_US));
    assert(queue_count == 2 && published(0) != published(1)); tests++;

    reset(); last = feed(request, 8, 65500);
    expire((uint16_t)(last + RTU_END_US)); assert(queue_count == 1 && published(0)->len == 8); tests++;

    reset(); g_rtu_guard = 1; Rtu_ArmTimer(0);
    last = feed(request, 8, 100); expire((uint16_t)(last + RTU_END_US));
    assert(queue_count == 0 && !g_rtu_guard);
    last = feed(request, 8, (uint16_t)(last + RTU_END_US + 100));
    expire((uint16_t)(last + RTU_END_US)); assert(queue_count == 1); tests++;

    reset(); Rtu_ReceiveByte(100, 1, 1, &wake);
    expire((uint16_t)(100 + RTU_END_US)); assert(queue_count == 0 && g_rx_error_count == 1); tests++;

    memset(overflow, 0x55, sizeof(overflow));
    reset(); last = feed(overflow, 256, 100); expire((uint16_t)(last + RTU_END_US));
    assert(queue_count == 1 && published(0)->len == 256); tests++;
    reset(); last = feed(overflow, 257, 100); expire((uint16_t)(last + RTU_END_US));
    assert(queue_count == 0 && g_rx_error_count == 1); tests++;

    reset(); last = 100;
    for (i = 0; i < 3; i++) {
        last = feed(request, 8, last); expire((uint16_t)(last + RTU_END_US));
        last = (uint16_t)(last + RTU_END_US + 100);
    }
    assert(queue_count == 3);
    last = feed(request, 8, last); expire((uint16_t)(last + RTU_END_US));
    assert(queue_count == 3 && g_rx_error_count == 1);
    for (i = 0; i < 3; i++) assert(!memcmp(published(i)->data, request, 8)); tests++;

    reset(); Rtu_ReceiveByte(100, 1, 0, &wake);
    queue_count = 3; expire((uint16_t)(100 + RTU_END_US));
    assert(g_rtu_frames[0].state == RTU_FREE && g_rx_error_count == 1); tests++;

    reset(); RS485_SendFrame(request, 0); assert(!output_len && !de); tests++;
    reset(); RS485_SendFrame(request, 8);
    assert(output_len == 8 && !memcmp(output, request, 8));
    assert(!de && !g_rtu_tx && g_rtu_guard && g_rtu_armed);
    assert((uart.CR1 & USART_IT_RXNE) != 0U); tests++;

    reset(); g_rtu_tx = 1; de = 1;
    g_rtu_tx_data = request; g_rtu_tx_len = 1; g_rtu_tx_pos = 0;
    uart.CR1 = USART_CR1_TXEIE; Rtu_ServiceTx();
    assert(de && output_len == 1);
    uart.SR = USART_SR_TXE; Rtu_ServiceTx(); assert(de && g_rtu_tx);
    uart.SR |= USART_SR_TC; Rtu_ServiceTx(); assert(!de && !g_rtu_tx); tests++;

    reset(); g_rtu_armed = 1; RS485_SendFrame(request, 8);
    assert(!de && output_len == 0 && g_rx_error_count == 1); tests++;
    reset(); uart.SR |= USART_SR_RXNE; RS485_SendFrame(request, 8);
    assert(!de && output_len == 0); tests++;
    reset(); queue_count = 1; RS485_SendFrame(request, 8);
    assert(!de && output_len == 0); tests++;

    reset(); memcpy(overflow, request, 8); overflow[6] ^= 1;
    assert(Modbus_BuildResponse(overflow, 8, 0, registers, response, 49) == 0); tests++;
    { const uint8_t other[8] = {2,3,0,0,0,2,0xc4,0x38};
      assert(Modbus_BuildResponse(other, 8, 0, registers, response, 49) == 0); tests++; }
    { uint8_t beyond[8] = {1,3,0,MODBUS_REGISTER_COUNT,0,1,0,0};
      uint16_t crc = Modbus_CRC16(beyond, 6);
      const uint8_t expected[5] = {1,0x83,2,0xc0,0xf1};
      beyond[6] = (uint8_t)crc; beyond[7] = (uint8_t)(crc >> 8);
      assert(Modbus_BuildResponse(beyond, 8, 0, registers, response, 49) == 5);
      assert(!memcmp(response, expected, 5)); tests++; }
    printf("PASS: %u RTU framing/protocol/TX scenarios\n", tests);
    return 0;
}
'''
# Temporary OLED diagnostics add observation counters in the TX path.
if 'g_diag_tx_start' in text:
    prefix += 'static volatile uint32_t g_diag_tx_start, g_diag_tx_done;\n'

with tempfile.TemporaryDirectory(prefix='bms-rtu-test-') as temp:
    cfile = Path(temp) / 'rtu_test.c'
    exe = Path(temp) / 'rtu_test.exe'
    cfile.write_text(prefix + core + hardware + protocol + tx + send + cases, encoding='utf-8')
    subprocess.run(['gcc', '-std=c90', '-Wall', '-Wextra', '-Werror', str(cfile), '-o', str(exe)], check=True)
    subprocess.run([str(exe)], check=True)
