"""Extract actual checked I2C read/conversion/register-map C and test with stubs.

Host mocks verify software cleanup and arithmetic, not electrical bus timing.
"""
from pathlib import Path
import re
import subprocess
import sys
import tempfile

source_path = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(__file__).resolve().parents[1] / 'firmware' / 'main.c'
source = source_path.read_text(encoding='utf-8-sig')
map_revision = int(re.search(r'registers\[35\] = (0x[0-9A-Fa-f]+)', source).group(1), 16)


def function(name):
    match = re.search(r'(?:static )?(?:uint8_t|void)\s+' + name + r'\([^;{}]*\)\s*\{', source)
    depth, end = 1, match.end()
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[match.start():end]


types = source[source.index('typedef struct {\n    int32_t current_ua;'):source.index('typedef enum {\n    BUZZER_CMD')]
protocol = source[source.index('static uint16_t Modbus_CRC16('):source.index('/*==================== 页面绘制')]
prefix = r'''
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>
typedef uint32_t TickType_t;
#define INA226_ADDR 0x40
#define INA226_CAL_VALUE 2048U
#define INA226_CURRENT_LSB_UA 250L
#define INA226_SHUNT_UOHM 10000U
#define ELECTRICAL_VALID_MASK 0x7FU
#define MODBUS_SLAVE_ADDRESS 1U
#define pdMS_TO_TICKS(x) (x)
#define SET 1
#define RESET 0
#define ENABLE 1
#define DISABLE 0
#define SUCCESS 1
#define I2C_FLAG_BUSY 1
#define I2C_FLAG_ADDR 2
#define I2C_FLAG_BTF 3
#define I2C_FLAG_AF 4
#define I2C_EVENT_MASTER_MODE_SELECT 5
#define I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED 6
#define I2C_EVENT_MASTER_BYTE_TRANSMITTED 7
#define I2C_Direction_Transmitter 0
#define I2C_Direction_Receiver 1
#define I2C_NACKPosition_Current 0
#define I2C_NACKPosition_Next 1
#define I2C_CR1_STOP 512U
typedef struct { uint32_t CR1, SR1, SR2; } MockI2c;
static MockI2c hw;
#define I2C1 (&hw)
static unsigned failure, starts, stops, ack, pos, critical, read_index, af_cleared;
static uint16_t read_value;
static unsigned sample_mode, last_reg, mask_reads, bad_reg=255;
static uint32_t g_i2c_stop_timeout;
static uint8_t g_ina226_id_ok = 1;
static uint8_t g_ina226_cal_ok = 1;
#define taskENTER_CRITICAL() (critical++)
#define taskEXIT_CRITICAL() (critical--)
static void I2C_NACKPositionConfig(MockI2c *p, unsigned v) { (void)p; pos=v; }
static void I2C_AcknowledgeConfig(MockI2c *p, unsigned v) { (void)p; ack=v; }
static unsigned I2C_GetFlagStatus(MockI2c *p, unsigned flag)
{
    (void)p;
    if (flag==I2C_FLAG_BUSY) return failure==1;
    if (flag==I2C_FLAG_ADDR) return failure!=6 && !(sample_mode && last_reg==bad_reg);
    if (flag==I2C_FLAG_BTF) return failure!=7;
    if (flag==I2C_FLAG_AF) return failure==3;
    assert(0); return 0;
}
static unsigned I2C_CheckEvent(MockI2c *p, unsigned flag)
{
    (void)p;
    if (flag==I2C_EVENT_MASTER_MODE_SELECT) return failure!=(starts==1 ? 2U:5U);
    if (flag==I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) return failure!=3;
    if (flag==I2C_EVENT_MASTER_BYTE_TRANSMITTED) return failure!=4;
    assert(0); return 0;
}
static void I2C_GenerateSTART(MockI2c *p,unsigned v) { (void)p;(void)v;starts++; }
static void I2C_GenerateSTOP(MockI2c *p,unsigned v)
{
    (void)v;stops++;p->CR1=(failure==8)?I2C_CR1_STOP:0;
}
static void I2C_Send7bitAddress(MockI2c *p,unsigned addr,unsigned dir)
{
    (void)p;assert(addr==0x80U);assert(dir<=1);
    if(dir==I2C_Direction_Receiver) {
        read_index=0;
        if(sample_mode) {
            if(last_reg==1 || last_reg==4) read_value=0xFFFF;
            else if(last_reg==2) read_value=9600;
            else if(last_reg==3) read_value=0;
            else if(last_reg==5) read_value=2048;
            else if(last_reg==6) read_value=(mask_reads++==0 || sample_mode==2)?8:0;
            else assert(0);
        }
    }
}
static void I2C_SendData(MockI2c *p,unsigned v) { (void)p;last_reg=v; }
static uint8_t I2C_ReceiveData(MockI2c *p)
{
    (void)p;return (uint8_t)(read_index++==0 ? (read_value>>8):read_value);
}
static void I2C_ClearFlag(MockI2c *p,unsigned flag) { (void)p;(void)flag;af_cleared++; }
static uint8_t I2C_MasterStop(void) { I2C_GenerateSTOP(I2C1,ENABLE);return 0; }
'''
tests = r'''
static void reset_bus(void)
{
    memset(&hw,0,sizeof(hw));failure=starts=stops=critical=read_index=af_cleared=0;
    ack=1;pos=0;g_i2c_stop_timeout=0;
    sample_mode=mask_reads=0;bad_reg=255;
}
static void valid(ElectricalSnapshot *e)
{
    memset(e,0,sizeof(*e));e->cal_read=2048;e->mask_start=8;e->current_raw=1600;e->shunt_raw=1600;
}
int main(void)
{
    const uint16_t values[6]={0,0xFFFF,0xFFFE,0x8000,0x7FFF,0x0800};
    const uint8_t all_request[8]={1,3,0,0,0,36,0x45,0xd1};
    const uint8_t legacy_request[8]={1,3,0,0,0,22,0xc4,4};
    const uint8_t beyond[8]={1,3,0,36,0,1,0xc4,1};
    const uint8_t exception[5]={1,0x83,2,0xc0,0xf1};
    uint16_t output,n;
    uint16_t guarded[38];
    uint8_t response[79];
    unsigned i,checks=0;
    ElectricalSnapshot e;
    MonitorSnapshot monitor;
    UiState ui;
    RtosDiagnostics diag;
    for(i=0;i<6;i++) {
        reset_bus();read_value=values[i];output=0x1234;
        assert(ina226_read_checked(1,&output)==0 && output==values[i]);
        assert(ack==1 && pos==0 && critical==0 && stops==1);checks++;
    }
    for(i=1;i<=8;i++) {
        reset_bus();failure=i;output=0x1234;
        assert(ina226_read_checked(1,&output)==i && output==0x1234);
        assert(ack==1 && pos==0 && critical==0);
        assert(stops==(i==1 ? 0U:1U));
        if(i==8) assert(g_i2c_stop_timeout==1);
        if(i==3) assert(af_cleared==1);
        checks++;
    }
    valid(&e);Electrical_Convert(&e,9600,1,1);
    assert(e.flags==127 && e.current_ua==400000 && e.power_uw==4800000);checks++;
    valid(&e);e.current_raw=e.shunt_raw=-1600;Electrical_Convert(&e,9600,1,1);
    assert(e.flags==127 && e.current_ua==-400000 && e.power_uw==-4800000);checks++;
    valid(&e);e.current_raw=e.shunt_raw=-1;Electrical_Convert(&e,9600,1,1);
    assert(e.flags==127 && e.current_ua==-250 && e.power_uw==-3000);checks++;
    valid(&e);e.current_raw=e.shunt_raw=0;Electrical_Convert(&e,9600,1,1);
    assert(e.flags==127 && !e.current_ua && !e.power_uw);checks++;
    valid(&e);e.current_raw=e.shunt_raw=32766;Electrical_Convert(&e,28800,1,1);
    assert(e.flags==127 && e.current_ua==8191500 && e.power_uw==294894000);checks++;
    valid(&e);e.cal_read=0;Electrical_Convert(&e,9600,1,1);
    assert(!(e.flags&2) && !e.current_ua && !e.power_uw);checks++;
    valid(&e);e.mask_start|=4;Electrical_Convert(&e,9600,1,1);
    assert(!(e.flags&8) && !e.current_ua);checks++;
    valid(&e);e.mask_end=8;Electrical_Convert(&e,9600,1,1);
    assert(!(e.flags&64) && !e.current_ua);checks++;
    valid(&e);e.mask_start=0;Electrical_Convert(&e,9600,1,1);
    assert(!(e.flags&32) && !e.current_ua);checks++;
    valid(&e);Electrical_Convert(&e,9600,0,1);assert(e.flags==4 && !e.current_ua);checks++;
    valid(&e);Electrical_Convert(&e,9600,1,0);assert(!(e.flags&4) && !e.current_ua);checks++;
    valid(&e);e.shunt_raw=32767;Electrical_Convert(&e,9600,1,1);
    assert(!(e.flags&16) && !e.current_ua);checks++;
    valid(&e);e.shunt_raw=(int16_t)0x8000;Electrical_Convert(&e,9600,1,1);
    assert(!(e.flags&16));checks++;
    valid(&e);Electrical_Convert(&e,28801,1,1);assert(!(e.flags&16));checks++;

    reset_bus();sample_mode=1;Electrical_Sample(&e);
    assert(e.flags==127 && e.shunt_raw==-1 && e.current_raw==-1);
    assert(e.current_ua==-250 && e.power_uw==-3000);checks++;
    reset_bus();sample_mode=1;bad_reg=4;Electrical_Sample(&e);
    assert(!(e.flags&1) && !e.current_ua && !e.power_uw);checks++;
    reset_bus();sample_mode=2;Electrical_Sample(&e);
    assert(!(e.flags&64) && !e.current_ua && !e.power_uw);checks++;

    memset(&monitor,0,sizeof(monitor));memset(&ui,0,sizeof(ui));memset(&diag,0,sizeof(diag));
    monitor.sensor.ina_raw=9600;monitor.sensor.bus_mv=12000;monitor.sensor.tick=100;
    monitor.sensor.temp_raw=-160;monitor.sensor.temp_status=1;monitor.sensor.sequence=0x12345678;
    valid(&monitor.sensor.electrical);monitor.sensor.electrical.current_raw=-1600;
    Electrical_Convert(&monitor.sensor.electrical,9600,1,1);
    diag.heap_now=784;diag.heap_min=784;ui.page=3;ui.keys=15;
    guarded[0]=0xAAAA;guarded[37]=0xBBBB;
    Modbus_MakeRegisters(&monitor,1,&ui,&diag,200,0x12345678,&guarded[1]);
    assert(guarded[0]==0xAAAA && guarded[37]==0xBBBB);
    assert(guarded[1]==12000 && guarded[2]==(uint16_t)-160 && (guarded[5]&31)==31);
    assert(guarded[6]==0x5678 && guarded[7]==0x1234 && guarded[10]==784);
    assert(guarded[19]==15 && guarded[20]==4 && guarded[21]==0x5678 && guarded[22]==0x1234);
    assert(guarded[23]==127 && guarded[34]==2048 && guarded[35]==10000 && guarded[36]==0x0501);
    assert(((uint32_t)guarded[30] | ((uint32_t)guarded[31]<<16))==(uint32_t)-400000);
    assert(((uint32_t)guarded[32] | ((uint32_t)guarded[33]<<16))==(uint32_t)-4800000);checks++;
    memset(response,0xAA,sizeof(response));
    n=Modbus_BuildResponse(all_request,8,0,&guarded[1],response,77);
    assert(n==77 && response[2]==72 && response[77]==0xAA);
    assert(Modbus_CRC16(response,75)==(uint16_t)(response[75]|((uint16_t)response[76]<<8)));checks++;
    n=Modbus_BuildResponse(legacy_request,8,0,&guarded[1],response,77);
    assert(n==49 && response[2]==44);checks++;
    n=Modbus_BuildResponse(beyond,8,0,&guarded[1],response,77);
    assert(n==5 && !memcmp(response,exception,5));checks++;
    assert(Modbus_BuildResponse(all_request,8,0,&guarded[1],response,76)==0);checks++;
    printf("PASS: %u M5 I2C cleanup/conversion/register scenarios\n",checks);
    return 0;
}
'''
tests = tests.replace('guarded[36]==0x0501', 'guarded[36]==' + str(map_revision))
code = prefix + types + function('ina226_read_checked') + function('Electrical_Convert') + function('Electrical_Sample') + protocol + function('Modbus_MakeRegisters') + tests
with tempfile.TemporaryDirectory(prefix='bms-m5-test-') as folder:
    path = Path(folder)
    cfile, exe = path / 'm5.c', path / 'm5.exe'
    cfile.write_text(code, encoding='utf-8')
    subprocess.run(['gcc', '-std=c90', '-Wall', '-Wextra', '-Werror', str(cfile), '-o', str(exe)], check=True)
    subprocess.run([str(exe)], check=True)
