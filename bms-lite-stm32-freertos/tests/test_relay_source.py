"""Extract real relay policy and map; test boundary behavior without hardware."""
from pathlib import Path
import re
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
source = (root / 'firmware' / 'main.c').read_text(encoding='utf-8-sig')


def function(name):
    match = re.search(r'static (?:void|uint16_t)\s+' + name + r'\([^;{}]*\)\s*\{', source)
    depth, end = 1, match.end()
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[match.start():end]


defines = '\n'.join(line for line in source.splitlines() if re.match(
    r'#define (?:RELAY_(?:WAIT|NORMAL|HOT|SENSOR|STALE|MAX_AGE_MS)|TEMP_ALARM_(?:ON|OFF)_RAW|INA226_(?:CAL_VALUE|SHUNT_UOHM))\s', line))
types = source[source.index('typedef struct {\n    int32_t current_ua;'):source.index('typedef enum {\n    BUZZER_CMD')]
prefix = r'''
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
typedef uint32_t TickType_t;
#define pdMS_TO_TICKS(x) (x)
#define MODBUS_REGISTER_COUNT 36U
static uint8_t g_ina226_id_ok = 1;
'''
cases = r'''
int main(void)
{
    SensorSnapshot s;
    AlarmState a;
    MonitorSnapshot m;
    UiState ui;
    RtosDiagnostics diag;
    uint16_t r[38];
    unsigned n = 0;
    memset(&s,0,sizeof(s)); memset(&a,0,sizeof(a));
    Relay_Evaluate(&a,&s,0,0);
    assert(!a.relay_on && a.relay_reason==RELAY_WAIT); n++;
    Relay_Evaluate(&a,&s,1,0);
    assert(!a.relay_on && a.relay_reason==RELAY_SENSOR); n++;
    s.tick=100; s.temp_status=1; s.temp_raw=25*16;
    Relay_Evaluate(&a,&s,1,100);
    assert(a.relay_on && !a.active && a.relay_reason==RELAY_NORMAL); n++;
    s.temp_raw=30*16-1; Relay_Evaluate(&a,&s,1,100);
    assert(a.relay_on && !a.active); n++;
    s.temp_raw=30*16; Relay_Evaluate(&a,&s,1,100);
    assert(!a.relay_on && a.active && a.relay_reason==RELAY_HOT); n++;
    s.temp_raw=29*16+1; Relay_Evaluate(&a,&s,1,100);
    assert(!a.relay_on && a.active); n++;
    s.temp_raw=29*16; Relay_Evaluate(&a,&s,1,100);
    assert(a.relay_on && !a.active); n++;
    s.temp_raw=-10*16; Relay_Evaluate(&a,&s,1,100);
    assert(a.relay_on); n++;
    s.temp_status=2; Relay_Evaluate(&a,&s,1,100);
    assert(!a.relay_on && a.relay_reason==RELAY_SENSOR); n++;
    s.temp_status=3; Relay_Evaluate(&a,&s,1,100);
    assert(!a.relay_on && a.relay_reason==RELAY_SENSOR); n++;
    s.temp_status=1; Relay_Evaluate(&a,&s,1,1600);
    assert(a.relay_on); n++;
    Relay_Evaluate(&a,&s,1,1601);
    assert(!a.relay_on && a.relay_reason==RELAY_STALE); n++;
    s.tick=1602; Relay_Evaluate(&a,&s,1,1602);
    assert(a.relay_on); n++;
    s.tick=0xFFFFFFF0U; Relay_Evaluate(&a,&s,1,20);
    assert(a.relay_on); n++;
    s.tick=100; s.temp_raw=31*16; Relay_Evaluate(&a,&s,1,100);
    s.temp_status=2; Relay_Evaluate(&a,&s,1,101);
    assert(a.active && !a.relay_on && a.relay_reason==RELAY_SENSOR); n++;
    s.temp_status=1; s.temp_raw=29*16+1; Relay_Evaluate(&a,&s,1,102);
    assert(a.active && !a.relay_on); n++;
    s.temp_raw=29*16; Relay_Evaluate(&a,&s,1,102);
    assert(!a.active && a.relay_on); n++;
    memset(&m,0,sizeof(m));memset(&ui,0,sizeof(ui));memset(&diag,0,sizeof(diag));
    m.sensor=s;m.sensor.ina_raw=9000;m.sensor.bus_mv=11250;
    m.relay_on=a.relay_on;m.relay_reason=a.relay_reason;
    r[0]=0xAAAA; r[37]=0xBBBB;
    Modbus_MakeRegisters(&m,1,&ui,&diag,102,7,&r[1]);
    assert(r[0]==0xAAAA && r[37]==0xBBBB && r[36]==0x0502);
    assert((r[5]&31)==31 && (r[5]&0x8000) && (r[5]&32));
    assert(((r[5]>>6)&7)==RELAY_NORMAL && r[1]==11250); n++;
    m.relay_on=0;m.relay_reason=RELAY_STALE;
    Modbus_MakeRegisters(&m,1,&ui,&diag,2000,7,&r[1]);
    assert(!(r[5]&32) && !(r[5]&8) && ((r[5]>>6)&7)==RELAY_STALE); n++;
    Modbus_MakeRegisters(&m,0,&ui,&diag,2000,7,&r[1]);
    assert((r[5]&0x8000) && !(r[5]&32) && ((r[5]>>6)&7)==RELAY_WAIT); n++;
    printf("PASS: %u relay startup/hysteresis/fault/staleness/map scenarios\n",n);
    return 0;
}
'''
code = prefix + defines + '\n' + types + function('Relay_Evaluate') + function('Modbus_MakeRegisters') + cases
with tempfile.TemporaryDirectory(prefix='bms-relay-test-') as folder:
    cfile, exe = Path(folder) / 'relay.c', Path(folder) / 'relay.exe'
    cfile.write_text(code, encoding='utf-8')
    subprocess.run(['gcc', '-std=c90', '-Wall', '-Wextra', '-Werror', str(cfile), '-o', str(exe)], check=True)
    subprocess.run([str(exe)], check=True)
