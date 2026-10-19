"""Offline plot of the real demo CSV. Requires matplotlib; does not fill missing values."""
from pathlib import Path
import csv
from datetime import datetime
import math

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

root = Path(__file__).resolve().parents[1]
with (root / 'docs/data/demo-session.csv').open(encoding='utf-8', newline='') as stream:
    rows = list(csv.DictReader(stream))
start = datetime.fromisoformat(rows[0]['time'])
times = [(datetime.fromisoformat(r['time']) - start).total_seconds() for r in rows]

def values(key):
    return [float(r[key]) if r['result'] == 'OK' and r[key] else math.nan for r in rows]

fig, axes = plt.subplots(3, 1, figsize=(11, 7), sharex=True,
                         gridspec_kw={'height_ratios': [2, 2, 1]})
axes[0].plot(times, values('temperature_c'), color='#cc4b37', marker='.', lw=1.7)
axes[0].axhline(30, color='#cc4b37', ls='--', lw=1, label='Trip: 30 C')
axes[0].axhline(29, color='#358856', ls='--', lw=1, label='Restore: <=29 C')
axes[0].set_ylabel('Temperature (C)')
axes[0].legend(loc='upper left', fontsize=9, frameon=False)
axes[1].plot(times, values('current_a'), color='#2466a2', marker='.', lw=1.7)
axes[1].set_ylabel('Current (A)')
axes[1].set_ylim(-0.025, 0.40)
axes[2].step(times, values('relay_on'), where='post', color='#6451a4', lw=1.7)
axes[2].set_ylabel('Relay command')
axes[2].set_yticks([0, 1], ['OFF', 'ON'])
axes[2].set_ylim(-0.15, 1.15)
axes[2].set_xlabel('Seconds from first CSV row (2026-10-19 12:58:12.714 +08:00)')
for ax in axes:
    ax.grid(alpha=0.2)
    for t, row in zip(times, rows):
        if row['result'] != 'OK':
            ax.axvspan(t - 0.45, t + 0.45, color='#b44b49', alpha=0.12)
    ax.set_xlim(0, times[-1])
fig.suptitle('Real hardware demo - offline CSV analysis', fontsize=15, x=0.53)
fig.text(0.12, 0.018,
         'Real recorded samples; missing values are left blank. Command is not contact feedback.\n'
         'Sensor conversion and relay action are asynchronous; current may change in the next sample.',
         fontsize=9, color='#444444')
fig.tight_layout(rect=[0, 0.065, 1, 0.95])
fig.savefig(root / 'docs/media/demo-trace.png', dpi=160, facecolor='white')
plt.close(fig)
