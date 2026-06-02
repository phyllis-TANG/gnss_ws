#!/usr/bin/env python3
"""
probe_obs_fields.py
Inspect obs object fields available from rinex_utils,
and survey del2AINLOS ML classifier inputs.
Run inside container.
"""
import os, sys

for _p in [
    '/root/gnss_ws/src/PSRI-73-2309-PR-Dev-main/rospak/src/del2AINLOS/scripts',
    '/root/gnss_ws/PSRI-73-2309-PR-Dev/rospak/src/del2AINLOS/scripts',
]:
    if os.path.isdir(_p) and _p not in sys.path:
        sys.path.insert(0, _p)

from rinex_utils import read_rinex_obs

OBS_FILE = '/root/urbannav_gnss/UrbanNav-HK-Medium-Urban-1.ublox.f9p.obs'

print('=== Inspecting obs object fields ===')
epochs = read_rinex_obs(OBS_FILE)
ep = epochs[0]
obs = ep.obs_list[0]

print(f'Epoch fields:  {[a for a in dir(ep)  if not a.startswith("_")]}')
print(f'Obs fields:    {[a for a in dir(obs) if not a.startswith("_")]}')
print(f'\nobs.sat_id:    {obs.sat_id}')
print(f'obs.sys:       {obs.sys}')
print(f'obs.pseudorange keys: {list(obs.pseudorange.keys()) if hasattr(obs,"pseudorange") else "N/A"}')

# Check for SNR / C/N0 fields
for attr in ['snr', 'cn0', 'signal_strength', 'snr_dict', 'doppler', 'carrier_phase']:
    if hasattr(obs, attr):
        val = getattr(obs, attr)
        print(f'obs.{attr}: {val}')
    else:
        print(f'obs.{attr}: NOT FOUND')

# Print full __dict__ if available
if hasattr(obs, '__dict__'):
    print(f'\nobs.__dict__ keys: {list(obs.__dict__.keys())}')
    for k, v in obs.__dict__.items():
        print(f'  {k}: {v}')

# Sample a few more obs to see SNR pattern
print('\n=== S1C/S2C check across 5 sats in first epoch ===')
for o in ep.obs_list[:5]:
    psr_keys = list(o.pseudorange.keys()) if hasattr(o, 'pseudorange') else []
    snr_val = getattr(o, 'snr', None) or getattr(o, 'cn0', None)
    print(f'  {o.sat_id}  psr_keys={psr_keys}  snr={snr_val}')
    if hasattr(o, '__dict__'):
        snr_candidates = {k: v for k, v in o.__dict__.items()
                          if 'snr' in k.lower() or 'cn' in k.lower() or 'S1' in k or 'S2' in k}
        if snr_candidates:
            print(f'    SNR candidates: {snr_candidates}')

# survey del2AINLOS ML files
print('\n=== del2AINLOS ML classifier survey ===')
base_dirs = [
    '/root/gnss_ws/src/PSRI-73-2309-PR-Dev-main/rospak/src/del2AINLOS',
    '/root/gnss_ws/PSRI-73-2309-PR-Dev/rospak/src/del2AINLOS',
]
for base in base_dirs:
    if not os.path.isdir(base):
        continue
    print(f'\nFound: {base}')
    for root, dirs, files in os.walk(base):
        dirs[:] = [d for d in dirs if d not in ('__pycache__', '.git')]
        for fn in files:
            if fn.endswith('.py'):
                fpath = os.path.join(root, fn)
                rel   = os.path.relpath(fpath, base)
                try:
                    txt = open(fpath).read(4000)
                    ml_kw = [k for k in ['classifier','RandomForest','SVM','sklearn',
                                         'torch','keras','train','predict','feature',
                                         'cn0','snr','C/N0','nlos_label']
                             if k.lower() in txt.lower()]
                    print(f'  {rel:50s}  kw={ml_kw}')
                except Exception:
                    print(f'  {rel}  (unreadable)')
