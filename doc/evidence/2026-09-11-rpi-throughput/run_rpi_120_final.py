import json
from run_rpi_bumble import trial, R
from bench_driver import IDENTITIES

opts = dict(BENCH_PHY=2, BENCH_MAC_DISCOVERY=1,
            BENCH_HCI_PACKET_OVERRIDE=251, BENCH_INTERVAL_MS=12.5)
adapter = 'AC:A7:F1:83:27:AD'
trial('rpi-usb-hci251-long-integrity',
      ['run_ram.py','Graceful','--bytes','6291456','--adapter',adapter,
       '--interval','10','--chunk','495'], **opts)
rates=[]
for n in range(1,4):
    name='rpi-usb-raw125-repeat-%d'%n
    row=trial(name,['node/tools/bench_ble_ota.py','node_X.bin','--adapter',adapter,
                  '--chunk','495','--setup','interval 10','--setup','link',
                  '--expect-base',IDENTITIES['Graceful']['image_sha'],
                  '--results','rpi-bumble-flash.jsonl'],**opts)
    if row.get('real_write') is not True:
        raise RuntimeError('not a full rewrite')
    rates.append(row['kB_s'])
    trial(name+'-restore',['node/tools/bench_ble_ota.py','node_Graceful.bin',
          '--adapter',adapter,'--chunk','495','--delta',
          '--results','rpi-bumble-flash.jsonl'],**opts)
summary=dict(raw_rates=rates,mean=sum(rates)/len(rates),settings=opts,
             receiver='Graceful',adapter=adapter,nonstandard_hci_override=True)
(R/'rpi-120-repeat-summary.json').write_text(json.dumps(summary,indent=2)+'\n')
print('FINAL',json.dumps(summary),flush=True)
