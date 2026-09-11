from run_rpi_bumble import trial

for interval,ce,chunk in ((7.5,0,495),(15,15,495),(30,30,495),(60,60,495),(7.5,7.5,495)):
    trial('rpi-builtin-mac-i%s-ce%s-c%s'%(interval,ce,chunk),
          ['run_ram.py','Graceful','--bytes','262144','--adapter','2C:CF:67:AA:43:02','--interval','6','--chunk',str(chunk)],
          BENCH_INTERVAL_MS=interval,BENCH_CE_MS=ce,BENCH_PHY=1,
          BENCH_PUBLIC=1,BENCH_LEGACY_SCAN=1,BENCH_MAC_DISCOVERY=1)
