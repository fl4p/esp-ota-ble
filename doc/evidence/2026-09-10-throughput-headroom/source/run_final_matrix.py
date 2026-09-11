import json,subprocess,time
from bench_driver import ROOT,PY,IDENTITIES,install,call

install('final-install','Final')
for name,extra in [('final-poll-zero',['--poll-s','0']),
                   ('final-poll-fast',['--poll-s','0.0001']),
                   ('final-dle2',['--setup','dle2'])]:
    call(name,'/tmp/node_X.bin',*extra,'--setup','link','--setup','dleinfo',
         '--expect-base',IDENTITIES['Final']['image_sha'])
    install(name+'-restore','Final')

for burst in [2,4,8]:
    name='final-ram-burst'+str(burst)
    with (ROOT/(name+'.log')).open('w') as f:
        run=subprocess.run([PY,str(ROOT/'run_ram.py'),'Final','--bytes','1048576','--burst',str(burst)],
            stdout=f,stderr=subprocess.STDOUT,timeout=180)
    print(name,'exit',run.returncode,(ROOT/(name+'.log')).read_text().splitlines()[-1],flush=True)
    time.sleep(3)

print('FINAL PACING MATRIX COMPLETE; Final receiver installed',flush=True)
