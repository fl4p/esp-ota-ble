import json,subprocess,time
from bench_driver import ROOT,PY,IDENTITIES,install,call

install('recover-poll-fast','Final')
call('final-dle2','/tmp/node_X.bin','--setup','dle2','--setup','link','--setup','dleinfo',
     '--expect-base',IDENTITIES['Final']['image_sha'])
install('final-dle2-restore','Final')
for burst in [2,4,8]:
    name='final-ram-burst'+str(burst)
    with (ROOT/(name+'.log')).open('w') as f:
        run=subprocess.run([PY,str(ROOT/'run_ram.py'),'Final','--bytes','1048576','--burst',str(burst)],
            stdout=f,stderr=subprocess.STDOUT,timeout=180)
    print(name,'exit',run.returncode,(ROOT/(name+'.log')).read_text().splitlines()[-1],flush=True)
    time.sleep(3)

install('combined-install','Combined')
rates=[]
for burst in [1,2]:
    name='combined-burst'+str(burst)
    row=call(name,'/tmp/node_X.bin','--burst',str(burst),'--setup','link',
             '--setup','buffers','--setup','dleinfo','--expect-base',IDENTITIES['Combined']['image_sha'])
    rates.append(row['kB_s'])
    install(name+'-restore','Combined')
label='Combined' if rates[0]>65 else 'Final'
if label=='Final':install('qualifying-install','Final')
print('Selected for repeats:',label,'combined rates',rates,flush=True)
for i in range(1,4):
    name='qualifying-'+str(i)
    row=call(name,'/tmp/node_X.bin','--setup','link','--setup','buffers','--setup','dleinfo',
        '--expect-base',IDENTITIES[label]['image_sha'])
    assert any('interval=8 mtu=517 phy_known=1 tx=2 rx=2' in s for s in row['negotiated_setup'])
    assert any('rx=251 ' in s for s in row['negotiated_setup'] if s.startswith('BENCH DLE '))
    install(name+'-restore',label)
for i in range(1,4):
    name='ram-long-'+str(i)
    with (ROOT/(name+'.log')).open('w') as f:
        subprocess.run([PY,str(ROOT/'run_ram.py'),label],stdout=f,stderr=subprocess.STDOUT,
            timeout=180,check=True)
    print(name,(ROOT/(name+'.log')).read_text().splitlines()[-1],flush=True)
    time.sleep(3)
print('ALL TRIALS COMPLETE; verified receiver',label,IDENTITIES[label],flush=True)
