import subprocess
from bench_driver import ROOT,PY,IDENTITIES,install,call

def ram(name,label,*extra):
    path=ROOT/(name+'.log')
    assert not path.exists(),path
    with path.open('w') as f:
        subprocess.run([PY,str(ROOT/'run_ram.py'),label,'--bytes','1048576',*extra],
            stdout=f,stderr=subprocess.STDOUT,check=True,timeout=180)
    print(name,'completed; '+path.read_text().splitlines()[-1],flush=True)

install('radio-install','Radio')
for name,extra in [('radio-default',[]),('radio-dle2',['--setup','dle2'])]:
    call(name,'/tmp/node_X.bin','--setup','interval 8',*extra,'--setup','link',
         '--setup','buffers','--setup','dleinfo','--expect-base',IDENTITIES['Radio']['image_sha'])
    install(name+'-restore','Radio')
ram('radio-ram-pilot','Radio','--calibrate')
install('msys-install','Msys')
for burst in [1,2]:
    name='msys-burst'+str(burst)
    result=call(name,'/tmp/node_X.bin','--burst',str(burst),'--setup','interval 8',
         '--setup','link','--setup','buffers','--setup','dleinfo','--expect-base',IDENTITIES['Msys']['image_sha'])
    assert any('msys1=50 ' in s for s in result['negotiated_setup']),result['negotiated_setup']
    install(name+'-restore','Msys')
ram('msys-ram-pilot','Msys','--calibrate')
print('RADIO MATRIX COMPLETE; Msys receiver installed',flush=True)
