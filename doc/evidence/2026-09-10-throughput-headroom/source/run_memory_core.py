from bench_driver import ROOT,IDENTITIES,install,call

for label in ['Internal','Core1']:
    install('placement-'+label+'-install',label)
    for burst in [1,2]:
        name='placement-'+label+'-burst'+str(burst)
        call(name,'/tmp/node_X.bin','--burst',str(burst),'--setup','interval 8','--setup','link',
             '--expect-base',IDENTITIES[label]['image_sha'])
        install(name+'-restore',label)
print('MEMORY/CORE MATRIX COMPLETE; Core1 receiver left installed',flush=True)
