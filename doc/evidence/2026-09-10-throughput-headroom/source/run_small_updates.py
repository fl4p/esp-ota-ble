from bench_driver import ROOT,IDENTITIES,install,call

for wire in ['raw','tamp','delta']:
    # Both slots deliberately contain Profile before each measured small edit.
    install('small-'+wire+'-seed1','Profile')
    install('small-'+wire+'-seed2','Profile')
    call('small-'+wire,ROOT/'node_SmallEdit.bin','--xform',wire,'--allow-skips',
         '--setup','link','--expect-base',IDENTITIES['Profile']['image_sha'])
install('small-tests-restore','Profile')
print('SMALL UPDATE MATRIX COMPLETE; Profile receiver left installed',flush=True)
