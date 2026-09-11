"""Apply the Pi's finer interval search to the strict native Mac sender."""
import json
import subprocess
from bench_driver import ROOT, PY, install

def trial(name, interval, chunk):
    path = ROOT / (name + '.log')
    if path.exists():
        raise RuntimeError('refusing to overwrite ' + str(path))
    with path.open('w') as f:
        rc = subprocess.run([PY, str(ROOT/'run_native_priority_queue.py'),
             'Graceful', name, '--interval', str(interval), '--chunk', str(chunk)],
             stdout=f, stderr=subprocess.STDOUT).returncode
    rows = [json.loads(s[7:]) for s in path.read_text().splitlines() if s.startswith('RESULT ')]
    if rc or not rows or not rows[-1]['verified'] or not rows[-1]['real_write']:
        raise RuntimeError('unverified: ' + str(path))
    row = rows[-1]
    print(name, row['kB_s'], row['verified'], row['real_write'], flush=True)
    with (ROOT/'mac-port-sweep.jsonl').open('a') as f:
        f.write(json.dumps(dict(name=name, interval=interval, chunk=chunk, result=row))+'\n')
    install(name+'-restore', 'Graceful')
    return row

if __name__ == '__main__':
    for interval,chunk in ((12,495),(10,495),(11,495),(13,495),(16,495),(24,495),(12,244),(10,244)):
        trial('mac-port-i%d-c%d'%(interval,chunk), interval, chunk)
