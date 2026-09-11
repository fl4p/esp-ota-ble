import json,subprocess
from pathlib import Path
R=Path(__file__).resolve().parent

def remote(name,host,arguments):
    # Arguments are fixed experiment tokens; quote each individually for zsh/ssh.
    import shlex
    path=R/(name+'.log');assert not path.exists(),path
    command='cd ~/ota-throughput-20260911 && timeout --signal=INT 180s venv/bin/python '+shlex.join(arguments)
    with path.open('w') as f:
        run=subprocess.run(['ssh','-o','BatchMode=yes','-i',str(Path.home()/'.ssh/id_ed25519'),
                            'fab@'+host,command],stdout=f,stderr=subprocess.STDOUT)
    content=path.read_text()
    rows=[json.loads(s[7:]) for s in content.splitlines() if s.startswith('RESULT ')]
    print(name,'exit',run.returncode,({k:rows[-1].get(k) for k in ['receiver_kB_s','kB_s','image_kB_s','verified','real_write','post_link']} if rows else content[-1500:]),flush=True)
    if run.returncode or not rows or not rows[-1]['verified']:raise RuntimeError(path)
    return rows[-1]
