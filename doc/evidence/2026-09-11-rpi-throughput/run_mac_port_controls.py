"""Current-session controls and larger writes after the interval sweep."""
from run_mac_port_sweep import trial

if __name__ == '__main__':
    for interval, chunk in ((8,512),(10,512),(8,495)):
        trial('mac-port-extra-i%d-c%d' % (interval,chunk), interval, chunk)
    for n in range(1,4):
        trial('mac-port-control-i8-c244-repeat-%d' % n, 8, 244)
