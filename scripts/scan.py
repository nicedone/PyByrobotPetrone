import argparse

import logging
from bluepy import btle


def petrone_list(hci):
    logging.info('scan for petrones...')
    scanner = btle.Scanner(hci)
    devices = scanner.scan(5)

    ll = []
    for d in devices:
        if 'PETRONE' not in d.scanData.get(9, ''):
            continue
        ll.append(d)
    return ll


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Scan for petrone bluetooth.')
    parser.add_argument('-i', '--hci', action='store', type=int, default=0,
                        help='Interface number for scan')

    args = parser.parse_args()
    devices = petrone_list(args.hci)

    print('{} of Petrones found.'.format(len(devices)))
    for d in devices:
        print('{} {} {}'.format(d.addr, d.rssi, d.scanData.get(9, '')))
