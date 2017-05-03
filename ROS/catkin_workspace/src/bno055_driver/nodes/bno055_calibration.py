#!/usr/bin/env python
from __future__ import print_function, division

import sys
import time
import logging
from argparse import ArgumentParser

from Adafruit_BNO055.BNO055 import BNO055

logger = logging.getLogger(__name__)


def main():
    parser = ArgumentParser(description="armarx version script")

    verbose_group = parser.add_mutually_exclusive_group()
    verbose_group.add_argument('-v', '--verbose', action='store_true', help='be verbose')
    verbose_group.add_argument('-q', '--quiet', action='store_true', help='be quiet')

#    parser.add_argument('-s', '--serial-port', default='/dev/ttyUSB0', help='the serial port')
    parser.add_argument('-f', '--calibration-file', default='bno055.json', help='the output calibration file')

    args = parser.parse_args()

    if args.verbose:
        logging.basicConfig(level=logging.DEBUG)
    elif args.quiet:
        logging.basicConfig(level=logging.ERROR)

#    device = BNO055(serial_port=args.serial_port)
    device = BNO055()
    device.set_external_crystal(True)

    calibration_status = device.get_calibration_status()
    while calibration_status != (3, ) * 4:
        logger.debug('waiting for device to be fully calibrated. please rotate IMU.')
        logger.debug('calibration status is {} {} {} {} '.format(*calibration_status))

        try:
            time.sleep(1)
        except KeyboardInterrupt:
            logger.info('keyboard interrupt. existing')
            sys.exit()

    logger.info('done calibrating device.')
    # os.makedirs(os.path.dirname(calibration_file))
    device.write_calibration(args.calibration_file)


if __name__ == '__main__':
    main()
