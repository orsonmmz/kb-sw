#!/usr/bin/env python3
from kicon_badge import KiconBadge

badge = KiconBadge('/dev/ttyACM0')
badge.reset()
badge.uart_send('badge test')
