"""
Simple example of how to receive data on the hackRF
"""

import time

import libhackrf

MY_FREQUENCY = 1234567890
SAMPLE_RATE = 2000000
LOOP_WAIT = 0.1

# init hackRF
hack_rf = libhackrf.HackRF()

# set frequency and sample rate parameters
hack_rf.set_freq(freq=MY_FREQUENCY)
hack_rf.set_sample_rate(rate=SAMPLE_RATE)

# start receiving data
rx_data = bytearray()
hack_rf.start_rx()

# keep receiving data to rx_data until Ctrl+C is received
while True:
    try:
        rx_data += hack_rf.get_buffer()
        time.sleep(LOOP_WAIT)

    except KeyboardInterrupt:
        break

# stop receiving data
hack_rf.stop_rx()
