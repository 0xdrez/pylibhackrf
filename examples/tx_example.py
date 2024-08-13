"""
Simple example of how to transmit data on the hackRF
"""

import time

import libhackrf

MY_FREQUENCY = 1234567890
SAMPLE_RATE = 2000000
LOOP_WAIT = 0.1
DATA_TO_SEND = bytearray("hello world", "utf-8")

# init hackRF
hack_rf = libhackrf.HackRF(mode="Tx")

# set frequency and sample rate parameters
hack_rf.set_freq(freq=MY_FREQUENCY)
hack_rf.set_sample_rate(rate=SAMPLE_RATE)

# set the data to the buffer
hack_rf.set_buffer(data=DATA_TO_SEND)

# start transmitting data
hack_rf.start_tx()

# busy wait until the data is done transmitting
while not hack_rf.get_buffer():
    time.sleep(LOOP_WAIT)

# stop transmitting data
hack_rf.stop_tx()
