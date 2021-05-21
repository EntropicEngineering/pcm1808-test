#!/usr/bin/env python3

import serial
import wave
import argparse

SAMPLE_RATE = 41666.66667
SAMPLE_WIDTH = 24
BYTES_PER_SAMPLE = 4
CHANNELS = 2

parser = argparse.ArgumentParser('Record I2S from USB serial port as wav file.')
parser.add_argument('seconds', type=int, help='Number of seconds of data to record.')
parser.add_argument('port', help='USB CDC ACM port to read from.')
parser.add_argument('outfile', help='Name of file to save data to.')

args = parser.parse_args()

samples = round(args.seconds * SAMPLE_RATE * CHANNELS)
data = bytearray(samples * SAMPLE_WIDTH // 8)

with serial.Serial(args.port, 115200) as ser:
    for i in range(samples):
        b = ser.read(BYTES_PER_SAMPLE)
        offset = i * SAMPLE_WIDTH // 8
        data[offset] = b[0]
        data[offset + 1] = b[1]
        data[offset + 2] = b[2]
        # Drop high byte of serial data

with wave.open(args.outfile, 'wb') as wav:
    wav.setparams((CHANNELS, SAMPLE_WIDTH // 8, SAMPLE_RATE, samples, 'NONE', 'NONE'))
    wav.writeframesraw(data)
