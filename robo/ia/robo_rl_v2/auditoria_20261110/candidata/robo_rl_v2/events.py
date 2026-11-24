"""Minimal TensorBoard scalar Event protobuf + TFRecord writer; no TensorFlow dependency.
Wire schema: tensorflow/core/util/event.proto and tensorflow/core/framework/summary.proto.
"""
import os, socket, struct, time
from pathlib import Path

def varint(n):
    out=bytearray()
    while n>127: out.append((n&127)|128);n>>=7
    out.append(n);return bytes(out)

def field(tag, payload): return varint((tag<<3)|2)+varint(len(payload))+payload

def crc32c(data):
    crc=0xffffffff
    for b in data:
        crc^=b
        for _ in range(8): crc=(crc>>1)^(0x82f63b78 if crc&1 else 0)
    return crc^0xffffffff

def masked_crc(data):
    c=crc32c(data);return struct.pack('<I',(((c>>15)|(c<<17))+0xa282ead8)&0xffffffff)

class ScalarEvents:
    def __init__(self, folder):
        Path(folder).mkdir(parents=True,exist_ok=True)
        self.fp=(Path(folder)/f'events.out.tfevents.{time.time_ns()}.{socket.gethostname()}.{os.getpid()}').open('wb')
        self.record(field(3,b'brain.Event:2'))
    def record(self, event):
        size=struct.pack('<Q',len(event))
        self.fp.write(size+masked_crc(size)+event+masked_crc(event));self.fp.flush()
    def scalar(self, tag, value, step):
        value_message=field(1,tag.encode())+b'\x15'+struct.pack('<f',float(value))
        event=b'\x09'+struct.pack('<d',time.time())+b'\x10'+varint(int(step))+field(5,field(1,value_message))
        self.record(event)
    def close(self): self.fp.close()
