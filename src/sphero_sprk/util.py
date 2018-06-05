#!/usr/bin/python

from bluepy.btle import Scanner


def search_for_sphero(second_time=1):
	scanner = Scanner()
	devices = scanner.scan(second_time)
	sphero_list = []
	for dev in devices:
		#get the complete local name
		local_name = dev.getValueText(9)
		if local_name != None:
			if local_name.startswith('SK-'):
				sphero_list.append(dev.addr)
	return sphero_list

#function for conversion int to bytes, exists in python3
def to_bytes(n, length, endianess='big'):
        h = '%x' % n
        s = ('0'*(len(h) % 2) + h).zfill(length*2).decode('hex')
        return s if endianess == 'big' else s[::-1]

#function for conversion bytes to int, exists in python3
def from_bytes (data, big_endian = False):
    if isinstance(data, str):
        data = bytearray(data)
    if big_endian:
        data = reversed(data)
    num = 0
    for offset, byte in enumerate(data):
        num += byte << (offset * 8)
    return num

def cal_packet_checksum(arr):
	#value = con_b(0,1,'big');
	value=0
	for a in arr:
		for i in range(0,len(a)):
			if isinstance(a[i], str):
				value += ord(a[i])
			elif isinstance(a[i],int):
				value+=a[i]
	return 255-(value%256)
