#!/usr/bin/python

from bluepy.btle import Scanner


def search_for_sphero(second_time=1):
    scanner = Scanner()
    devices = scanner.scan(second_time)
    sphero_list = []
    for dev in devices:
        # get the complete local name
        local_name = dev.getValueText(9)
        if local_name is not None:
            if local_name.startswith('SK-'):
                sphero_list.append(dev.addr)
    return sphero_list


def to_bytes(n, length, endianess='big'):
    """Convert int to bytes (exists in python3)."""
    h = '%x' % n
    s = ('0' * (len(h) % 2) + h).zfill(length * 2).decode('hex')
    return s if endianess == 'big' else s[::-1]


def from_bytes(data, big_endian=False):
    """Convert bytes to int (exists in python3)."""
    if isinstance(data, str):
        data = bytearray(data)
    if big_endian:
        data = reversed(data)
    num = 0
    for offset, byte in enumerate(data):
        num += byte << (offset * 8)
    return num


def cal_packet_checksum(arr):
    # value = con_b(0,1,'big');
    value = 0
    print('arr')
    print(arr)
    for a in arr:
        for i in range(0, len(a)):
            if isinstance(a[i], str):
                value += ord(a[i])
            elif isinstance(a[i], int):
                value += a[i]
            # print(type(a[i]))
            # print(type(value))
            print(repr("%s" % value))
        # print(int.from_bytes(a,'big'))
        # print(value)
    # return value
    return 255 - (value % 256)
    # return 255-(int(value.encode('hex'),16)%256)


def package_validator(data):
    if(len(data) < 5):
        return False
    if(data[0] != 255):
        return False

    # check dlen
    if(data[4] != 255):
        if(data[4] != len(data[4:-1])):
            return False

    # now we check the checksum
    data_pack = data[2:-1]  # from DID to second last, exclude checksum
    checksum = cal_packet_checksum([data_pack])
    return (checksum == data[-1])


def OR_mask(b1, b2):
    if(len(b1) != len(b2)):
        raise Exception("OR bytes with different length")
    arr = []
    for i in range(0, len(b1)):
        arr.append(b1[i] | b2[i])
    return bytes(arr)


def XOR_mask(b1, b2):
    if(len(b1) != len(b2)):
        raise Exception("XOR bytes with different length")
    arr = []
    for i in range(0, len(b1)):
        arr.append(b1[i] ^ b2[i])
    return bytes(arr)


def count_data_size(arr_list):
    val = 0
    for l in arr_list:
        val += len(l)
    return val
