import cantools


def get_bit_val(bytes_array, byte_pos, bit_pos):
    return (bytes_array[byte_pos] >> bit_pos) & 1


def decode_littleend_sig(bytes_array, start, len):
    val = 0
    byte = start // 8
    bit = start - byte * 8

    for i in range(len):
        val = val | (get_bit_val(bytes_array, byte, bit) << i)
        if bit == 7:
            bit = 0
            byte += 1
        else:
            bit += 1

    return val


def decode_bigend_sig(bytes_array, start, len):
    val = 0
    byte = start // 8
    bit = start - byte * 8

    for i in range(len):
        val = val | (get_bit_val(bytes_array, byte, bit) << (len - 1 - i))

        if bit == 0:
            bit = 7
            byte += 1
        else:
            bit -= 1

    return val


def read_msg(msg_obj, msg):
    bytes_array = bytes.fromhex(msg)
    values = []
    for sig in msg_obj.signals:
        if sig.byte_order == "little_endian":
            value = decode_littleend_sig(bytes_array, sig.start, sig.length)
        else:
            value = decode_bigend_sig(bytes_array, sig.start, sig.length)

        if sig.is_signed and (value & (1 << (sig.length - 1))):
            value -= 1 << sig.length

        value = value * sig.scale + sig.offset
        values.append(value)

    return [values, msg_obj]


db = cantools.database.load_file("iBMS.dbc", strict=False)

print(read_msg(db.get_message_by_name("BMS_CumuTimeInfor208"), "ff800080000000ff"))
