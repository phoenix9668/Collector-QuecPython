str_test = 'Quectel build a smarter world'


def str_to_hex(a, b=""):  # 可实现的功能：转成HEX格式，可选择加上需要的分隔符
    hexadecimal = ''.join([hex(ord(c)).replace('0x', b) for c in a])
    return hexadecimal, len(a)


def hex_to_str(a, b=''):  # 可实现的功能：转成STR，如果有分隔符，可以添加分割符参数转换
    string = ''.join(
        [chr(int(c.replace(b, ''), 16)) for c in [a[i:i + 2 + len(b)] for i in range(0, len(a), 2 + len(b))]])
    return string


hex_test = str_to_hex(str_test)
print(type(hex_test[0]))
print(hex_test[1])
print(hex_test)

# hex_test = str(hex_test[0])
str_test = hex_to_str(hex_test[0])
print(str_test)
