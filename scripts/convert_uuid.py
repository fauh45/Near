# Source: https://stackoverflow.com/a/68125858/3588626

uuid = input("Enter a UUID: ")
uuid = uuid.replace("-", "")
uuid = uuid[::-1]  # reverse the string
hexArrayStr = ""
splitToTwos = map("".join, zip(*[iter(uuid)] * 2))
count = 0
for v in splitToTwos:
    count += 1
    hexArrayStr = hexArrayStr + ("0x" + (v[::-1]).lower())
    if count != 16:
        hexArrayStr = hexArrayStr + ", "
print(hexArrayStr)
