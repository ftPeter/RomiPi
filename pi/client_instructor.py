print("client jiffy starting")
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((addr_str, port_int))

while True:
    s = input("c to continue>")

    if s is "c":
        print("continuing")
        s.send(b'ready')
    else:
        break

print("ending")
s.close()

