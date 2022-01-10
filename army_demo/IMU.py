from um7py import UM7Serial
um7_serial = UM7Serial(port_name='/dev/ttyUSB0')
for packet in um7_serial.recv_all_proc_broadcast():
    print(f"packet: {packet}")