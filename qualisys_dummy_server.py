import socket
import struct
import threading
import time
from datetime import datetime
from enum import Enum

start = int(time.time() * 1_000_000)

class Type(Enum):
    Error = 0
    Command = 1
    XML = 2
    Data = 3  # real time data
    NoMoreData = 4 # empty packet
    C3Cfile = 5
    Event = 6
    Discover = 7
    QTMfile = 8

class Component(Enum):
    _6D = 5


QTM_RT_PACKET_HEADER_fmt = '<II' # little endian


def send_response(conn, type, data):
    if len(data) > (1 << 32):
        raise ValueError("data too long")

    header = struct.pack(QTM_RT_PACKET_HEADER_fmt, len(data)+9, type.value)
    packet = header + data.encode("ascii") + b'\0'
    conn.sendall(packet)



freq = 50
freq_div = 1
stream = False
udp_stream_ip = '127.0.0.1'
udp_stream_port = 12345
_6d = False
thread = threading.Thread()

def stream_data_udp(ip, port, freq):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    endpoint = (ip, port)
    tick = 0

    while True:
        QTM_RT_DATA_PACKET_HEADER_fmt = "<IIQII"
        QTM_RT_6DOF_HEADER_fmt = "<IIIHH"
        QTM_RT_6DOF_BODY_fmt = "<fff fff fff fff" # xyz, then rotMat

        # 6DOF component data. No Euler, No residual
        n_body = 3
        _6d_data = b''
        for i in range(n_body):
            x, y, z    = 0., 1., 2. + i
            r1, r2, r3 = 0., -1., 0.
            r4, r5, r6 = 1., 0., 0.
            r7, r8, r9 = 0., 0., 1.
            _6d_data += struct.pack(QTM_RT_6DOF_BODY_fmt,
                x,y,z,r1,r2,r3,r4,r5,r6,r7,r8,r9)

        _6d_header = struct.pack(QTM_RT_6DOF_HEADER_fmt,
            16 + n_body*12*4, Component._6D.value, n_body, 17, 5)

        # data packet header
        length = 24 + len(_6d_header) + len(_6d_data)
        timeUs = int(time.time() * 1_000_000) - start # us since start of program
        n_component = 1 # just one for sanity
        header = struct.pack(QTM_RT_DATA_PACKET_HEADER_fmt,
            length, Type.Data.value, timeUs, tick, n_component) # just 
        packet = header + _6d_header + _6d_data

        tick += 1

        sock.sendto(packet, endpoint)
        time.sleep(1. / freq)


def handle_command(conn, data, length):
    global freq_div, stream, udp_stream_ip, udp_stream_port, euler_6d, thread

    if data.startswith("Version"):
        spl = data.split(" ")
        if len(spl) == 1:
            send_response(conn, Type.Command, "Version set to 1.25")
        elif spl[1] == "1.1":
            send_response(conn, Type.Command, "Version set to 1.1")
        else:
            send_response(conn, Type.Error, "Version NOT supported")

    elif data.startswith("StreamFrames"):
        spl = data.split(" ")
        if "Stop" in spl:
            thread.join()
            stream = False
            return
        elif stream == True:
            send_response(conn, Type.Error, "Already streaming. Send Stop first")
            return

        stream = True

        for arg in spl:
            if arg.startswith("FrequencyDivisor"):
                freq_div = int(arg.split(":")[1])
            elif arg.startswith("AllFrames"):
                freq_div = 1
            elif arg.startswith("6D"):
                _6d = True
            elif arg.startswith("UDP"):
                argspl = arg.split(":")
                if len(argspl) == 1:
                    send_response(conn, Type.Error, "Parse Error")
                elif len(argspl) == 2:
                    udp_stream_port = int(argspl[1])
                elif len(argspl) == 3:
                    udp_stream_ip = argspl[1]
                    udp_stream_port = int(argspl[2])
                    if udp_stream_port < 1024 or udp_stream_port > 65535:
                        send_response(conn, Type.Error, "Port number out of range")
            elif arg.startswith("Skeleton"):
                if arg.split(":")[1] != "global":
                    send_response(conn, Type.Error, "Only global skeleton position implemented.")

        thread = threading.Thread(target=stream_data_udp, args=(udp_stream_ip, udp_stream_port, freq/freq_div))
        thread.daemon = True
        thread.start()



def handle_client(conn):
    #%% send welcome
    welcome = "QTM RT Interface connected."
    send_response(conn, Type.Command, welcome)
    #conn.sendall(welcome.encode("ascii"))

    while True:
        packet = conn.recv(1024)
        if not packet:
            break

        while len(packet) > 0:
            if len(packet) < 8:
                send_response(conn, Type.Error, "Parse Error, message too short.")

            # decode header
            length, type = struct.unpack(QTM_RT_PACKET_HEADER_fmt, packet[:8])
            data_ascii = packet[8:length].decode('ascii')

            print(f"Received packet of length {length} and type {Type(type)}: {data_ascii}")

            # Handle "Version" command
            match type:
                case Type.Command.value:
                    handle_command(conn, data_ascii, length-8)
                case _:
                    send_response(conn, Type.Error, f"Packet type {Type(type)} not implemented.")

            packet = packet[length:]


def start_server(host='127.0.0.1', port=22223):
    # default ports
    # 22222 legacy
    # 22223 little-endian
    # 22224 big-endian
    # 22226 QTM auto discover. not implemented
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind((host, port))
        s.listen()
        print(f"Dummy Qualisys RTS Server running on {host}:{port}")

        while True:
            global udp_stream_ip
            conn, addr = s.accept()
            udp_stream_ip = addr[0]
            with conn:
                handle_client(conn)

if __name__ == "__main__":
    start_server()
