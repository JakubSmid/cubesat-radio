import time
import threading
import libcsp_py3 as csp
from typing import Any, Callable
import zmq.asyncio
import time
from bleak import BleakClient
import asyncio

CUBESAT_MAC = "EF:F5:8F:CC:B2:8D"
UART_TX_UUID = "6e400002-b5a3-f393-e0a9-e50e24dcca9e" #Nordic NUS characteristic for TX
UART_RX_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e" #Nordic NUS characteristic for RX

SERV_ADDR = 1
SERV_PORT = 10

ctx = zmq.asyncio.Context()

def rx_handler(sender, data):
    # print(data)
    csp.basic_rx(data)

async def basic_iface():
    async with BleakClient(CUBESAT_MAC) as cubesat_ble:
        if not cubesat_ble.is_connected:
            await cubesat_ble.connect()
        await cubesat_ble.start_notify(UART_RX_UUID, rx_handler)
        
        # Create a context and socket
        sub = ctx.socket(zmq.SUB)
        sub.bind("tcp://*:5555")

        # Subscribe to the topic
        sub.setsockopt(zmq.SUBSCRIBE, b"")
        
        print("starting basic")
        
        while(True):
            msg = await sub.recv()
            print(msg)
            await cubesat_ble.write_gatt_char(UART_TX_UUID, msg)

def printer(node: str, color: str) -> Callable:
    def f(inp: str) -> None:
        print('{color}[{name}]: {inp}\033[0m'.format(
            color=color, name=node.upper(), inp=inp))
    return f

def client_task(addr: int, port: int) -> None:
    _print = printer('client', '\033[92m')
    _print('Starting client task')
    time.sleep(5)

    while 1:
    	ping = csp.ping(10, 5000, 1, csp.CSP_O_NONE)
    	_print('Ping {addr}: {ping}ms'.format(addr=addr, ping=ping))
    """
        conn = csp.connect(csp.CSP_PRIO_NORM, 10, 10, 1000, csp.CSP_O_NONE)
        if conn is None:
            raise Exception('Connection failed')

        packet = csp.buffer_get(10)
        if packet is None:
            raise Exception('Failed to get CSP buffer')

        data = bytes('Hello', 'ascii') + b'\x00'

        csp.packet_set_data(packet, data)
        csp.send(conn, packet)
        time.sleep(30)"""

if __name__ == "__main__":
    csp.init("", "", "")
    csp.route_start_task()
    # use basic interface
    csp.basic_init("BASIC")
    
    cspThread = threading.Thread(target=client_task, args=(SERV_ADDR, SERV_PORT))
    cspThread.start()
    
    asyncio.run(basic_iface())
