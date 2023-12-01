import time
import threading
import libcsp_py3 as csp
from typing import Any, Callable

def printer(node: str, color: str) -> Callable:
    def f(inp: str) -> None:
        print('{color}[{name}]: {inp}\033[0m'.format(
            color=color, name=node.upper(), inp=inp))
    return f

def client_task(addr: int, port: int) -> None:
    _print = printer('client', '\033[92m')
    _print('Starting client task')

    while 1:
        time.sleep(1)

        ping = csp.ping(addr, 1000, 100, csp.CSP_O_NONE)
        _print('Ping {addr}: {ping}ms'.format(addr=addr, ping=ping))

def tx():
    print("this is called when interface tries to send data")

csp.init("", "", "")
csp.route_start_task()
# use basic interface
csp.basic_init("BASIC", tx)

serv_addr = 1
serv_port = 10

# open client and send ping packet
t = threading.Thread(target=client_task, args=(serv_addr, serv_port))
t.start()
