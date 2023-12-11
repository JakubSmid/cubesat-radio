from bleak import BleakClient
import asyncio
from time import sleep

CUBESAT_MAC = "EF:F5:8F:CC:B2:8D"
UART_TX_UUID = "6e400002-b5a3-f393-e0a9-e50e24dcca9e" #Nordic NUS characteristic for TX
UART_RX_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e" #Nordic NUS characteristic for RX

def notification_handler(sender, data):
    print(f"received: {data}")

async def main():
    async with BleakClient(CUBESAT_MAC) as cubesat_ble:
        if not cubesat_ble.is_connected:
            await cubesat_ble.connect()
        await cubesat_ble.start_notify(UART_RX_UUID, notification_handler)

        while(True):
            await asyncio.sleep(0.2)
            await cubesat_ble.write_gatt_char(UART_TX_UUID, bytes(f"0", 'utf-8'))
            await asyncio.sleep(0.2)
            await cubesat_ble.write_gatt_char(UART_TX_UUID, bytes("1", 'utf-8'))

asyncio.run(main())