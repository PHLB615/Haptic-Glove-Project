import asyncio
from bleak import BleakClient, BleakScanner

# UUIDs for Nordic UART Service (used by Bluefruit LE)
UART_SERVICE_UUID = "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
UART_RX_UUID      = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"  # Notify from device
UART_TX_UUID      = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"  # Write to device

# Name of your Bluefruit module (change if needed!)
DEVICE_NAME = "Adafruit Bluefruit LE"

async def run():
    print("Scanning for BLE devices...")
    devices = await BleakScanner.discover()

    target = None
    for d in devices:
        print(f"Found: {d.name} [{d.address}]")
        if d.name == DEVICE_NAME:
            target = d
            break

    if not target:
        print("Device not found. Make sure it's powered on and advertising.")
        return

    print(f"Connecting to {target.name} ({target.address})")
    async with BleakClient(target.address) as client:

                # Message buffering
        message_buffer = ""

        def handle_rx(_, data):
            nonlocal message_buffer  # allow modifying outer variable
            text = data.decode("utf-8")
            message_buffer += text

            if "\n" in message_buffer:
                lines = message_buffer.split("\n")
                for line in lines[:-1]:
                    print("Received:", line.strip())
                message_buffer = lines[-1]  # keep leftover chunk


        await client.start_notify(UART_RX_UUID, handle_rx)
        print("Listening for Bluetooth data... Press Ctrl+C to stop.\n")

        while True:
            await asyncio.sleep(1)

asyncio.run(run())