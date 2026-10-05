#!/usr/bin/env python3
# A WebSocket in front of a TCP MAVLink port, so the page's "WS" link (www/link.js, ?link=ws://...)
# can reach a SITL or a vehicle that only speaks TCP: what websockify does, for one port.
#
#   python3 tools/ws_relay.py [ws_port=5800] [tcp_host=127.0.0.1] [tcp_port=5760]
import asyncio, sys
import websockets

WS_PORT = int(sys.argv[1]) if len(sys.argv) > 1 else 5800
TCP_HOST = sys.argv[2] if len(sys.argv) > 2 else "127.0.0.1"
TCP_PORT = int(sys.argv[3]) if len(sys.argv) > 3 else 5760

async def relay(socket, _path=None):
    reader, writer = await asyncio.open_connection(TCP_HOST, TCP_PORT)
    print(f"ws_relay: client {socket.remote_address} <-> tcp:{TCP_HOST}:{TCP_PORT}", flush=True)

    async def from_vehicle():
        while data := await reader.read(4096):
            await socket.send(data)

    async def to_vehicle():
        async for message in socket:
            if isinstance(message, bytes):
                writer.write(message)
                await writer.drain()

    try:
        await asyncio.gather(from_vehicle(), to_vehicle())
    except (websockets.ConnectionClosed, ConnectionError):
        pass
    finally:
        writer.close()

async def main():
    async with websockets.serve(relay, "127.0.0.1", WS_PORT, max_size=None):
        print(f"ws_relay: ws://127.0.0.1:{WS_PORT} -> tcp:{TCP_HOST}:{TCP_PORT}", flush=True)
        await asyncio.Future()

asyncio.run(main())
