#!/usr/bin/env python3
# A test tailnet's coordination server as Tailscale's own answers a web page: Headscale behind a
# proxy that adds `Access-Control-Allow-Origin: *` to its HTTP answers, as controlplane.tailscale.com
# sends it, so the page's Tailscale node may read the server's key. WebSocket upgrades (the control
# protocol, DERP) pass through untouched. For check/tailscale_check.js only.
#
#   python3 check/cors_proxy.py <listen port> <upstream port>
import asyncio, sys

LISTEN, UPSTREAM = int(sys.argv[1]), int(sys.argv[2])
CORS = b"Access-Control-Allow-Origin: *\r\nAccess-Control-Allow-Headers: *\r\nAccess-Control-Allow-Methods: *\r\n"

async def pipe(reader, writer):
    try:
        while data := await reader.read(65536):
            writer.write(data)
            await writer.drain()
    except ConnectionError:
        pass
    finally:
        writer.close()

async def handle(client_reader, client_writer):
    head = await client_reader.readuntil(b"\r\n\r\n")
    if head.startswith(b"OPTIONS "):
        client_writer.write(b"HTTP/1.1 204 No Content\r\n" + CORS + b"Content-Length: 0\r\n\r\n")
        await client_writer.drain()
        client_writer.close()
        return
    upstream_reader, upstream_writer = await asyncio.open_connection("127.0.0.1", UPSTREAM)
    upgrade = b"\r\nupgrade:" in head.lower()
    if not upgrade:
        # One request per connection: the answer's head gets the header, the rest passes.
        lines = head.split(b"\r\n")
        lines = [l for l in lines if not l.lower().startswith(b"connection:")]
        head = b"\r\n".join(lines[:-2]) + b"\r\nConnection: close\r\n\r\n"
    upstream_writer.write(head)
    await upstream_writer.drain()
    if upgrade:
        await asyncio.gather(pipe(client_reader, upstream_writer), pipe(upstream_reader, client_writer))
        return
    answer = await upstream_reader.readuntil(b"\r\n")
    client_writer.write(answer + CORS)
    await asyncio.gather(pipe(client_reader, upstream_writer), pipe(upstream_reader, client_writer))

async def main():
    server = await asyncio.start_server(handle, "127.0.0.1", LISTEN)
    print(f"cors_proxy: 127.0.0.1:{LISTEN} -> 127.0.0.1:{UPSTREAM}", flush=True)
    async with server:
        await server.serve_forever()

asyncio.run(main())
