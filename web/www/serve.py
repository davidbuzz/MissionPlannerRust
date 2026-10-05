#!/usr/bin/env python3
# Serves www/ on 127.0.0.1:8080 with the headers gpui_web's threads need (SharedArrayBuffer):
# the same pair Zed's hello_web sets in its trunk.toml.
#
#   serve.py [port] [cert.pem key.pem]
#
# With a certificate and its key, over https, as GitHub Pages serves the page: a page over https
# may not ask anything over plain http (check/tiles_check.js). A certificate of one's own will do:
#   openssl req -x509 -newkey rsa:2048 -nodes -days 30 -subj /CN=127.0.0.1 \
#       -keyout key.pem -out cert.pem
import http.server, ssl, sys, os

class Handler(http.server.SimpleHTTPRequestHandler):
    def end_headers(self):
        self.send_header("Cross-Origin-Opener-Policy", "same-origin")
        self.send_header("Cross-Origin-Embedder-Policy", "require-corp")
        self.send_header("Cache-Control", "no-store")
        super().end_headers()

Handler.extensions_map[".wasm"] = "application/wasm"
Handler.extensions_map[".js"] = "text/javascript"
tls = [os.path.abspath(path) for path in sys.argv[2:4]]
os.chdir(os.path.dirname(os.path.abspath(__file__)))
port = int(sys.argv[1]) if len(sys.argv) > 1 else 8080
server = http.server.ThreadingHTTPServer(("127.0.0.1", port), Handler)
if len(tls) == 2:
    context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    context.load_cert_chain(*tls)
    server.socket = context.wrap_socket(server.socket, server_side=True)
server.serve_forever()
