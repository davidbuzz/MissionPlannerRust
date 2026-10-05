#!/usr/bin/env python3
# Serves www/ on 127.0.0.1:8080 with the headers gpui_web's threads need (SharedArrayBuffer):
# the same pair Zed's hello_web sets in its trunk.toml.
import http.server, sys, os

class Handler(http.server.SimpleHTTPRequestHandler):
    def end_headers(self):
        self.send_header("Cross-Origin-Opener-Policy", "same-origin")
        self.send_header("Cross-Origin-Embedder-Policy", "require-corp")
        self.send_header("Cache-Control", "no-store")
        super().end_headers()

Handler.extensions_map[".wasm"] = "application/wasm"
Handler.extensions_map[".js"] = "text/javascript"
os.chdir(os.path.dirname(os.path.abspath(__file__)))
port = int(sys.argv[1]) if len(sys.argv) > 1 else 8080
http.server.ThreadingHTTPServer(("127.0.0.1", port), Handler).serve_forever()
