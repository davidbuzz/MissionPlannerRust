"""How a GUI script's stand-in - the legacy vehicle, the RTK caster, the silent proxy - gets into
the background: the runner waits for a `setup` command to finish, and what it starts must outlive
it, still listening on the socket the command opened.

Where there is a fork (Linux, macOS) the process forks and the child serves, its stdio on
/dev/null. Windows has none: the script is started again as a detached process, its stdio its own
(so it holds none of the runner's handles, which would keep the runner waiting), and handed the
listening socket itself with `socket.share` - the port the parent bound, and wrote into a config
file, is the one served. The Windows GUI suite in the tiny10 VM runs these (2026-09-26).
"""
import os
import socket
import subprocess
import sys

# The argument that marks the process started to serve, on Windows.
CHILD = "--gui-background-child"


def child_listener():
    """In the process `serve_in_background` started on Windows: the listener it was handed, the
    marker taken off the arguments first. None in any other process."""
    if CHILD not in sys.argv:
        return None
    sys.argv.remove(CHILD)
    return socket.fromshare(sys.stdin.buffer.read())


def serve_in_background(listener, announce, serve):
    """`serve(listener)` in a process of its own; this one prints `announce` and returns."""
    if hasattr(os, "fork"):
        if os.fork() > 0:
            print(announce)
            return
        os.setsid()
        devnull = os.open(os.devnull, os.O_RDWR)
        for fd in (0, 1, 2):
            os.dup2(devnull, fd)
        serve(listener)
        os._exit(0)
    flags = subprocess.DETACHED_PROCESS | subprocess.CREATE_NEW_PROCESS_GROUP
    child = subprocess.Popen(
        [sys.executable, sys.argv[0], *sys.argv[1:], CHILD],
        stdin=subprocess.PIPE,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        creationflags=flags,
        close_fds=True,
    )
    child.stdin.write(listener.share(child.pid))
    child.stdin.close()
    listener.close()
    print(announce)
