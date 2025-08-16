# io/uart_rx.py — placeholder (pure stdlib approach)
# For a pure-stdlib build, implementing robust UART on Linux uses termios/fcntl.
# We'll add it in a later pack. For now this is a stub so imports succeed.
class UartReceiver:
    def __init__(self, dev: str, baud: int):
        self.dev = dev
        self.baud = baud
        self.last_rx_ms = 0
        self.packets = 0
        self.drops = 0

    def poll(self):
        # TODO: implement in later step
        return []
