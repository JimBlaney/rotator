import asyncio

from .platform import Platform

class RotctlTCPServerProtocol(asyncio.Protocol):
    def __init__(self, platform: Platform):
        self.platform = platform
        self._recv_buffer = bytearray()
        self.transport = None
        self.connection_ready = asyncio.Event()
    
    def connection_made(self, transport: asyncio.Transport) -> None:
        print('rotctl connected')
        self.transport = transport
        self.connection_ready.set()
    
    def data_received(self, data: bytes) -> None:
        self._recv_buffer.extend(data)

        while b'\n' in self._recv_buffer:
            line, _, self._recv_buffer = self._recv_buffer.partition(b'\n')
            command = line.decode().strip()
            asyncio.create_task(self.process_command(command))

    async def process_command(self, command: str):
        # print(f'Received: {command}')
        if command == 'p':
            try:
                az, el = await self.platform.get_current_position()
                response = f'{az:.1f} {el:.1f}\nRPRT 0\n'
            except Exception as e:
                response = 'RPRT -1\n'
        elif command.upper().startswith("P"):
            try:
                parts = command[1:].strip().split()
                if len(parts) != 2:
                    raise ValueError('Expected azimuth and elevation')
                az = float(parts[0])
                el = float(parts[1])
                await self.platform.move(az, el)
                response = 'RPRT 0\n'
            except Exception as e:
                response = 'RPRT -1\n'
        else:
            response = 'RPRT -1\n'

        if self.transport:
            self.transport.write(response.encode())

    def connection_lost(self, exc: Exception) -> None:
        print('rotctl disconnected')
        