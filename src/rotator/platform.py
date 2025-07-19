import asyncio
import serial_asyncio

from .lookup import lookup_pose
from .lx16a import LX16AServo, LX16AProtocol, LX16AError
from .math_utils import get_positions

class Platform:

    DEG_PER_SEC = 100

    def __init__(self, serial_port: str, baud_rate: int, *args, **kwargs):
        self.serial_port = serial_port
        self.baud_rate = baud_rate
        self.controller = None
        self.transport = None
        self.servos = None
        self.commanded_azimuth = None
        self.commanded_elevation = None

    async def __aenter__(self):
        await self.initialize()
        return self
    
    async def __aexit__(self, exc_type, exc_val, exc_tb):
        await asyncio.sleep(1)
        await self.disengage_all()
        self.transport.close()

    async def initialize(self):
        loop = asyncio.get_running_loop()

        transport, protocol = await serial_asyncio.create_serial_connection(
            loop,
            LX16AProtocol,
            self.serial_port,
            self.baud_rate,
            bytesize=serial_asyncio.serial.EIGHTBITS,
            parity=serial_asyncio.serial.PARITY_NONE,
            stopbits=serial_asyncio.serial.STOPBITS_ONE
        )

        await protocol.connection_ready.wait()

        self.transport = transport
        self.controller = protocol
        self.servos = [
            LX16AServo(i+1, self.controller)
            for i in range(4)
        ]
        for servo in self.servos:
            await servo.initialize()

        pose = await asyncio.gather(*[servo.get_physical_angle() for servo in self.servos])
        # print(f'current pose: {pose}')
        
        self.commanded_azimuth, self.commanded_elevation = lookup_pose(*pose)
        print(f'initial az: {self.commanded_azimuth}, initial el: {self.commanded_elevation}')

    def get_current_position(self):
        return self.commanded_azimuth, self.commanded_elevation

    async def move(self, azimuth: float, elevation: float) -> None:
        azimuth = (360 + azimuth) % 360

        elevation = max(elevation, 50)
        elevation = min(elevation, 90)

        steps, arc_distance = get_positions(
            self.commanded_azimuth, self.commanded_elevation,
            azimuth, elevation,
            1
        )

        if not len(steps):
            # print(f'no steps for distance {arc_distance}, az: {self.commanded_azimuth} -> {azimuth}, el: {self.commanded_elevation} -> {elevation}')
            return

        deg_per_step = arc_distance / len(steps)
        sec_per_step = deg_per_step / Platform.DEG_PER_SEC

        # print(f'sec_per_step: {sec_per_step}')

        for step in steps:
            step = [x + 120 for x in step]
            await asyncio.gather(*[
                servo.move(pos) for servo, pos in zip(self.servos, step)
            ])
            # await asyncio.sleep(0.0001)

        self.commanded_azimuth = azimuth
        self.commanded_elevation = elevation

    async def disengage_all(self) -> None:
        await asyncio.gather(*[
            servo.disable_torque()
            for servo in self.servos
        ])
        disabled = await asyncio.gather(*[
            servo.is_torque_enabled(True)
            for servo in self.servos
        ])
        if not any(disabled):
            print('all servos disabled')

    async def disengage(self, id: int) -> None:
        await self.servos[id].disable_torque()

    async def engage(self, id: int) -> None:
        await self.servos[id].enable_torque()

    async def get_servo_angle(self, id: int) -> None:
        return await self.servos[id].get_physical_angle()

    async def adjust_servo_offset(self, id: int, offset: float) -> None:
        await self.servos[id].set_angle_offset(offset, True)