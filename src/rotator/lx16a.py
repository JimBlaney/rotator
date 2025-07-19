import asyncio
import enum
from typing import Union

class LX16ACommand(enum.Enum):
    pass

class LX16AWriteCommand(LX16ACommand):
    SERVO_MOVE_TIME           =  1
    SERVO_MOVE_TIME_WAIT      =  7
    SERVO_MOVE_START          = 11
    SERVO_MOVE_STOP           = 12
    SERVO_ID                  = 13
    SERVO_ANGLE_OFFSET_ADJUST = 17
    SERVO_ANGLE_OFFSET        = 18
    SERVO_ANGLE_LIMIT         = 20
    SERVO_VIN_LIMIT           = 22
    SERVO_TEMP_MAX_LIMIT      = 24
    SERVO_OR_MOTOR_MODE       = 29
    SERVO_LOAD_OR_UNLOAD      = 31
    SERVO_LED_CTRL            = 33
    SERVO_LED_ERROR           = 35

class LX16AReadCommand(LX16ACommand):
    SERVO_MOVE_TIME       =  2
    SERVO_MOVE_TIME_WAIT  =  8
    SERVO_ID              = 14
    SERVO_ANGLE_OFFSET    = 19
    SERVO_ANGLE_LIMIT     = 21
    SERVO_VIN_LIMIT       = 23
    SERVO_TEMP_MAX_LIMIT  = 25
    SERVO_TEMP            = 26
    SERVO_VIN             = 27
    SERVO_POS             = 28
    SERVO_OR_MOTOR_MODE   = 30
    SERVO_LOAD_OR_UNLOAD  = 32
    SERVO_LED_CTRL        = 34
    SERVO_LED_ERROR       = 36

class LX16AError(Exception):
    pass

class LX16ATimeoutError(LX16AError):
    pass

class LX16AChecksumError(LX16AError):
    pass

class LX16AArgumentError(LX16AError):
    pass

class LX16ALogicalError(LX16AError):
    pass

class LX16AProtocol(asyncio.Protocol):

    HEADER = [0x55, 0x55]

    @staticmethod
    def to_bytes(n: int) -> tuple[int, int]:
        return n & 0xFF, (n >> 8) & 0xFF
    
    @staticmethod
    def from_bytes(low: int, high: int) -> int:
        return (high << 8) | low

    @staticmethod
    def checksum(packet: list[int]) -> int:
        return (~sum(packet[2:])) & 0xFF

    @staticmethod
    def check_packet(packet: list[int], servo_id: int = -1) -> bool:
        if sum(packet) == 0:
            raise LX16ATimeoutError(f'servo {servo_id}: not responding', servo_id)
        if LX16AProtocol.checksum(packet[:-1]) != packet[-1]:
            raise LX16AChecksumError(f'servo {servo_id}: bad checksum', servo_id)
        return True

    def __init__(self) -> None:
        self.transport = None
        self._recv_buffer = bytearray()
        self._response_future = None
        self._expected_length = None
        self._lock = asyncio.Lock()
        self.connection_ready = asyncio.Event()

    def connection_made(self, transport: asyncio.Transport) -> None:
        print(f'platform connected')
        self.transport = transport
        self.connection_ready.set()

    def data_received(self, data: bytes) -> None:
        # print(f"[RX] {data.hex()}")
        self._recv_buffer.extend(data)
        if self._response_future and self._expected_length:
            # print(f'receive buffer size: {len(self._recv_buffer)} - expected: {self._expected_length}')
            if len(self._recv_buffer) >= self._expected_length:
                response = bytes(self._recv_buffer[:self._expected_length])
                self._recv_buffer = self._recv_buffer[self._expected_length:]
                if not self._response_future.done():
                    if not LX16AProtocol.check_packet(response):
                        raise LX16AChecksumError('bad checksum')
                    self._response_future.set_result(list(response[5:-1]))
                # self._response_future = None
                # self._expected_length = None

    def connection_lost(self, exc: Exception) -> None:
        print('platform disconnected')

    async def send_command(self, servo_id: int, cmd: LX16ACommand, params: list[int] = [], expect_response: bool = False, response_len: int = 0, timeout: float = 0.5) -> bytes | None:
        packet_len = 3 + len(params)
        packet = LX16AProtocol.HEADER + [servo_id, packet_len, cmd.value] + params
        checksum = LX16AProtocol.checksum(packet)
        packet.append(checksum)

        async with self._lock:
            if expect_response:
                self._response_future = asyncio.get_event_loop().create_future()
                self._expected_length = response_len + 6
                
            # print(f'[TX] {bytes(packet).hex()}')
            self.transport.write(bytes(packet))

            if expect_response:
                try:
                    response = await asyncio.wait_for(self._response_future, timeout)
                    return response
                except asyncio.TimeoutError:
                    print('TIMEOUT ERROR')
                    self._response_future = None
                    self._expected_length = None
                    return None
                
            await asyncio.sleep(0.0025)
            return None


class LX16AServo:
    
    @staticmethod
    def angle_to_position(angle: float) -> int:
        return int(round(angle * 25 / 6))
    
    @staticmethod
    def position_to_angle(position: int) -> float:
        return position * 6 / 25.0
    
    @staticmethod
    def check_within_limits(
        value: Union[float, int],
        lower_limit: Union[float, int],
        upper_limit: Union[float, int],
        variable_name: str,
        servo_id: int
    ) -> None:
        if value < lower_limit or value > upper_limit:
            raise LX16AArgumentError(
                f'servo {servo_id}: {variable_name} must be between {lower_limit} and {upper_limit} (received {value})',
                servo_id
            )
       
    def __init__(self, id: int, controller: LX16AProtocol, disable_torque: bool = False) -> None:
        LX16AServo.check_within_limits(id, 1, 253, 'servo_id', id)

        self.id = id
        self.controller = controller

        self.commanded_position = None
        self.waiting_position = None
        self.waiting_for_move = None
        self.angle_offset = None
        self.angle_limits = None
        self.vin_limits = None
        self.temp_limit = None
        self.motor_mode = None
        self.motor_speed = None
        self.torque_enabled = None
        self.led_powered = None
        self.led_error_triggers = None
        self.disable_torque_on_init = disable_torque

    async def initialize(self):
        self.commanded_position = LX16AServo.angle_to_position(await self.get_physical_angle())
        self.waiting_position = self.commanded_position
        self.waiting_for_move = False
        self.angle_offset = LX16AServo.angle_to_position(await self.get_angle_offset(True))
        self.angle_limits = tuple(map(LX16AServo.angle_to_position, await self.get_angle_limits(True)))
        self.vin_limits = await self.get_vin_limits(True)
        self.temp_limit = await self.get_temp_limit(True)
        self.motor_mode = await self.is_motor_mode(True)
        self.motor_speed = await self.get_motor_speed(True) if self.motor_mode else None
        self.torque_enabled = await self.is_torque_enabled(True)
        self.led_powered = await self.is_led_power_on(True)
        self.led_error_triggers = await self.get_led_error_triggers(True)
        self.position_offset = await self.get_angle_offset(True)

        if self.disable_torque_on_init:
            await self.disable_torque()
        else:
            await self.enable_torque()

    async def move(self, angle: float, time_ms: int = 0, relative: bool = False, wait: bool = False) -> None:
        if not self.torque_enabled:
            raise LX16ALogicalError(f'servo {self.id}: torque must be enabled to move', self.id)
        
        if self.motor_mode:
            raise LX16ALogicalError(f'servo {self.id}: motor mode must be disabled to control movement', self.id)
        
        LX16AServo.check_within_limits(angle, 0, 240, 'angle', self.id)
        LX16AServo.check_within_limits(
            angle, 
            LX16AServo.position_to_angle(self.angle_limits[0]),
            LX16AServo.position_to_angle(self.angle_limits[1]),
            'angle',
            self.id
        )

        position = self.angle_to_position(angle)

        if relative:
            position += self.commanded_position
        
        if wait:
            await self.controller.send_command(
                self.id,
                LX16AWriteCommand.SERVO_MOVE_TIME_WAIT,
                [*LX16AProtocol.to_bytes(position), *LX16AProtocol.to_bytes(time_ms)]
            )
            self.waiting_position = position
            self.waiting_for_move = True
        else:
            await self.controller.send_command(
                self.id,
                LX16AWriteCommand.SERVO_MOVE_TIME,
                [*LX16AProtocol.to_bytes(position), *LX16AProtocol.to_bytes(time_ms)],
            )
            self.commanded_position = position

    async def move_start(self) -> None:
        if not self.torque_enabled:
            raise LX16ALogicalError(f'servo {self.id}: torque must be enabled to move', self.id)
        
        if self.motor_mode:
            raise LX16ALogicalError(f'servo {self.id}: motor mode must be disabled to control movement', self.id)

        await self.controller.send_command(
            self.id,
            LX16AWriteCommand.SERVO_MOVE_START,
        )

        self.commanded_position = self.waiting_position
        self.waiting_for_move = False

    async def move_stop(self) -> None:
        if self.motor_mode:
            raise LX16ALogicalError(f'servo {self.id}: motor mode must be disabled to control movement', self.id)
        
        await self.controller.send_command(
            self.id,
            LX16AWriteCommand.SERVO_MOVE_STOP,
        )

        self.commanded_position = LX16AServo.angle_to_position(self.get_physical_angle())

    async def set_id(self, id: int) -> None:
        await self.controller.send_command(
            self.id,
            LX16AWriteCommand.SERVO_ID,
            [id]
        )
        self.id = id

    async def set_angle_offset(self, offset: int, permanent: bool = False) -> None:
        LX16AServo.check_within_limits(offset, -30, 30, "angle offset", self.id)
        
        offset = LX16AServo.angle_to_position(offset)
        if offset < 0:
            offset += 256

        await self.controller.send_command(
            self.id,
            LX16AWriteCommand.SERVO_ANGLE_OFFSET_ADJUST,
            [offset],
        )

        self.position_offset = offset

        if permanent:
            await self.controller.send_command(
                self.id,
                LX16AWriteCommand.SERVO_ANGLE_OFFSET,
            )

    async def set_angle_limits(self, lower_limit: float, upper_limit: float) -> None:
        LX16AServo.check_within_limits(lower_limit, 0, 240, "lower limit", self.id)
        LX16AServo.check_within_limits(upper_limit, 0, 240, "upper limit", self.id)

        if upper_limit < lower_limit:
            raise LX16AArgumentError(f'servo {self.id}: lower limit (received {lower_limit}) must be less than upper limit (received {upper_limit})', self.id)
        
        lower_limit = LX16AServo.angle_to_position(lower_limit)
        upper_limit = LX16AServo.angle_to_position(upper_limit)

        await self.controller.send_command(
            self.id,
            LX16AWriteCommand.SERVO_ANGLE_LIMIT,
            [*LX16AProtocol.to_bytes(lower_limit), *LX16AProtocol.to_bytes(upper_limit)]
        )

        self.angle_limits = lower_limit, upper_limit

    async def set_vin_limits(self, lower_limit: int, upper_limit: int) -> None:
        LX16AServo.check_within_limits(lower_limit, 4500, 12000, 'lower limit', self._id)
        LX16AServo.check_within_limits(upper_limit, 4500, 12000, 'upper limit', self._id)
        
        if upper_limit < lower_limit:
            raise LX16AArgumentError(f'servo {self.id}: lower limit (received {lower_limit}) must be less than upper limit (received {upper_limit})', self.id)

        await self.controller.send_command(
            self.id,
            LX16AWriteCommand.SERVO_VIN_LIMIT,
            [*LX16AProtocol.to_bytes(lower_limit), *LX16AProtocol.to_bytes(upper_limit)]
        )

        self.vin_limits = lower_limit, upper_limit

    async def set_temp_limit(self, upper_limit: int) -> None:
        LX16AServo.check_within_limits(upper_limit, 50, 100, 'temperature_limit', self.id)

        await self.controller.send_command(
            self.id,
            LX16AWriteCommand.SERVO_TEMP_MAX_LIMIT,
            [upper_limit]
        )

        self.temp_limit = upper_limit

    async def set_motor_mode(self, speed: int) -> None:
        if not self.torque_enabled:
            raise LX16ALogicalError(f'servo {self.id}: torque must be enabled to control movement', self.id)
        
        LX16AServo.check_within_limits(speed, -1000, 1000, 'motor speed', self.id)

        if speed < 0:
            speed += 65536

        await self.controller.send_command(
            self.id,
            LX16AWriteCommand.SERVO_OR_MOTOR_MODE,
            [1, 0, *LX16AProtocol.to_bytes(speed)]
        )

        self.motor_mode = True
        self.motor_speed = speed

    async def set_servo_mode(self) -> None:
        await self.controller.send_command(
            self.id,
            LX16AWriteCommand.SERVO_OR_MOTOR_MODE,
            [0, 0, 0, 0]
        )

        self.motor_mode = False

    async def enable_torque(self) -> None:
        await self.controller.send_command(
            self.id,
            LX16AWriteCommand.SERVO_LOAD_OR_UNLOAD,
            [0],
        )

        self.torque_enabled = True

    async def disable_torque(self) -> None:
        await self.controller.send_command(
            self.id,
            LX16AWriteCommand.SERVO_LOAD_OR_UNLOAD,
            [0],
        )

        self.torque_enabled = False

    async def set_led_power_off(self) -> None:
        await self.controller.send_command(
            self.id,
            LX16AWriteCommand.SERVO_LED_CTRL,
            [1]
        )

        self.led_powered = False

    async def set_led_power_on(self) -> None:
        await self.controller.send_command(
            self.id,
            LX16AWriteCommand.SERVO_LED_CTRL,
            [0]
        )

        self.led_powered = True

    async def set_led_error_triggers(
        self,
        over_temperature: bool,
        over_voltage: bool,
        rotor_locked: bool,
    ) -> None:
        combined = 4 * rotor_locked + 2 * over_voltage + over_temperature

        await self.controller.send_command(
            self.id,
            LX16AWriteCommand.SERVO_LED_ERROR,
            [combined]
        )

        self.led_error_triggers = over_temperature, over_voltage, rotor_locked

    async def get_id(self, poll_hardware: bool = False) -> int:
        if not poll_hardware:
            return self.id
        
        response = await self.controller.send_command(
            self.id,
            LX16AReadCommand.SERVO_ID,
            expect_response=True,
            response_len=1
        )

        return response[1]
    
    async def get_angle_offset(self, poll_hardware: bool = False) -> int:
        if not poll_hardware:
            return LX16AServo.position_to_angle(self.position_offset)
        
        response = await self.controller.send_command(
            self.id,
            LX16AReadCommand.SERVO_ANGLE_OFFSET,
            expect_response=True,
            response_len=1
        )

        offset = response[0]

        if offset > 125:
            offset -= 256
        
        return LX16AServo.position_to_angle(offset)
    
    async def get_angle_limits(self, poll_hardware: bool = False) -> tuple[float, float]:
        if not poll_hardware:
            return LX16AServo.position_to_angle(self.angle_limits[0]), LX16AServo.position_to_angle(self.angle_limits[1])
        
        response = await self.controller.send_command(
            self.id,
            LX16AReadCommand.SERVO_ANGLE_LIMIT,
            expect_response=True,
            response_len=4
        )

        lower_limit = LX16AServo.position_to_angle(LX16AProtocol.from_bytes(*response[:2]))
        upper_limit = LX16AServo.position_to_angle(LX16AProtocol.from_bytes(*response[2:]))
        
        return lower_limit, upper_limit
    
    async def get_vin_limits(self, poll_hardware: bool = False) -> tuple[int, int]:
        if not poll_hardware:
            return self.vin_limits
        
        response = await self.controller.send_command(
            self.id,
            LX16AReadCommand.SERVO_VIN_LIMIT,
            expect_response=True,
            response_len=4
        )

        lower_limit = LX16AServo.position_to_angle(LX16AProtocol.from_bytes(*response[:2]))
        upper_limit = LX16AServo.position_to_angle(LX16AProtocol.from_bytes(*response[2:]))
        
        return lower_limit, upper_limit
    
    async def get_temp_limit(self, poll_hardware: bool = False) -> int:
        if not poll_hardware:
            return self.temp_limit
        
        response = await self.controller.send_command(
            self.id,
            LX16AReadCommand.SERVO_TEMP_MAX_LIMIT,
            expect_response=True,
            response_len=1
        )

        return response[0]
    
    async def is_motor_mode(self, poll_hardware: bool = False) -> bool:
        if not poll_hardware:
            return self.motor_mode
        
        response = await self.controller.send_command(
            self.id,
            LX16AReadCommand.SERVO_OR_MOTOR_MODE,
            expect_response=True,
            response_len=4
        )

        return response[0] == 1
    
    async def get_motor_speed(self, poll_hardware: bool = False) -> int:
        if not self.motor_mode:
            raise LX16ALogicalError(f'servo {self.id}: not in motor mode', self.id)
        
        if not poll_hardware:
            return self.motor_speed

        response = await self.controller.send_command(
            self.id,
            LX16AReadCommand.SERVO_OR_MOTOR_MODE,
            expect_response=True,
            response_len=4
        )

        if response[0] == 1:
            speed = LX16AProtocol.from_bytes(*response[2:])
        else:
            speed = None

        if speed and speed > 32767:
            speed -= 65536

        return speed

    async def is_torque_enabled(self, poll_hardware: bool = False) -> bool:
        if not poll_hardware:
            return self.torque_enabled
        
        response = await self.controller.send_command(
            self.id,
            LX16AReadCommand.SERVO_LOAD_OR_UNLOAD,
            expect_response=True,
            response_len=1
        )

        return response[0] == 1
    
    async def is_led_power_on(self, poll_hardware: bool = False) -> bool:
        if not poll_hardware:
            return self.led_powered
        
        response = await self.controller.send_command(
            self.id,
            LX16AReadCommand.SERVO_LED_CTRL,
            expect_response=True,
            response_len=1
        )

        return response[0] == 0
    
    async def get_led_error_triggers(self, poll_hardware: bool = False) -> tuple[bool, bool, bool]:
        if not poll_hardware:
            return self.led_error_triggers
        
        response = await self.controller.send_command(
            self.id,
            LX16AReadCommand.SERVO_LED_ERROR,
            expect_response=True,
            response_len=1
        )

        over_temperature = response[0] & 1 != 0
        over_voltage = response[0] & 2 != 0
        rotor_locked = response[0] & 4 != 0

        return over_temperature, over_voltage, rotor_locked
    
    async def get_temp(self) -> int:
        response = await self.controller.send_command(
            self.id,
            LX16AReadCommand.SERVO_TEMP,
            expect_response=True,
            response_len=1
        )

        return response[0]
    
    async def get_vin(self) -> int:
        response = await self.controller.send_command(
            self.id,
            LX16AReadCommand.SERVO_VIN,
            expect_response=True,
            response_len=2
        )

        return LX16AProtocol.from_bytes(*response[:2])
    
    async def get_physical_angle(self) -> float:
        response = await self.controller.send_command(
            self.id,
            LX16AReadCommand.SERVO_POS,
            expect_response=True,
            response_len=2
        )
        
        position = LX16AProtocol.from_bytes(*response[:2])

        if position > 32767:
            position -= 65536

        return LX16AServo.position_to_angle(position)
    
    def get_commanded_angle(self) -> float:
        return LX16AServo.position_to_angle(self.commanded_position)
    
    def get_waiting_angle(self) -> float:
        if not self.waiting_for_move:
            raise LX16ALogicalError(f'servo {self.id}: not waiting for move')
        
        return LX16AServo.position_to_angle(self.waiting_position)
