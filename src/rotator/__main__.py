import asyncio

from .platform import Platform
from .rotctl import RotctlTCPServerProtocol
from .math_utils import lerp

async def calibrate(serial_port, baud_rate, **kwargs):
    async with Platform(serial_port, baud_rate) as platform:
        # reset the servo offsets to 'factory' settings
        # platform.calibration.offsets = [120.0] * 4

        offsets = []

        for i, arm in enumerate('NESW'):
            # move the servo into fully extended position
            await platform.move((i * 90 + 180) % 360, 50)

            # disable the servo to allow adjustment
            await asyncio.sleep(1)
            await platform.disengage(i)

            # prompt the user to make the manual adjustment
            input(f'adjust the {arm} arm to be fully extended and press ENTER...')
            
            # re-enable the servo
            await platform.engage(i)

            offset = await platform.get_servo_angle(i) - 181.02
            await platform.adjust_servo_offset(i, offset)

            offsets.append(offset)

            # capture the offset and set it
            # offsets = platform.config.offsets
            # offsets[i] = platform.get_servo_angle(i) - 120 # TODO check if + or - 120
            # platform.config.offsets = offsets

        # reset the rotator to default position

        print(offsets)

        await platform.move(0, 90)

async def rotctl(serial_port, baud_rate, host, port, **kwargs):
    async with Platform(serial_port, baud_rate) as platform:

        loop = asyncio.get_running_loop()

        server = await loop.create_server(
            lambda: RotctlTCPServerProtocol(platform),
            host=host,
            port=port,
        )

        addr = server.sockets[0].getsockname()
        print(f'rotctl running on {addr}')

        async with server:
            await server.serve_forever()

async def demo(serial_port, baud_rate, **kwargs):
    async with Platform(serial_port, baud_rate) as platform:

        await platform.move(0, 50)

        for elevation in range(50, 91, 5):
            for azimuth in range (0, 360, 5):
                await platform.move(azimuth, elevation)

async def position(serial_port, baud_rate, azimuth, elevation, **kwargs):
    async with Platform(serial_port, baud_rate) as platform:

        await platform.move(azimuth, elevation)


if __name__ == '__main__':

    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument('--serial-port', default='/dev/serial0')
    parser.add_argument('--baud-rate', type=int, default=115200)

    subparsers = parser.add_subparsers()

    calibrate_parser = subparsers.add_parser('calibrate')
    calibrate_parser.set_defaults(fn=calibrate)

    rotctl_parser = subparsers.add_parser('rotctl')
    rotctl_parser.add_argument('--host', default='0.0.0.0')
    rotctl_parser.add_argument('--port', type=int, default=4533)
    rotctl_parser.set_defaults(fn=rotctl)

    demo_parser = subparsers.add_parser('demo')
    demo_parser.set_defaults(fn=demo)

    position_parser = subparsers.add_parser('position')
    position_parser.add_argument('azimuth', type=float)
    position_parser.add_argument('elevation', type=float)
    position_parser.set_defaults(fn=position)

    
    args = parser.parse_args()

    asyncio.run(args.fn(**vars(args)))
    