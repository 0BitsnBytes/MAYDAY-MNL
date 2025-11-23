import asyncio
from mpu6050 import mpu6050

class AsyncMPU:
    def __init__(self, address=0x68):
        self.mpu = mpu6050(address)
        self.latest = None

    def read_sensor(self):
        """
        Returns:
            {
                'accel': {x,y,z},
                'gyro': {x,y,z},
                'temp': float
            }
        """
        try:
            accel = self.mpu.get_accel_data()
            gyro = self.mpu.get_gyro_data()
            temp = self.mpu.get_temp()
            return {
                'accel' : accel,
                'gyro' : gyro,
                'temp' : temp
            }
        except Exception:
            return None

    async def read_loop(self):
        """Async background reader."""
        while True:
            data = self.read_sensor()
            if data:
                self.latest = data
            await asyncio.sleep(0.5)  # 100 Hz

    def get(self):
        """Return latest sample or None."""
        return self.latest
