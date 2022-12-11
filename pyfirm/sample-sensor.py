from machine import UART, Pin
from time import sleep
import uasyncio as asyncio


class GSM():
    def __init__(self):
        self.uart = UART(1, 115200, tx=Pin(8), rx=Pin(9))
        self.swriter = asyncio.StreamWriter(self.uart, {})
        self.sreader = asyncio.StreamReader(self.uart)

    async def sender(self, msg):
        await self.swriter.awrite(msg)

    async def flush(self):
        res = ""
        try:
            while "OK" not in res:
                res = await self.sreader.readline()
                print('Recieved:', res)
        except asyncio.TimeoutError:
            print("Timeout: ERROR in response")
        finally:
            print("Exiting")

    async def receiver(self):
        res = ""
        ret = []
        try:
            while "OK" not in res:
                res = await self.sreader.readline()
                ret.append(res)
        except asyncio.TimeoutError:
            print("Timeout: Error in response")
            return []
        finally:
            print("Exiting")
            return ret


    def send_and_flush(self, msg):
        asyncio.run(self.sender(msg))
        asyncio.run(self.flush())

    def send_and_rec(self, msg):
        asyncio.run(self.sender(msg))
        ret = asyncio.run(self.receiver())
        return ret

    def auth_sender(self, msg_list):
        try:
            return self.number == res[1].decode("utf-8").replace('"','').split(",")[2]
        except:
            return -1


gsm = GSM()
gsm.send_and_flush('AT+CPMS="SM"\r\n')

gsm.send_and_flush('AT+CMGF=1\r\n')

res = gsm.send_and_rec('AT+CMGL="ALL"\r\n')
print(res)