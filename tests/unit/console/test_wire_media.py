import asyncio

from console import wire


class _Transport:
    def __init__(self, buffered):
        self.buffered = buffered

    def get_write_buffer_size(self):
        return self.buffered

    def abort(self):
        pass


class _Ws:
    def __init__(self, buffered):
        self.transport = _Transport(buffered)

    async def send(self, payload):
        return None


class Fan(wire.MediaFanoutMixin):
    def __init__(self):
        self.media = {}
        self.media_drops = 0
        self.media_closed = 0


def test_send_media_says_whether_the_frame_went_out():
    async def body():
        fan = Fan()
        busy = {"ws": _Ws(wire.MEDIA_SKIP_BYTES + 1), "rates": {}, "last": {}}
        fan.media["a"] = busy
        assert await fan._send_media("a", busy, "grid", b"x") is False
        assert fan.media_drops == 1
        free = {"ws": _Ws(0), "rates": {}, "last": {}}
        fan.media["b"] = free
        assert await fan._send_media("b", free, "grid", b"x") is True
        await asyncio.sleep(0)

    asyncio.run(body())
