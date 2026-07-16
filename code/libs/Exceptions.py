
class WrongPortError(Exception):
    __slots__ = ("port", "expected", "found", "_message")

    def __init__(self, port=None, expected=None, found=None, message=None):
        if message is None:
            if port is None and expected is None:
                message = "Wrong port"
            elif expected is None:
                message = "Wrong port: {}".format(port)
            elif port is None:
                message = "Wrong port, expected: {}".format(expected)
            elif found is None:
                message = "Wrong port: {}; expected {}".format(port, expected)
            else:
                message = "Wrong port: {}; expected {}, got {}".format(port, expected, found)

        self.port = port
        self.expected = expected
        self.found = found
        self._message = message

    def __str__(self):
        return self._message

