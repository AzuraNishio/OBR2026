from multiprocessing.connection import Client

class command:
    def __init__(self, name, *args):
        self.name = name
        self.args = args


class Bridge:
    def __init__(self):
        self.connection = Client(
            ("localhost", 6000),
            authkey=b"robot"
        )

        print("Connected to simulator!")

    async def send_command_and_wait(self, cmd, *args):
        self.connection.send(command(cmd, *args))
        while not self.connection.poll():
            pass
        return self.connection.recv()

    def send_command(self, cmd, *args):
        self.connection.send(command(cmd, *args))
        while not self.connection.poll():
            pass
        return self.connection.recv()

