class py:
    """Dummy-stub for gclib.py."""

    def __init__(self):
        """Constructor for the Connection class. Initializes gclib's handle and read buffer."""

    def GCommand(self, command):
        """ """
        print('Sending: '+command)

    def GOpen(self, address):
        print('Connecting: ' + address)

    def GClose(self):
        print('Closed')

    def GMotionComplete(self, command):
        """ """
        print('Sending: AM '+command)

