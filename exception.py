class OperationalException(Exception):
    """Represents the exceptions occured while operating vessel commands"""

    def __init__(self, message: str):
        self.message: str = message
