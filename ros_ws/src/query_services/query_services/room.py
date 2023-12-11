from typing import Tuple


class Room:
    """
    Room is a helper class to handle room objects to and fro the DB
    """

    def __init__(self, id: int, name: str, location: Tuple[float, float]):
        self.id: int = id
        self.name = name
        self.location = location
