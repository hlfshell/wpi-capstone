from __future__ import annotations

from typing import List, Optional, Tuple


class Item:
    """
    Item is an object within the gazebo simulator that litterbug
    manages. This object tracks state and basic helper functions
    for the item.

    The type parameter is an extra for additional implementing
    classes to use for additional interaction wrapping.
    """

    def __init__(
        self,
        name: str,
        model: str,
        placement: Tuple[float, float, float],
        orientation: Tuple[float, float, float, float] = (0.0, 0.0, 0.0, 0.0),
    ):
        self.name = name
        self.model = model
        self.origin = placement
        self.orientation = orientation

    def __str__(self):
        return f"{self.name}: {self.model} @ {self.origin}"

    @staticmethod
    def from_csv(filepath: str) -> List[Item]:
        items = []
        with open(filepath) as f:
            for line in f.readlines():
                name, model, x, y, z, roll, pitch, yaw, id = line.split(",")
                items.append(
                    Item(
                        name=name,
                        model=model,
                        placement=(float(x), float(y), float(z)),
                        orientation=(float(roll), float(pitch), float(yaw)),
                        id=id.strip(),
                    )
                )
        return items

    @staticmethod
    def to_csv(items: List[Item], filepath: str):
        with open(filepath, "w") as f:
            for item in items:
                f.write(
                    f"{item.name},"
                    + f"{item.model},"
                    + f"{item.origin[0]},"
                    + f"{item.origin[1]},"
                    + f"{item.origin[2]},"
                    + f"{item.orientation[0]},"
                    + f"{item.orientation[1]},"
                    + f"{item.orientation[2]},"
                    + f"{item.id}\n"
                )
