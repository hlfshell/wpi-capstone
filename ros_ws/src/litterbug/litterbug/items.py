from __future__ import annotations

import math
from typing import List, Optional, Tuple

import numpy as np


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
        label: str,
        model: str,
        origin: Tuple[float, float, float],
        orientation: Tuple[float, float, float, float] = (0.0, 0.0, 0.0, 0.0),
        spawn: bool = False,
    ):
        self.name = name
        self.label = label
        self.model = model
        self.origin = origin
        self.orientation = orientation
        self.spawn = spawn

    def __str__(self):
        return f"{self.name}: {self.label} @ {self.origin}"

    @staticmethod
    def from_csv(
        filepath: str,
    ) -> List[Item]:
        items = []
        with open(filepath) as f:
            for line in f.readlines():
                name, label, model, x, y, z, qx, qy, qz, qw, spawn = line.split(",")

                items.append(
                    Item(
                        name=name,
                        label=label,
                        model=model,
                        origin=(float(x), float(y), float(z)),
                        orientation=(float(qx), float(qy), float(qz), float(qw)),
                        spawn=spawn.strip() == "TRUE",
                    )
                )
        return items

    @staticmethod
    def to_csv(items: List[Item], filepath: str):
        with open(filepath, "w") as f:
            for item in items:
                f.write(
                    f"{item.name},"
                    + f"{item.label},"
                    + f"{item.model},"
                    + f"{item.origin[0]},"
                    + f"{item.origin[1]},"
                    + f"{item.origin[2]},"
                    + f"{item.orientation[0]},"
                    + f"{item.orientation[1]},"
                    + f"{item.orientation[2]},"
                    + f"{item.orientation[3]},"
                    + f"{'TRUE' if item.spawn else 'FALSE'}\n"
                    + "\n"
                )
