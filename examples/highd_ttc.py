from pathlib import Path

import pandas as pd
from pydantic.dataclasses import dataclass

from commonroad_dataset_converter.conversion.tabular.interface import Window
from commonroad_dataset_converter.datasets.highD import HighdConverterFactory
from commonroad_dataset_converter.datasets.highD.implementation import (
    HighdObstacleWindowGenerator,
)


# Create a class that only provides index values of vehicles with a
# minimum time-to-collision of less than 2s.
@dataclass
class HighdTtcVehicleWindowGenerator(HighdObstacleWindowGenerator):
    min_ttc: float = 2.0

    def _get_index(self, recording: Window) -> pd.Index:
        index = super()._get_index(recording)
        # Get metadata of vehicles selected by parent class.
        meta = recording.vehicle_meta.loc[index]
        # Vehicles without leading vehicle have a minimum TTC of -1.0.
        index = meta[(meta["minTTC"] < self.min_ttc) & (meta["minTTC"] >= 0.0)].index
        return index


# Customize the factory to use the newly created class.
class HighdTtcFactory(HighdConverterFactory):
    min_ttc: float = 2.0

    def build_obstacle_window_generator(self):
        return HighdTtcVehicleWindowGenerator(self.lane_change, self.min_ttc)


if __name__ == "__main__":
    input_dir = Path("../tests/resources/highd")
    output_dir = Path("/tmp/highd_ttc_scenarios")
    output_dir.mkdir(exist_ok=True)

    # Create the factory
    factory = HighdTtcFactory(
        input_dir=input_dir, output_dir=output_dir, num_time_steps=None
    )
    # Build the runner
    runner = factory.build_runner()
    # Run it
    runner.run()
