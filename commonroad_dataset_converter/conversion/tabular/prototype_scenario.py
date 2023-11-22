from dataclasses import dataclass
from typing import Generic, TypeVar

from commonroad.scenario.scenario import Scenario, ScenarioID

from commonroad_dataset_converter.conversion.tabular.interface import (
    IMetaScenarioCreator,
    Window,
)

WindowMetaT = TypeVar("WindowMetaT")


class ScenarioIDCreator:
    """Create the scenario id for a window.

    Resulting scenario IDs have the schema
    {meta scenario country}_{meta scenario map name}-{meta scenario map id}_{window minimum time stamp}_T-
    {window maximum time stamp}
    """

    def __call__(self, window_job: Window, meta_scenario: Scenario) -> ScenarioID:
        meta_scenario_id = meta_scenario.scenario_id
        scenario_id = ScenarioID(
            country_id=meta_scenario_id.country_id,
            map_name=meta_scenario_id.map_name,
            map_id=meta_scenario_id.map_id,
            obstacle_behavior="T",
            configuration_id=int(
                str(meta_scenario_id.configuration_id or "")
                + str(window_job.vehicle_states.index.get_level_values(-1).min())
            ),
            prediction_id=window_job.vehicle_states.index.get_level_values(-1).max(),
        )
        return scenario_id


@dataclass
class ScenarioPrototypeCreator(Generic[WindowMetaT]):
    """Create the actual scenario prototype to which the dynamic obstacles are added.

    The prototype is created form the meta scenario. Country, map name, map id, author, source, affiliation,
    and location fields are copied from the meta scenario.
    Note, that the difference to the MetaScenarioGenerator is the dynamic scenario id of the
    prototype scenario. The meta scenario has the fixed scenario id of the map.
    """

    scenario_id_generator: ScenarioIDCreator
    meta_scenario_creator: IMetaScenarioCreator[WindowMetaT]

    def __call__(self, window_job: Window, window_meta: WindowMetaT) -> Scenario:
        meta_scenario = self.meta_scenario_creator(window_job, window_meta)
        scenario_id = self.scenario_id_generator(window_job, meta_scenario)
        scn = Scenario(
            window_job.dt,
            scenario_id,
            meta_scenario.author,
            meta_scenario.tags,
            meta_scenario.affiliation,
            meta_scenario.source,
            meta_scenario.location,
        )
        scn.add_objects(meta_scenario.lanelet_network)
        return scn
