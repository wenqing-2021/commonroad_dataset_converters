# Script for converting interaction dataset maps from Lanelet2 to CommonRoad
# Requires Scenario Designer version >=0.8.0
from pathlib import Path
import numpy as np

from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.common.file_writer import CommonRoadFileWriter, OverwriteExistingFile
from commonroad.planning.planning_problem import PlanningProblemSet

from crdesigner.config.gui_config import gui_config
from crdesigner.config.lanelet2_config import lanelet2_config
from crdesigner.map_conversion.map_conversion_interface import lanelet_to_commonroad

from commonroad_dataset_converter.conversion.util.yaml import load_yaml


lanelet2_config.proj_string = gui_config.utm_default

root_dir = Path(__file__).parent.parent
interaction_path = Path("/path/to/TUM/INTERACTION/")
cr_path = root_dir / Path("commonroad_dataset_converter/datasets/INTERACTION/maps")

paths = (list((interaction_path/Path("INTERACTION-Dataset-DR-v1_0/maps")).glob("*osm")) +
         list((interaction_path/Path("INTERACTION-Dataset-TC-v1_0/maps")).glob("*osm")))
cr_files = list(cr_path.glob("*xml"))

config = load_yaml(str(root_dir / "commonroad_dataset_converter/datasets/INTERACTION/config.yaml"))

for sc_path in cr_files:
    sc, pp = CommonRoadFileReader(sc_path).open()
    for path in paths:
        if sc.scenario_id.country_id in str(path) and sc.scenario_id.map_name in str(path):
            offset = config["offsets"][f"{sc.scenario_id.country_id}_{sc.scenario_id.map_name}-{sc.scenario_id.map_id}"]
            scenario = lanelet_to_commonroad(str(path), lanelet2_conf=lanelet2_config)
            scenario.scenario_id = sc.scenario_id
            scenario.lanelet_network.translate_rotate(
                    np.array([-offset["x_offset_lanelets"], -offset["y_offset_lanelets"]]), 0)
            writer = CommonRoadFileWriter(
                scenario=scenario,
                planning_problem_set=PlanningProblemSet(),
                author="Sebastian Maierhofer",
                affiliation="Technical University of Munich",
                source="CommonRoad Scenario Designer",
                tags=sc.tags,
            )
            writer.write_to_file(str(sc_path), OverwriteExistingFile.ALWAYS)
