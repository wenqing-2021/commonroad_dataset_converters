from crdesigner.verification_repairing.map_verification_repairing import verify_and_repair_scenario
from crdesigner.verification_repairing.verification.formula_ids import FormulaID, LaneletFormulaID
from crdesigner.verification_repairing.config import MapVerParams
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.common.file_writer import CommonRoadFileWriter, OverwriteExistingFile
from commonroad.visualization.mp_renderer import MPRenderer
from typing import List
from multiprocessing import Pool
from functools import partial
import logging
import traceback
import glob
from pathlib import Path


date_strftime_format = "%d-%b-%y %H:%M:%S"
message_format = "%(asctime)s - %(levelname)s - %(message)s"
logging.basicConfig(encoding='utf-8', level=logging.INFO, format=message_format, datefmt=date_strftime_format)


num_cores = 1
formulas = []


def eval_maps(config: MapVerParams, path: str):
    if "datasets/sind" in path:  # traffic lights are added during dataset conversion itself
        config.verification.excluded_formulas = [LaneletFormulaID.STOP_LINE_REFERENCES]
    sc, pps = CommonRoadFileReader(path).open()
    rnd = MPRenderer()
    sc.lanelet_network.draw(rnd)
    rnd.render(filename=f"{Path(__file__).parent.parent / Path('figures')}/{sc.scenario_id}_org")
    try:
        sc, valid = verify_and_repair_scenario(sc, config)
        rnd = MPRenderer()
        sc.lanelet_network.draw(rnd)
        rnd.render(filename=f"{Path(__file__).parent.parent / Path('figures')}/{sc.scenario_id}_new")
        if not valid:
            CommonRoadFileWriter(sc, pps).write_to_file(path, OverwriteExistingFile.ALWAYS)
        else:
            logging.info("eval map {} is valid".format(sc.scenario_id))
    except Exception:
        logging.error("eval map {}, error: {}".format(sc.scenario_id, traceback.format_exc()))


def prepare_and_execute_eval(scenario_base_dir: Path, selected_formulas: List[FormulaID]):
    relevant_scenario_paths = glob.glob(f"{scenario_base_dir}/**/*.xml", recursive=True)
    path = Path(__file__).parent.parent / Path("figures")
    if not path.exists():
        path.mkdir()
    config = MapVerParams()
    config.verification.formulas = selected_formulas
    config.evaluation.overwrite_scenario = True
    func = partial(eval_maps, config)
    with Pool(processes=num_cores) as pool:
        pool.map(func, relevant_scenario_paths)


if __name__ == '__main__':
    prepare_and_execute_eval(Path(__file__).parent.parent / Path("commonroad_dataset_converter/datasets"), formulas)
