import os
import shutil
import warnings
from pathlib import Path
from unittest import TestCase, skip

from typer.testing import CliRunner

from commonroad_dataset_converter.main import cli

from .validate_cr import validate_all


def remove_output_dir(output_path):
    if os.path.exists(output_path):
        warnings.warn("Output path already exists, deleted")
        shutil.rmtree(output_path)


class BaseSuite:
    class ConverterSuite(TestCase):
        def setUp(self):
            test_path = Path(__file__).parent.parent
            self.input_path = os.path.join(test_path, f"resources/{self.dataset}")
            self.input_path_single = (
                test_path / f"resources/{self.dataset}/single_process"
            )
            self.output_path = test_path / f"outputs/{self.dataset}"
            self.xsd_path = os.path.join(
                test_path, "tools/XML_commonRoad_XSD_2020a.xsd"
            )

        def test_vanilla(self):
            remove_output_dir(self.output_path)
            self.output_path.mkdir(exist_ok=True, parents=True)
            runner = CliRunner()
            result = runner.invoke(
                cli,
                [
                    "--max-scenarios",
                    "10",
                    str(self.input_path),
                    str(self.output_path),
                    self.dataset,
                ],
                catch_exceptions=False,
            )
            self.assertEqual(0, result.exit_code, result.stdout)
            # validate
            validate_all(self.output_path, self.xsd_path)

        def test_multiprocessing(self):
            remove_output_dir(self.output_path)
            self.output_path.mkdir(exist_ok=True, parents=True)
            runner = CliRunner()
            result = runner.invoke(
                cli,
                [
                    "--num-processes",
                    "4",
                    "--max-scenarios",
                    "10",
                    str(self.input_path),
                    str(self.output_path),
                    self.dataset,
                ],
                catch_exceptions=False,
            )
            self.assertEqual(0, result.exit_code, result.stdout)

            # validate
            validate_all(self.output_path, self.xsd_path)

        def test_cooperative(self):
            remove_output_dir(self.output_path)
            self.output_path.mkdir(exist_ok=True, parents=True)
            runner = CliRunner()
            result = runner.invoke(
                cli,
                [
                    "--num-planning-problems",
                    "2",
                    "--max-scenarios",
                    "10",
                    str(self.input_path),
                    str(self.output_path),
                    self.dataset,
                ],
                catch_exceptions=False,
            )
            self.assertEqual(0, result.exit_code, result.stdout)

            # validate
            validate_all(self.output_path, self.xsd_path)

        def test_others(self):
            remove_output_dir(self.output_path)
            self.output_path.mkdir(exist_ok=True, parents=True)
            runner = CliRunner()
            result = runner.invoke(
                cli,
                [
                    "--downsample",
                    "5",
                    "--max-scenarios",
                    "10",
                    str(self.input_path),
                    str(self.output_path),
                    self.dataset,
                ],
                catch_exceptions=False,
            )
            self.assertEqual(0, result.exit_code, result.stdout)

            # validate
            validate_all(self.output_path, self.xsd_path)
