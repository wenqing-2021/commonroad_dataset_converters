import os
import sys
import shutil
import unittest
import warnings


def remove_output_dir(output_path):
    if os.path.exists(output_path):
        warnings.warn("Output path already exists, deleted")
        shutil.rmtree(output_path)


class TestInDConversion(unittest.TestCase):
    def setUp(self) -> None:
        super().setUp()
        test_path = os.path.dirname(os.path.realpath(__file__))
        self.dataset = "ind"
        self.input_path = os.path.join(test_path, f"resources/{self.dataset}")
        self.input_path_single = os.path.join(test_path, f"resources/{self.dataset}/single_process")
        self.output_path = os.path.join(test_path, f"outputs/{self.dataset}")
        self.xsd_path = os.path.join(test_path, "tools/XML_commonRoad_XSD_2020a.xsd")

        sys.path.append(test_path)
        sys.path.append(os.path.join(test_path, ".."))

    def test_vanilla(self):
        from tools.validate_cr import validate_all

        remove_output_dir(self.output_path)

        if os.system(f"crconvert {self.dataset} "
                     f"{self.input_path_single} "
                     f"{self.output_path}") != 0: # --num-time-steps 1000
            raise Exception

        # validate
        validate_all(self.output_path, self.xsd_path)

    def test_multiprocessing(self):
        from tools.validate_cr import validate_all
        remove_output_dir(self.output_path)
        if os.system(f"crconvert {self.dataset} "
                     f"{self.input_path} "
                     f"{self.output_path} "
                     f"--num-processes 4") != 0:
            raise Exception

        # validate
        validate_all(self.output_path, self.xsd_path)

    def test_cooperative(self):
        from tools.validate_cr import validate_all

        remove_output_dir(self.output_path)
        if os.system(f"crconvert {self.dataset} "
                     f"{self.input_path_single} "
                     f"{self.output_path} "
                     f"--num-planning-problems 2") != 0:
            raise Exception

        # validate
        validate_all(self.output_path, self.xsd_path)

    def test_others(self):
        from tools.validate_cr import validate_all

        remove_output_dir(self.output_path)
        if os.system(f"crconvert {self.dataset} "
                     f"{self.input_path_single} "
                     f"{self.output_path} "
                     f"--keep-ego "
                     f"--obstacle-start-at-zero "
                     f"--all-vehicles") != 0:
            raise Exception

        # validate
        validate_all(self.output_path, self.xsd_path)


if __name__ == '__main__':
    unittest.main()
