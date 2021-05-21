import os
import sys
import shutil
import unittest
import warnings


class TestHighDConversion(unittest.TestCase):
    def setUp(self) -> None:
        super().setUp()
        test_path = os.path.dirname(os.path.realpath(__file__))
        self.input_path = os.path.join(test_path, "resources/highD")
        self.input_path_single = os.path.join(test_path, "resources/highD/single_process")
        self.output_path = os.path.join(test_path, "outputs/highD")
        self.xsd_path = os.path.join(test_path, "tools/XML_commonRoad_XSD_2020a.xsd")
        if os.path.exists(self.output_path):
            warnings.warn("Output path already exists, deleted")
            shutil.rmtree(self.output_path)

        sys.path.append(test_path)
        sys.path.append(os.path.join(test_path, ".."))

    def test_vanilla(self):
        from main import get_args
        from main import main as conversion
        from tools.validate_cr import validate_all

        args_str = (
            "highD "
            f"{self.input_path_single} "
            f"{self.output_path}"
        )

        args = get_args().parse_args(args_str.split(sep=" "))
        conversion(args)

        # validate
        validate_all(self.output_path, self.xsd_path)

    def test_multiprocessing(self):
        from main import get_args
        from main import main as conversion
        from tools.validate_cr import validate_all

        args_str = (
            "highD "
            f"{self.input_path} "
            f"{self.output_path} "
            f"--num_processes 2"
        )
        args = get_args().parse_args(args_str.split(sep=" "))
        conversion(args)

        # validate
        validate_all(self.output_path, self.xsd_path)

    # def test_validate(self):
    #     from tools.validate_cr import validate_all
    #
    #     # validate
    #     validate_all(self.output_path, self.xsd_path)

    def test_cooperative(self):
        from main import get_args
        from main import main as conversion
        from tools.validate_cr import validate_all

        args_str = (
            "highD "
            f"{self.input_path_single} "
            f"{self.output_path} "
            f"--num_planning_problem 2"
        )

        args = get_args().parse_args(args_str.split(sep=" "))
        conversion(args)

        # validate
        validate_all(self.output_path, self.xsd_path)

    def test_others(self):
        from main import get_args
        from main import main as conversion
        from tools.validate_cr import validate_all

        args_str = (
            "highD "
            f"{self.input_path_single} "
            f"{self.output_path} "
            f"--keep_ego "
            f"--obstacle_start_at_zero "
            f"--downsample 5 "
            f"--num_vertices 50"
        )

        args = get_args().parse_args(args_str.split(sep=" "))
        conversion(args)

        # validate
        validate_all(self.output_path, self.xsd_path)

if __name__ == '__main__':
    unittest.main()