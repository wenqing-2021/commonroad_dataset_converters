import unittest

from tests.tools.base_suite import BaseSuite


class TestExiDConversion(BaseSuite.ConverterSuite):
    @classmethod
    def setUpClass(cls) -> None:
        cls.dataset = "exid"


if __name__ == "__main__":
    unittest.main()
