import unittest

from tests.tools.base_suite import BaseSuite


class TestHighDConversion(BaseSuite.ConverterSuite):
    @classmethod
    def setUpClass(cls) -> None:
        cls.dataset = "highd"


if __name__ == "__main__":
    unittest.main()
