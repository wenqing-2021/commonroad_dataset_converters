import unittest

from tests.tools.base_suite import BaseSuite


class TestRounDConversion(BaseSuite.ConverterSuite):
    @classmethod
    def setUpClass(cls) -> None:
        cls.dataset = "round"


if __name__ == "__main__":
    unittest.main()
