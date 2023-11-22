import unittest

from tests.tools.base_suite import BaseSuite


class TestInDConversion(BaseSuite.ConverterSuite):
    @classmethod
    def setUpClass(cls) -> None:
        cls.dataset = "ind"


if __name__ == "__main__":
    unittest.main()
