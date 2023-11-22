import unittest

from tests.tools.base_suite import BaseSuite


class TestInteractionConversion(BaseSuite.ConverterSuite):
    @classmethod
    def setUpClass(cls) -> None:
        cls.dataset = "interaction"


if __name__ == "__main__":
    unittest.main()
