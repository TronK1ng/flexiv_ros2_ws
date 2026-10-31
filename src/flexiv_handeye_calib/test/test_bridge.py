"""Bridge so that `colcon test` (which runs unittest) executes the pytest suite.

The real tests live in the other test_*.py files and use pytest features.
This file is ignored by pytest itself through conftest.py, so there is no
recursion.
"""

import os
import unittest

os.environ.setdefault("PYTEST_DISABLE_PLUGIN_AUTOLOAD", "1")

import pytest  # noqa: E402  (must come after the env var)

HERE = os.path.dirname(os.path.abspath(__file__))


class TestPytestSuite(unittest.TestCase):
    def test_suite(self):
        code = pytest.main(
            [HERE, "-q", "--ignore", os.path.join(HERE, "test_bridge.py")]
        )
        self.assertEqual(int(code), 0, "pytest suite failed")


if __name__ == "__main__":
    unittest.main()
