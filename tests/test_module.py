"""Example test module
"""
from example_package import example_module

def test_example_function():
    """Tests that the example function completes successfully
    """
    example_module.example_function()
    assert True

def test_example_return():
    """Tests that the function returns something
    """
    retval = example_module.example_returning_function()
    assert retval is not None
