"""Example module
"""
from typing import List

def example_function() -> None:
    """Example function that prints out a message
    """
    print("This is a function that does something")

def example_returning_function() -> str:
    """Example function that returns a value

    Returns:
        str: Example return value
    """
    return "This function returned a string"

def example_complex_function(arg_a: int, arg_b: List[str]) -> str:
    """Example function that does something complex with a function

    Args:
        a (int): Number to insert between strings
        b (List[str]): Strings to combine

    Returns:
        str: String in b concatenated using the string representation of a
    """
    return f"{arg_a}".join(arg_b)

def example_entry_point() -> None:
    """Example entry point that exersizes all of the module's functions
    """
    example_function()
    print(example_returning_function())
    print(f"This is a complex function: {example_complex_function(3, ['asdf', 'foo', 'bar'])}")

if __name__ == "__main__":
    example_entry_point()
