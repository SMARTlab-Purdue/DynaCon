import ast

def extract_function_text(filename):
    """
    Extract function text from a python file and return in a dictionary format.

    Parameters:
    - filename (str): The path to the Python file.

    Returns:
    - dict: A dictionary where keys are function names and values are the text of the function.
    """

    with open(filename, 'r') as f:
        node = ast.parse(f.read())

    functions = [n for n in node.body if isinstance(n, ast.FunctionDef)]
    function_texts = {}

    for function in functions:
        start_line = function.lineno - 1
        end_line = function.end_lineno if hasattr(function, 'end_lineno') else start_line

        with open(filename, 'r') as f:
            lines = f.readlines()

        func_text = ''.join(lines[start_line:end_line+1])
        function_texts[function.name] = func_text

    return function_texts

if __name__ == '__main__':
    # For testing purposes
    functions = extract_function_text('sample.py')  # Replace with your Python file
    print(functions)
