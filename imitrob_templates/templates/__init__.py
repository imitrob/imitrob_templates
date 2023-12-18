import importlib.util
import os
import warnings

from .BaseTask import BaseTask, TaskExecutionMode

warnings.formatwarning = lambda message, category, filename, lineno, line=None: \
    f"{category.__name__} [{filename}]: {message}\n"

# Get the current directory
current_dir = os.path.dirname(os.path.abspath(__file__))

# Get a list of all .py files in the current directory
py_files = [f for f in os.listdir(current_dir) if f.endswith('.py')]

# Initialize a list to store the imported classes
imported_classes = []

# Loop through each .py file
for py_file in py_files:
    if py_file in ['BaseTask.py', '__init__.py']:
        continue

    # Remove the .py extension from the file name
    module_name = os.path.splitext(py_file)[0]

    # Import the module
    spec = importlib.util.spec_from_file_location(module_name, os.path.join(current_dir, py_file))
    if spec is None:
        warnings.warn(f"Module '{module_name}' in file '{py_file}' could not be imported. Skipping import.")
        continue

    if spec.loader is None:
        warnings.warn(f"Module '{module_name}' in file '{py_file}' has no loader. Skipping import.")
        continue

    module = importlib.util.module_from_spec(spec)

    try:
        spec.loader.exec_module(module)

        # Get the class name (assumed to be the same as the file name)
        class_name = module_name

        # Get the class from the module
        imported_class = getattr(module, class_name)

        # Check if the imported class is a subclass of BaseTask
        if not issubclass(imported_class, BaseTask):
            # Skip importing this class and issue a warning
            warnings.warn(f"Class '{class_name}' in file '{py_file}' is not a subclass of BaseTask. Skipping import.")
            continue

        # Add the imported class to the list
        imported_classes.append(imported_class)

    except AttributeError:
        # Class not found in the file, issue a warning
        warnings.warn(f"No class found matching file name '{module_name}' in file '{py_file}'. Skipping import.")

# Print the imported classes
print(f"Imported task: [{', '.join([str(imported_class.__name__) for imported_class in imported_classes])}]")
