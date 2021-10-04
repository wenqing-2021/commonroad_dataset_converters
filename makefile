# Installs the converter locally in editable mode
install:
	pip install -e .

# Builds the dataset converter sdist and wheel
build:
	python -m build

# Clean build artifacts
clean:
	rm -rf build dist commonroad_dataset_converter.egg-info
