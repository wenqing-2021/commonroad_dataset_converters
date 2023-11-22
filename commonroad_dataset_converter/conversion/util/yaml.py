from typing import Dict, Union

from ruamel.yaml import YAML


def load_yaml(file_name: str) -> Union[Dict, None]:
    """
    Loads configuration setup from a yaml file

    :param file_name: name of the yaml file
    """
    with open(file_name, "r") as stream:
        config = YAML().load(stream)
    return config
