'''RCT Options
'''
from __future__ import annotations
import logging
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional, Type

import yaml
from RCTComms.options import Options, validate_option


@dataclass
class ParamEntry:
    """Parameter Validation dataclass
    """
    type_list: List[Type]
    validation_fn: Optional[Callable[[Any], bool]] = None
    transform_fn: Optional[Callable[[Any], Any]] = None

class RCTOpts:
    """Global Options
    """

    param_fn_table: Dict[str, ParamEntry] = {}

    def __init__(self, config_path: Path) -> None:
        self.__log = logging.getLogger('opts')
        self._config_file = config_path
        self._params: Dict[Options, Any] = {}
        self.load_params()

    def load_params(self) -> None:
        """Loads the parameters from disk
        """
        with open(self._config_file, 'r', encoding='utf-8') as handle:
            config: Dict[str, Any] = yaml.safe_load(handle)
        for option in config:
            self._params[Options(option)] = config[option]
            self.__log.info('Discovered {} as {}', option, str(config[option]))

    _option_map: Dict[Path, RCTOpts] = {}
    @classmethod
    def get_instance(cls, path: Path) -> RCTOpts:
        """Retrieves the global intance for the given path

        Args:
            path (Path): Path reference

        Returns:
            RCTOpts: Global options instance
        """
        if path not in cls._option_map:
            cls._option_map[path] = RCTOpts(config_path=path)
        return cls._option_map[path]

    def __getitem__(self, key):
        if not isinstance(key, Options):
            raise TypeError()
        return self._params[key]

    def __setitem__(self, key, value):
        if not isinstance(key, Options):
            raise TypeError()
        self._params[key] = value

    def write_options(self):
        """Writes the current options to disk
        """
        backups = list(self._config_file.parent.glob('*.bak'))
        if len(backups) > 0:
            backup_numbers = [path.stem.lstrip('rct_config')
                              for path in backups]
            backup_numbers = [int(number)
                              for number in backup_numbers
                              if number != '']
            next_number = max(backup_numbers) + 1
        else:
            next_number = 1

        new_name = f'{self._config_file.stem}{next_number}.bak'
        new_path = self._config_file.parent.joinpath(new_name)
        self._config_file.rename(new_path)

        with open(self._config_file, 'w', encoding='ascii') as var_file:
            data = {key.value:value for key, value in self._params.items()}
            yaml.dump(data, var_file)

    def get_all_options(self) -> Dict[Options, Any]:
        """Retrieves all options

        Returns:
            Dict[str, Any]: All option values
        """
        return self._params
    
    def set_options(self, options: Dict[Options, Any]):
        """Updates the current options from the specified dictionary

        Args:
            options (Dict[Options, Any]): Options to update
        """
        for key, value in options.items():

            value = validate_option(key, value)

            # update
            self._params[key] = value
