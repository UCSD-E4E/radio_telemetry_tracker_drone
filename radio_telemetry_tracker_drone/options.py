'''RCT Options
'''
from __future__ import annotations
import logging
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional, Type

import yaml
from RCTComms.options import Options


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
        if path not in cls._option_map:
            cls._option_map[path] = RCTOpts(config_path=path)
        return cls._option_map[path]

    def __getitem__(self, key):
        if not isinstance(key, Options):
            raise TypeError()
        return self._params[key]