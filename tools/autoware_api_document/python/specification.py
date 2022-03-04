# Copyright 2021 Tier IV, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import pathlib
import re
import yaml
from ament_index_python.packages import get_package_share_directory


class TypeDefinition(object):

    definitions = {}
    my_packages = {
        'autoware_external_api_msgs',
        'tier4_external_api_msgs'
    }

    def __init__(self, path):
        self._path = path
        self._page = self.__load_page(self._path)

    def __load_page(self, name):
        name = name.split('/')
        path = pathlib.Path(get_package_share_directory(name[0]))
        path = path.joinpath(*name[1:]).with_suffix('.' + name[1])
        return path.read_text().strip() if path.exists() else None

    @property
    def name(self):
        return self._path.split('/')[-1]

    @property
    def path(self):
        return self._path

    @property
    def page(self):
        return self._page

    @classmethod
    def Get(cls, name):
        definition = cls.definitions.get(name)
        if not definition:
            definition = TypeDefinition(name)
            cls.definitions[name] = definition
        return definition

class AutowareAPI(object):

    def __init__(self, target, name, data):
        self._name = name.strip('/').split('/')
        self._data = data
        self._page = self.__load_page(target, name)
        self._type = self.__load_type(data['type'])

    def __load_page(self, target, name):
        path = target.joinpath(self.spec_name.strip('/')).with_suffix('.md')
        return path if path.exists() else None

    def __load_type(self, name):
        return TypeDefinition.Get(name)

    @property
    def spec_name(self):
        return '/api/' + '/'.join(self._name)

    @property
    def data_type(self):
        return self._data.get('type', '---')

    @property
    def behavior(self):
        return self._data['behavior']

    @property
    def page(self):
        return self._page

    @property
    def typedef(self):
        return self._type

    @staticmethod
    def Load(target, package: str, path : str):
        package = get_package_share_directory(package)
        path = pathlib.Path(package, path)
        for name, data in yaml.safe_load(path.read_text()).items():
            yield AutowareAPI(target, name, data)
