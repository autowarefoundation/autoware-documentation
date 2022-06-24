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

import argparse
import pathlib
from ament_index_python.packages import get_package_share_directory
from .specification import AutowareAPI
from .specification import TypeDefinition
from .markdown import MarkdownTable


def generate():
    parser = argparse.ArgumentParser()
    parser.add_argument('path', default='docs/design/autoware-interfaces/prototyping', nargs='?')
    args = parser.parse_args()

    target = pathlib.Path(args.path)
    if not target.exists():
        raise Exception('target is not found')

    groups = [
        ('external', 'autoware_api_document', 'resource/tier4.yaml'),
    ]
    groups = [(data[0], list(AutowareAPI.Load(target, *data[1:]))) for data in groups]

    # create list
    generate_list(target, groups)

    # create page
    for _, specs in groups:
        for spec in specs:
            if spec.page:
                generate_page(target, spec)

    # create type
    for definition in TypeDefinition.definitions.values():
        if definition.page:
            generate_type(target, definition)


def generate_list(target, groups):
    with target.joinpath('index.md').open('w') as fp:
        fp.write('# List of TIER IV API\n\n')
        for group, specs in groups:
            table = MarkdownTable('Type', 'Name', 'Data')
            for spec in specs:
                spec_name = make_page_link(spec)
                type_name = make_type_link(spec)
                table.line(spec.behavior, spec_name, type_name)
            fp.write(table.text() + '\n')


def make_page_link(spec : AutowareAPI):
    if spec.page is None:
        return spec.spec_name
    spec_link = '.' + spec.spec_name + '.md'
    return '[{}]({})'.format(spec.spec_name, spec_link)


def make_type_link(spec : AutowareAPI):
    if spec.typedef.page is None:
        return spec.data_type
    data_link = './type/' + spec.data_type + '.md'
    return '[{}]({})'.format(spec.data_type, data_link)


def generate_page(target, spec):
    text = spec.page.read_text().split('## Description')
    text = text[0] if len(text) < 2 else text[1]
    line = [
        '# ' + spec.spec_name,
        '',
        '## Classification',
        '',
        '- Behavior: ' + spec.behavior.capitalize(),
        '- DataType: ' + spec.data_type,
        '',
        '## Description',
    ]
    spec.page.write_text('\n'.join(line) + '\n\n' + text.strip() + '\n')


def generate_type(target, definition):
    # use html tag to enable hyperlink in code block
    lines = [
        '# ' + definition.name,
        '',
        '```txt',
        definition.page,
        '```',
    ]
    path = target.joinpath('type', definition.path).with_suffix('.md')
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text('\n'.join(lines) + '\n')
