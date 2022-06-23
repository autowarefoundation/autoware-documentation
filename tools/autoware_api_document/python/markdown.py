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


class MarkdownTable(object):

    def __init__(self, *header):
        self.header = header
        self.table = []

    def line(self, *values):
        self.table.append(values)

    def text(self):
        return '\n'.join('| {} |'.format(line) for line in self.texts())

    def texts(self):
        width = map(len, self.header)
        for line in self.table:
            width = map(max, width, map(len, line))
        width = list(width)
        yield ' | '.join(s.ljust(w) for s, w in zip(self.header, width))
        yield ' | '.join('-' * w for w in width)
        for line in self.table:
            yield ' | '.join(s.ljust(w) for s, w in zip(line, width))
