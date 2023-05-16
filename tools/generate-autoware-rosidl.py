#!/usr/bin/env python3

import yaml
from pathlib import Path

from rosidl_adapter.parser import MessageSpecification
from rosidl_adapter.parser import parse_message_file
from rosidl_adapter.parser import parse_service_file
from ament_index_python.packages import get_package_share_directory

def load_markdown_yaml(path: Path):
    lines = path.read_text().splitlines()
    if (2 < len(lines)) and (lines[0] == "---"):
        data = lines[1:lines.index("---", 1)]
        return yaml.safe_load("\n".join(data))
    return None


def parse_rosidl_spec(spec: MessageSpecification):
    fields = dict()
    for field in spec.fields:
        fields[field.name] = str(field.type)
    return fields


def parse_rosidl_file(name: str, specs: dict):
    pkg, ext, msg = name.split("/")
    path = Path(get_package_share_directory(pkg)).joinpath(ext, msg).with_suffix("." + ext)
    if ext == "msg":
        specs["msg"][pkg] = parse_rosidl_spec(parse_message_file(pkg, path))
    if ext == "srv":
        service = parse_service_file(pkg, path)
        specs["req"][name] = parse_rosidl_spec(service.request)
        specs["res"][name] = parse_rosidl_spec(service.response)
    return None


def main():
    adapi = Path("docs/design/autoware-interfaces/ad-api/list/api")
    yamls = (load_markdown_yaml(path) for path in adapi.glob("**/*.md"))
    names = (yaml["interface"]["type"] for yaml in yamls if yaml)

    specs = {"msg": {}, "req": {}, "res": {}}
    for name in names:
        parse_rosidl_file(name, specs)

    data = {"rosidl": specs}
    Path("yaml/autoware-rosidl.yaml").write_text(yaml.safe_dump(data))

main()
