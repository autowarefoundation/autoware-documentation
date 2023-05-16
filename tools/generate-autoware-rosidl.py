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


def strip_array_suffix(name):
    return name.split("[")[0]


def normalize_msg_name(name: str):
    parts = str(name).split("/")
    return parts[0] if len(parts) == 1 else f"{parts[0]}/msg/{parts[1]}"


def parse_rosidl_spec(depends: set, visited: set, spec: MessageSpecification):
    fields = {field.name: normalize_msg_name(field.type) for field in spec.fields}
    for name in fields.values():
        name = strip_array_suffix(name)
        if name not in visited:
            depends.add(name)
    return fields


def parse_rosidl_file(depends: set, visited: set, specs: dict, name: str):
    visited.add(name)
    parts = name.split("/")
    if len(parts) != 3:
        return
    pkg, ext, msg = parts
    if pkg not in ["autoware_adapi_version_msgs", "autoware_adapi_v1_msgs"]:
        return
    path = Path(get_package_share_directory(pkg)).joinpath(ext, msg).with_suffix("." + ext)
    if ext == "msg":
        message = parse_message_file(pkg, path)
        specs["msg"][name] = parse_rosidl_spec(depends, visited, message)
    if ext == "srv":
        service = parse_service_file(pkg, path)
        specs["req"][name] = parse_rosidl_spec(depends, visited, service.request)
        specs["res"][name] = parse_rosidl_spec(depends, visited, service.response)


def main():
    adapi = Path("docs/design/autoware-interfaces/ad-api/list/api")
    yamls = (load_markdown_yaml(path) for path in adapi.glob("**/*.md"))
    names = (yaml["type"]["name"] for yaml in yamls if yaml)

    # Create field list of messages and services.
    visited = set()
    depends = set(names)
    specs = {"msg": {}, "req": {}, "res": {}}
    while depends:
        name = depends.pop()
        parse_rosidl_file(depends, visited, specs, name)

    # Remove empty fields.
    for key in ("msg", "req", "res"):
        specs[key] = {k: v for k, v in specs[key].items() if v}

    data = {"rosidl": specs}
    Path("yaml/autoware-rosidl.yaml").write_text(yaml.safe_dump(data))

main()
