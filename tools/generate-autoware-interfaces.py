#!/usr/bin/env python3

import yaml
from pathlib import Path

from rosidl_adapter.parser import MessageSpecification
from rosidl_adapter.parser import parse_message_file
from rosidl_adapter.parser import parse_service_file
from ament_index_python.packages import get_package_share_directory


def load_markdown_metadata(path: Path):
    lines = path.read_text().splitlines()
    if (2 < len(lines)) and (lines[0] == "---"):
        data = lines[1:lines.index("---", 1)]
        return yaml.safe_load("\n".join(data))
    return None


def is_documentation_msg(name: str):
    targets = set(["autoware_adapi_version_msgs", "autoware_adapi_v1_msgs"])
    return name.split("/")[0] in targets


def strip_array_suffix(name: str):
    return name.split("[")[0]


def resolve_msg_file_path(name: str):
    pkg, ext, msg = name.split("/")
    return Path(get_package_share_directory(pkg)).joinpath(ext, msg).with_suffix("." + ext)

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
    if not is_documentation_msg(name):
        return
    pkg, ext, msg = name.split("/")
    path = Path(get_package_share_directory(pkg)).joinpath(ext, msg).with_suffix("." + ext)
    if ext == "msg":
        msg = parse_message_file(pkg, path)
        msg = parse_rosidl_spec(depends, visited, msg)
        specs[name] = {"msg": msg}
        specs[name] = {k: v for k, v in specs[name].items() if v}
    if ext == "srv":
        srv = parse_service_file(pkg, path)
        req = parse_rosidl_spec(depends, visited, srv.request)
        res = parse_rosidl_spec(depends, visited, srv.response)
        specs[name] = {"req": req, "res": res}
        specs[name] = {k: v for k, v in specs[name].items() if v}


def generate_type_page(name: str, uses: set, used: set):
    path = Path("docs/design/autoware-interfaces/ad-api/types").joinpath(name).with_suffix(".md")
    print(path)
    print(resolve_msg_file_path(name))


def main():
    # Create a list of data types used in adapi.
    adapi = Path("docs/design/autoware-interfaces/ad-api/list/api")
    yamls = (load_markdown_metadata(path) for path in adapi.glob("**/*.md"))
    names = (yaml["type"]["name"] for yaml in yamls if yaml)

    # Create a field list for each data type.
    visited = set()
    depends = set(names)
    specs = {}
    while depends:
        name = depends.pop()
        parse_rosidl_file(depends, visited, specs, name)

    # Export a field list.
    data = {"types": specs}
    Path("yaml/autoware-interfaces.yaml").write_text(yaml.safe_dump(data))

    return

    # Create pages of data types
    type_uses = {name: set() for name in visited if is_documentation_msg(name)}
    type_used = {name: set() for name in visited if is_documentation_msg(name)}
    for key in ("msg", "req", "res"):
        for name, fields in specs[key].items():
            for field in fields.values():
                field = strip_array_suffix(field)
                if is_documentation_msg(field):
                    type_uses[name].add(field)
                    type_used[field].add(name)
    for name in visited:
        if is_documentation_msg(name):
            generate_type_page(name, type_uses[name], type_used[name])

main()
