import os  # for relpath, cannot use pathlib
import urllib


def define_env(env):
    @env.filter
    def drawio(image_path):
        image_url = urllib.parse.quote(f"{env.conf['site_url']}{image_path}", "")
        return f"https://app.diagrams.net/?lightbox=1#U{image_url}"

    @env.macro
    def create_relative_link(text, root_path):
        path = os.path.relpath(root_path, os.path.dirname(env.page.file.src_uri))
        return f"[{text}]({path})"

    @env.macro
    def link_ad_api(name):
        return create_relative_link(name, f"design/autoware-interfaces/ad-api/list/{name}.md")

    @env.macro
    def resolve_msg_field(type, name, ext):
        specs = env.variables["autoware_interfaces"]["types"]
        for field in name.split("."):
            type = type.split("[")[0]
            type = specs[type][ext][field]
            ext = "msg"
        return type
