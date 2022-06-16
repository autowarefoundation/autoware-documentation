import os  # for relpath, cannot use pathlib


def define_env(env):

    @env.macro
    def link_ad_api(name):
        root_path = "design/autoware-interfaces/ad-api"
        page_path = env.page.file.src_path
        base_path = os.path.relpath(root_path, page_path) + "/list"
        return f"[{name}]({base_path}{name})"
