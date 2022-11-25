import os  # for relpath, cannot use pathlib
import urllib


def define_env(env):

    @env.macro
    def link_ad_api(name):
        root_path = "design/autoware-interfaces/ad-api"
        base_path = os.path.relpath(root_path, env.page.url) + "/list"
        return f"[{name}]({base_path}{name})"

    @env.filter
    def drawio(image_path):
        image_url = urllib.parse.quote(f"{env.conf['site_url']}{image_path}", "")
        return f"https://app.diagrams.net/?lightbox=1#U{image_url}"
