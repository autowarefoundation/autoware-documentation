import os  # for relpath, cannot use pathlib
import urllib


def define_env(env):
    @env.filter
    def drawio(image_path):
        image_url = urllib.parse.quote(f"{env.conf['site_url']}{image_path}", "")
        return f"https://app.diagrams.net/?lightbox=1#U{image_url}"

    @env.macro
    def link_ad_api(name):
        full_path = "design/autoware-interfaces/ad-api/list" + name
        link_path = os.path.relpath(full_path, env.page.url)
        docs_path = os.path.join(env.conf["docs_dir"], full_path) + ".md"
        return f"[{name}]({link_path})" if os.path.exists(docs_path) else name
