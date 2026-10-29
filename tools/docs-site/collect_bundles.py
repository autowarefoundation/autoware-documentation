# Copyright 2026 The Autoware Contributors
# SPDX-License-Identifier: Apache-2.0

"""Download one built site per version of the documentation.

The versions come from GitHub itself:

- every published release, whose built site the github-release workflow
  attaches to the release as an archive
- the default branch, whose built site the newest successful deploy-docs run
  of a push keeps as an artifact
- every open pull request from this repository, except those of Dependabot,
  whose built site its newest successful deploy-docs run keeps as an artifact

A pull request from a fork is left out, so that only reviewed code of this
repository reaches the site. A version whose site cannot be found, for example
because its artifact expired, is left out with a warning.

The script only downloads files. It never runs code from a pull request.
"""

import argparse
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tarfile

RELEASE_ASSET = "autoware-docs-site.tar.gz"
WORKFLOW = "deploy-docs.yaml"

# How the GitHub CLI names the author of a Dependabot pull request.
DEPENDABOT = {"app/dependabot", "dependabot[bot]"}


def gh(*arguments):
    """Run the GitHub CLI and return what it prints."""
    result = subprocess.run(["gh", *arguments], check=True, capture_output=True, text=True)
    return result.stdout


def gh_json(*arguments):
    return json.loads(gh(*arguments))


def warn(message):
    print(f"::warning::{message}", flush=True)


def artifact_name(version):
    """Return the name of the artifact that holds the built site of a version."""
    return f"docs-{version}"


def extract_release(archive, destination):
    """Extract a release archive, whose files sit in a top directory "site"."""
    with tarfile.open(archive) as tar:
        members = []
        for member in tar.getmembers():
            parts = Path(member.name).parts
            if len(parts) < 2 or parts[0] != "site":
                continue
            member.name = str(Path(*parts[1:]))
            members.append(member)
        destination.mkdir(parents=True)
        tar.extractall(destination, members=members, filter="data")


def collect_releases(repository, output, cache):
    """Download the site of every published release, from the cache if possible."""
    cache.mkdir(parents=True, exist_ok=True)
    releases = gh_json(
        "release", "list", "-R", repository, "--limit", "1000", "--json", "tagName,isDraft"
    )
    for release in releases:
        tag = release["tagName"]
        if release["isDraft"]:
            continue
        archive = cache / f"{tag}.tar.gz"
        if not archive.exists():
            partial = cache / f"{tag}.tar.gz.partial"
            try:
                gh(
                    "release",
                    "download",
                    tag,
                    "-R",
                    repository,
                    "-p",
                    RELEASE_ASSET,
                    "-O",
                    str(partial),
                    "--clobber",
                )
            except subprocess.CalledProcessError:
                warn(f"Release {tag} has no {RELEASE_ASSET}, so it is left out.")
                continue
            partial.rename(archive)
        extract_release(archive, output / tag)
        print(f"Release {tag}: from {archive.name}")


def download_artifact(repository, run_id, version, output):
    """Download the site artifact of a run. Return True if it was found."""
    destination = output / version
    try:
        gh(
            "run",
            "download",
            str(run_id),
            "-R",
            repository,
            "-n",
            artifact_name(version),
            "-D",
            str(destination),
        )
    except subprocess.CalledProcessError:
        shutil.rmtree(destination, ignore_errors=True)
        return False
    return True


def collect_default_branch(repository, branch, output):
    """Download the site of the newest successful build of the default branch."""
    runs = gh_json(
        "run",
        "list",
        "-R",
        repository,
        "--workflow",
        WORKFLOW,
        "--branch",
        branch,
        "--status",
        "success",
        "--limit",
        "20",
        "--json",
        "databaseId,event",
    )
    for run in runs:
        if run["event"] in ("push", "workflow_dispatch"):
            if download_artifact(repository, run["databaseId"], branch, output):
                print(f"Default branch {branch}: from run {run['databaseId']}")
                return
    raise RuntimeError(f"No successful build of {branch} has a site artifact")


def collect_pull_requests(repository, output):
    """Download the preview of every open pull request from this repository."""
    pulls = gh_json(
        "pr",
        "list",
        "-R",
        repository,
        "--state",
        "open",
        "--limit",
        "500",
        "--json",
        "number,headRefName,headRefOid,isCrossRepository,author",
    )
    for pull in pulls:
        if pull["isCrossRepository"]:
            continue  # A fork: its site stays an artifact.
        if pull["author"]["login"] in DEPENDABOT:
            continue  # Dependabot only updates workflows, so it gets no preview.
        version = f"pr-{pull['number']}"
        runs = gh_json(
            "run",
            "list",
            "-R",
            repository,
            "--workflow",
            WORKFLOW,
            "--branch",
            pull["headRefName"],
            "--event",
            "pull_request",
            "--status",
            "success",
            "--limit",
            "20",
            "--json",
            "databaseId,headSha",
        )
        # The run of the newest commit first, then older runs.
        runs.sort(key=lambda run: run["headSha"] != pull["headRefOid"])
        for run in runs:
            if download_artifact(repository, run["databaseId"], version, output):
                print(f"Pull request {version}: from run {run['databaseId']}")
                break
        else:
            warn(f"Pull request {version} has no site artifact, so it is left out.")


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--repository", required=True, help="owner/name of the repository")
    parser.add_argument("--default-branch", required=True, help="branch of the default version")
    parser.add_argument("--output", required=True, help="directory to write one site per version")
    parser.add_argument("--release-cache", required=True, help="directory that keeps the archives")
    args = parser.parse_args()

    output = Path(args.output)
    if output.exists():
        shutil.rmtree(output)
    output.mkdir(parents=True)
    collect_releases(args.repository, output, Path(args.release_cache))
    collect_default_branch(args.repository, args.default_branch, output)
    collect_pull_requests(args.repository, output)
    return 0


if __name__ == "__main__":
    sys.exit(main())
