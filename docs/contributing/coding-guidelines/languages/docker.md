# Docker

This page describes how to write Dockerfiles in Autoware. pre-commit checks each Dockerfile with hadolint. The Dockerfiles of the [`docker/`](https://github.com/autowarefoundation/autoware/tree/2.0.0/docker) directory of the `autoware` repository follow these rules, and the examples on this page come from them.

## References

- [hadolint rules](https://github.com/hadolint/hadolint#rules)
- [Docker best practices for building images](https://docs.docker.com/build/building/best-practices/)
- [How the Autoware images are built](https://github.com/autowarefoundation/autoware/blob/2.0.0/docker/README.md)

## hadolint

pre-commit runs [hadolint](https://github.com/hadolint/hadolint) on all Dockerfiles. hadolint also checks the shell commands of the `RUN` instructions with ShellCheck. Fix every warning.

If a rule does not fit one instruction, turn it off for that instruction with a comment on the line before it. For example, `base.Dockerfile` turns off `DL3003` (use `WORKDIR` to change the directory) for a `RUN` instruction that uses `cd`:

```dockerfile
# hadolint ignore=DL3003
RUN --mount=type=bind,source=ansible,target=/tmp/ansible/ansible \
    ...
    pipx install --include-deps "ansible==10.*" && \
    cd /tmp/ansible && \
    ...
```

If a rule does not fit the whole repository, add it to the `.hadolint.yaml` file at the root of the repository, with a comment that gives the reason. For example, the [`.hadolint.yaml`](https://github.com/autowarefoundation/autoware/blob/2.0.0/.hadolint.yaml) of the `autoware` repository turns off `DL3002` (the last user must not be `root`), because the intermediate stages of the multi-stage builds need `root`.

## Name the files

Give each Dockerfile a name of the form `<name>.Dockerfile`, for example `base.Dockerfile` and `universe-cuda.Dockerfile`. Put them in the `docker/` directory. If a project builds more than one image, describe the targets in a [Docker Bake](https://docs.docker.com/build/bake/) file such as [`docker/docker-bake.hcl`](https://github.com/autowarefoundation/autoware/blob/2.0.0/docker/docker-bake.hcl).

## Use multi-stage builds

Build the code in one stage and copy only the result to the runtime stage. Give each stage a name with `AS`. For example, `universe.Dockerfile` builds Autoware in the `universe-devel` stage and copies `/opt/autoware` to the `universe` stage:

```dockerfile
FROM ${CORE_IMAGE} AS universe
# ...
COPY --from=universe-devel /opt/autoware /opt/autoware
```

## Set the shell

Set Bash with `pipefail` as the shell at the start of each stage, so that a failed command in a pipe fails the build (hadolint rule `DL4006`):

```dockerfile
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
```

## Install packages

- Install `apt` packages with `--no-install-recommends`.
- Use cache mounts for `apt`, `pip`, and `ccache`, so that the next build reuses the downloaded files:

  ```dockerfile
  RUN --mount=type=cache,id=apt-cache-${ROS_DISTRO},target=/var/cache/apt,sharing=locked \
      --mount=type=cache,id=apt-lists-${ROS_DISTRO},target=/var/lib/apt/lists,sharing=locked \
      apt-get update && \
      apt-get install -y --no-install-recommends \
      sudo \
      pipx
  ```

- Install the dependencies of ROS packages with `rosdep` from their `package.xml` files. Copy only the `package.xml` files before this step, so that a change to the source code does not run `rosdep` again:

  ```dockerfile
  COPY --parents --chown=${USERNAME}:${USERNAME} src/**/package.xml /tmp/autoware/
  ```

- Use the Ansible roles of the `autoware` repository to install other dependencies, so that the Docker images and the source installation install the same dependencies.

## Pin remote files

When you download a file with `ADD`, give its checksum. Then the build fails if the file changes. For example, `base.Dockerfile` pins the ROS package schemas:

```dockerfile
ADD --chmod=644 --checksum=sha256:585201809cbf8c81ec5a4d7d84d827f4676dd62a4bc0a812da7a485dea2ad43a \
    http://download.ros.org/schema/package_format3.xsd /usr/share/xml/ros/
```

## Users and permissions

The Autoware images create the user `aw` and run the commands of the build as this user where possible. The entrypoint runs as `root`. It changes the user and group IDs of `aw` to the IDs of the host user and applies the network settings for DDS. Then it starts the command as `aw` with `gosu`.

## Comments

Write a comment for each step that is not obvious. Say why the step is necessary, not only what it does. For example:

```dockerfile
# ament_xmllint downloads the package.xml schema from download.ros.org for every package,
# and its tests time out when that server is slow. The catalog points xmllint to local copies.
```
