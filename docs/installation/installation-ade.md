Installation with ADE {#installation-ade}
=====================

@tableofcontents

# Goals {#installation-and-development-goals}


This article demonstrates how to use the Agile Development Environment (ADE) to develop Autoware.Auto applications.


# Install ADE {#installation-and-development-install-ade}

[ADE](https://ade-cli.readthedocs.io/en/latest/) is a modular Docker-based tool to ensure that all developers in a project have a common, consistent development environment.

Follow the [install](https://ade-cli.readthedocs.io/en/latest/install.html) instructions, which are reproduced here for convenience:

1. Verify that the requirements [listed here](https://ade-cli.readthedocs.io/en/latest/install.html#requirements) are fulfilled. In particular, if docker was not used before, one may need to go through the [docker post-install steps](https://docs.docker.com/engine/install/linux-postinstall/).
2. Download the latest statically-linked binary for your platform from the [Releases](https://gitlab.com/ApexAI/ade-cli/-/releases) page of the `ade-cli` project
3. Name the binary `ade` and install it in `PATH`. On Ubuntu, `/usr/local/bin` is recommended for system-wide installation, otherwise choose e.g. `~/.local/bin` for a local installation that doesn't require `sudo` rights.
4. Make the binary executable: `chmod +x ade`
5. Check that it is installed:

```{bash}
$ which ade
/path/to/ade
$ ade --version
<version>
```

# Setup ADE home and project checkout {#installation-and-development-setup-ade-home-and-project-checkout}

ADE needs a directory on the host machine which is mounted as the user's
home directory within the container. The directory is populated with
dotfiles, and must be different than the user's home directory
*outside* of the container. In the event ADE is used for multiple, projects it
is recommended to use dedicated `adehome` directories for each project.

ADE looks for a directory containing a file named `.adehome`
starting with the current working directory and continuing with the
parent directories to identify the ADE home directory to be mounted.

```{bash}
$ mkdir -p ~/adehome
$ cd ~/adehome
$ touch .adehome
```

For ADE to function, it must be properly configured. Autoware.Auto provides
an [.aderc](https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/blob/master/.aderc) file
which is expected to exist in the current working
directory, or in any parent directory. Additionally, default configuration values can be
overridden by setting environment variables. See the `ade --help` output for more information about
using environment variables to define the configuration.

```{bash}
$ cd ~/adehome
$ git clone https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto.git
```

Checkout the [latest release](https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/releases) by checking out the corresponding tag or release branch.
Alternatively, when not checking out any specific tag, the latest `master` branch will be used
which may include features that are still being developed. For example:
```
$ cd AutowareAuto
$ git checkout tags/1.0.0 -b release-1.0.0
```

## Sharing files between the host system and ADE (Optional)
It might come in handy to share files such as dotfiles or utility programs from your host machine
with ADE. If you only have a single `adehome` directory, there is a way to do that without
duplicating them: move them inside the `adehome` directory, then create a symlink in the host system
to their regular location. For instance,

```{bash}
$ cd ~
$ cp ~/.bashrc ~/.bashrc.bak
$ mv ~/.bashrc ~/adehome/.bashrc
$ ln -s ~/adehome/.bashrc
```

It will then appear as `~/.bashrc` to the host system and to ADE.

Another option is to put utility programs into `~/adehome/.local/bin` and symlink. The opposite
direction will not work, files in a Docker container can not be symlinks to the outside.

@note The programs have to be self-contained! They should not depend on loading libraries from e.g.
`/usr/lib`.

@note There is a risk of an error (symlink would be broken and .bashrc would not be loaded when your terminal is started). In this case, you should delete the symlink and move .bashrc back to the original directory.

# Entering the development environment

```{bash}
$ cd AutowareAuto
```

To start the default environment:

```{bash}
$ ade start --update --enter
```

There are several preconfigured environments to choose from by specifying an ADE rc file. To see
what is available, run

```{bash}
ls -l .aderc*
```

Choose one, then launch with:

```{bash}
ade --rc .aderc-amd64-foxy  start --update --enter
```

Congratulations! Now you should have a terminal inside ADE:

```{bash}
$ade:~$
```

The next steps are to proceed to @ref usage, or to work on the Autoware.Auto code itself as
described in @ref contributors-guide.

# What is where inside ADE?

Upon entering, ADE outputs the images used to create the environment; e.g.

```{bash}
$ ade enter
Entering ade with following images:
ade-foxy    | 8b1e0efdde07 | master  | registry.gitlab.com/autowarefoundation/autoware.auto/autowareauto/amd64/ade-foxy:master
binary-foxy | 0e582f863d4c | master  | registry.gitlab.com/autowarefoundation/autoware.auto/autowareauto/amd64/binary-foxy:master
foxy        | 2020.06      | 2020.06 | registry.gitlab.com/autowarefoundation/autoware.auto/ade-lgsvl/foxy:2020.06
```

The images are mounted under `/opt`:

```{bash}
@ade:~$ ls /opt
AutowareAuto # image: binary-foxy:master
lgsvl        # image: ade-lgsvl/foxy:2020.06
ros          # image: ade-foxy:master
```

The code in `/opt/AutowareAuto` is built from a particular version of the master branch of
Autoware.Auto. The master branch is built multiple times a day in CI; see the [container
registry](https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/container_registry). With
`ade ... --update`, the latest available version of each image is downloaded.

# Cleanup {#installation-and-development-cleanup}

ADE uses Docker, and over time unused images, containers, and volumes begin to clutter the hard
drive. Follow the steps below to clean the Docker file system of stale images.


## Start relevant Docker resources {#installation-and-development-start-relevant-docker-resources}

First, verify that ADE is running:

```{bash}
$ cd ~/adehome/AutowareAuto
$ ade start
```

If ADE is used for more than one project, verify all ADE instances are running; the same rule
applies for any other non-ADE Docker containers that should be preserved.

\note
Docker resources that are not started/running **will be removed**!


## Docker disk usage {#installation-and-development-docker-disk-usage}

To assess the disk usage situation, run the following command:

```{bash}
$ docker system df
TYPE                TOTAL               ACTIVE              SIZE                RECLAIMABLE
Images              13                  11                  14.03GB             916.9MB (6%)
Containers          11                  0                   2.311MB             2.311MB (100%)
Local Volumes       17                  15                  5.411GB             17.8MB (0%)
Build Cache         0                   0                   0B                  0B
```


## Remove unused docker items {#installation-and-development-remove-unused-docker-items}

Use `docker system prune` to remove any Docker items not used for currently running containers:

```{bash}
$ docker system prune -a --volumes
```

# Troubleshooting {#ade-troubleshooting}
Here are solutions for a few specific errors:

## Error - "forward compatibility was attempted on non supported hw" when starting ADE

When starting `ade` with GPU support enabled for NVIDIA graphics, you may sometimes receive the following error:

```{bash}
docker: Error response from daemon: OCI runtime create failed: container_linux.go:349: starting container process caused "process_linux.go:449: container init caused \"process_linux.go:432: running prestart hook 0 caused \\\"error running hook: exit status 1, stdout: , stderr: nvidia-container-cli: initialization error: cuda error: forward compatibility was attempted on non supported hw\\\\n\\\"\"": unknown.
ERROR: Command return non-zero exit code (see above): 125
```

This usually indicates that a new NVIDIA graphics driver has been installed (usually via `apt`) but the system has not yet been restarted. A similar message may appear if the graphics driver is not available, for example because of resuming after suspend.

### Solution

Restart your system after installing the new NVIDIA driver.

```{bash}
ade$ exit
$ ade stop
$ ade start --update --enter
```
