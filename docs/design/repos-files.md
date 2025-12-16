# `repositories/*.repos` files

Autoware uses **multiple Git repositories** managed through a meta-repository approach.

The `autoware` repository itself **does not contain the full source code**. Instead, it references many separately maintained repositories.

These files reside under [autoware/repositories](https://github.com/autowarefoundation/autoware/blob/main/repositories) directory.

<div class="grid cards" markdown>

- `autoware.repos`

  ***

  Holds the references to essential autoware repositories.

- `autoware-nightly.repos`

  ***

  This sets some of the `autoware.repos` repositories to the main branch for development.

- `tools.repos`

  ***

  Mainly holds the reference to [autowarefoundation/autoware_tools](https://github.com/autowarefoundation/autoware_tools.git) repository.

- `tools-nightly.repos`

  ***

  Sets the tools repository to the main branch.

- `simulator.repos`

  ***

  Holds the reference to [tier4/scenario_simulator_v2](https://github.com/tier4/scenario_simulator_v2.git) repository.

- `extra-packages.repos`

  ***

  Holds optional repositories. Right now holds the references to [tier4/pacmod_interface](https://github.com/tier4/pacmod_interface.git) and [tier4/tamagawa_imu_driver](https://github.com/tier4/tamagawa_imu_driver.git).

</div>

## How to use `.repos` files

### Prerequisites

Autoware uses [vcs2l](https://github.com/dirk-thomas/vcs2l) to construct workspaces.

You can install it with `sudo apt install python3-vcs2l`.

### Configurations

Always first pull the `autoware.repos` since it holds the essential repositories.

```bash
vcs import src < repositories/autoware.repos
```

For most cases, this will be enough.

If you want to use the nightly versions of the repositories, also pull the `autoware-nightly.repos` file. (It requires `autoware.repos` to be pulled first.)

```bash
vcs import src < repositories/autoware-nightly.repos
```

If you want to use the other repositories such as scenario simulator or the tools, pull the corresponding `.repos` files.

### Managing repositories

Generally, the following command should be enough to update all the repositories.

```bash
vcs pull src
```

But if some repositories have not-committed changes, you may need to manage them manually.

!!! tip

    You can run `vcs status src` to check the status of the repositories.

!!! note "`vcs help`"

    The available commands are:
    ```
    branch     Show the branches
    custom     Run a custom command
    delete     Remove the directories indicated by the list of given repositories.
    diff       Show changes in the working tree
    export     Export the list of repositories
    import     Import the list of repositories
    log        Show commit logs
    pull       Bring changes from the repository into the working copy
    push       Push changes from the working copy to the repository
    remotes    Show the URL of the repository
    status     Show the working tree status
    validate   Validate the repository list file
    ```
    You can call them with `vcs <command> src`.
