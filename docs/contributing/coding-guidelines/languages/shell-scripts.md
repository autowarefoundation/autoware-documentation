# Shell scripts

This page describes the rules for shell scripts in Autoware. pre-commit checks each script with ShellCheck and formats it with shfmt.

## References

Follow the Google Shell Style Guide when this page does not define a rule:

- <https://google.github.io/styleguide/shellguide.html>

## Use Bash

Write scripts for Bash. Start each script with a shebang line:

```bash
#!/usr/bin/env bash
```

`#!/bin/bash` is also used, for example in [`docker/docker-entrypoint.sh`](https://github.com/autowarefoundation/autoware/blob/2.0.0/docker/docker-entrypoint.sh). If a script must run in `sh`, use only POSIX features in it.

## Stop on errors

Put `set -euo pipefail` after the shebang and the header comment:

- `-e` stops the script when a command fails.
- `-u` stops the script when it uses a variable that is not set.
- `-o pipefail` makes a pipeline fail when any command in it fails, not only the last command.

For example, [`ansible/scripts/install-ansible.sh`](https://github.com/autowarefoundation/autoware/blob/2.0.0/ansible/scripts/install-ansible.sh) starts with:

```bash
#!/usr/bin/env bash
# Install Ansible (and its OS prerequisites) so that the playbooks under
# autoware/ansible/ can be run.
# ...

set -euo pipefail
```

If a command can fail and the script must continue, handle the failure on that line, for example with `|| true`.

## ShellCheck

pre-commit checks all shell scripts with [ShellCheck](https://github.com/koalaman/shellcheck). Fix every warning. For example, ShellCheck warns about a variable without quotes (`SC2086`). Put variables in double quotes, for example `"${HOST_UID}"`, so that a value with spaces stays one argument.

If a warning is a false positive, add a `# shellcheck` directive on the line before the command. A directive such as `# shellcheck disable=SC2012` turns off one check for that command. Write the reason in a comment above the directive. As another example, `docker-entrypoint.sh` sources a file that ShellCheck cannot read, so it tells ShellCheck not to follow the file:

```bash
# shellcheck source=/dev/null
source "/opt/ros/${ROS_DISTRO}/setup.bash"
```

The `autoware` repository has a [`.shellcheckrc`](https://github.com/autowarefoundation/autoware/blob/2.0.0/.shellcheckrc) file with `external-sources=true`. With this option, ShellCheck follows the files that a script sources.

## shfmt

pre-commit formats all shell scripts with [shfmt](https://github.com/mvdan/sh) and these options:

```yaml
- repo: https://github.com/scop/pre-commit-shfmt
  hooks:
    - id: shfmt
      args: [-w, -s, -i=4]
```

- `-w` writes the result to the file.
- `-s` simplifies the code, for example it removes unnecessary `$` in arithmetic expressions.
- `-i=4` indents with four spaces.

Thus, indent shell scripts with four spaces, not with tabs.

## Style

- Give functions and variables `snake_case` names. Give environment variables and constants `UPPER_SNAKE_CASE` names.
- Use `"${var}"` with braces for variables in strings.
- Use `$(command)` for command substitution, not backticks.
- Send error messages to standard error, for example `echo "error: file not found" >&2`.
- If a script needs root permissions for some commands, use `sudo` for those commands only. Do not run the whole script as root.
