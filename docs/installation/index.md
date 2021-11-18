# Installation

The supported environments are specified in [target-environments](target-environments.md).

The recommended method for installation is through the use of [ADE](https://ade-cli.readthedocs.io/en/latest/),
a Docker-based tool to ensure that all developers in a project have a common, consistent development
environment. It comes with a pre-built version of Autoware.Auto, so that you will not need to compile it yourself
if you do not want to.

- [installation-ade](installation-ade.md)
- [installation-ade-arm64](installation-ade-arm64.md)

Autoware.Core can also be built without the use of [ADE](https://ade-cli.readthedocs.io/en/latest/)
for cases where a more granular control of the installation environment is needed.

- [installation-no-ade](installation-no-ade.md)

A prerequisite for running the full software stack with simulation is the LGSVL simulator:

- [lgsvl](lgsvl.md)
