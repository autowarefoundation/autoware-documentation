# Fixing dependent package versions

Autoware manages dependent package versions in autoware.repos.
For example, let's say you make a branch in autoware_universe and add new features.
Suppose you update other dependencies with `vcs pull` after cutting a branch from autoware_universe. Then the version of autoware_universe you are developing and other dependencies will become inconsistent, and the entire Autoware build will fail.
We recommend saving the dependent package versions by executing the following command when starting the development.

```bash
vcs export src --exact > my_autoware.repos
```
