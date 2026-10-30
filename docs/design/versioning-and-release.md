# Versioning and the release process

We use semantic versioning [(SemVer)](https://semver.org/) for releases.

## Frequency

Recent releases came about every two to three months: `1.8.0` in May 2026, `1.9.0` in July 2026, and `2.0.0` in October 2026. The [releases page](https://github.com/autowarefoundation/autoware/releases) lists all releases.

## Version numbers

!!! warning "Autoware has its own version number"

    The Autoware version does not follow the version of `autoware_core` or of any other sub-repository. For example, Autoware `2.0.0` uses `autoware_core` `1.10.0`.
    Before Autoware `2.0.0`, the Autoware version was the same as the `autoware_core` version.

Each sub-repository tracks its own version. The `repositories/autoware.repos` file at each Autoware release tag records the version of each sub-repository.

## The release process

Autoware has many repositories that must work together. A release follows these steps:

<div class="aw-steps" markdown>

1. **Release the sub-repositories.** We release all the _required_ sub-repositories, such as `autoware_core` and `autoware_universe`. A repository that did not change keeps its existing version.
2. **Update the references.** We update the versions of the sub-repositories in the `repositories/autoware.repos` file.
3. **Test the system.** We test the system manually with various [demos](../demos/index.md), and we make sure that the CI for the build passes.
4. **Release Autoware.** We make the Autoware release.

</div>

## Patches

If a critical bug needs a fix, we make a patch release of the affected repositories. Then we make a patch release of Autoware.
