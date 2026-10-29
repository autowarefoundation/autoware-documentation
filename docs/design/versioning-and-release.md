# Versioning and the release process

We use semantic versioning [(SemVer)](https://semver.org/) for releases.

## Frequency

Releases are made approximately monthly.

## Constraints and the release process

Autoware has many repositories that are expected to work together.

When doing a release, we first make releases from sub-repositories such as `autoware_core`, `autoware_universe`, etc.

⚠️ Autoware has its own version number. It does not follow the version of `autoware_core` or of any other sub-repository. For example, Autoware `2.0.0` uses `autoware_core` `1.10.0`.

ℹ️ Each sub-repository tracks its own version. The `repositories/autoware.repos` file at each Autoware release tag records the version of each sub-repository.

Before Autoware `2.0.0`, the Autoware version was the same as the `autoware_core` version.

Some repositories might not have changed at all, we can refer to their existing versions.

We will release all the _required_ sub-repositories and update their references within the `repositories/autoware.repos` file.

We then test this system with various [demos](../demos/index.md) manually. And make sure the CI for the build passes.

Then we make an Autoware release.

## Patches

If there is a critical bug that needs to be fixed, we will make a patch release for the affected repositories.

Then we will go up a patch version release for Autoware.
