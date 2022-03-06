# Commit guidelines

This page explains our guidelines related to committing.

## Branch rules

### Start branch names with the corresponding issue numbers (advisory, non-automated)

#### Rationale

- Developers can quickly find the corresponding issues.
- It is helpful for tools.
- It is consistent with GitHub's default behavior.

#### Exception

If there are no corresponding issues, you can ignore this rule.

#### Example

```text
123-add-feature
```

#### Reference

- [GitHub Docs](https://docs.github.com/en/issues/tracking-your-work-with-issues/creating-a-branch-for-an-issue)

### Use `kebab-case` for the separator of branch names (advisory, non-automated)

#### Rationale

- It is consistent with GitHub's default behavior.

#### Example

```text
123-add-feature
```

#### Reference

- [GitHub Docs](https://docs.github.com/en/issues/tracking-your-work-with-issues/creating-a-branch-for-an-issue)

### Make branch names descriptive (advisory, non-automated)

#### Rationale

- It can avoid conflicts of names.
- Developers can understand the purpose of the branch.

#### Exception

If you have already submitted a pull request, you do not have to change the branch name because you need to re-create a pull request, which is noisy and a waste of time.  
Be careful from the next time.

#### Example

Usually it is good to start with a verb.

```text
123-fix-memory-leak-of-trajectory-follower
```

## Commit rules

### Sign-off your commits (required, automated)

Developers must certify that they wrote or otherwise have the right to submit the code they are contributing to the project.

#### Rationale

If not, it will lead to complex license problems.

#### Example

```bash
git commit -s
```

```text
feat: add a feature

Signed-off-by: Autoware <autoware@example.com>
```

#### Reference

- [GitHub Apps - DCO](https://github.com/apps/dco)

### Follow `Conventional Commits` (required, automated)

#### Rationale

- It can generate categorized changelog, for example using [git-cliff](https://github.com/orhun/git-cliff).

#### Example

```text
feat(trajectory_follower): add an awesome feature
```

Since Autoware uses the `Squash and merge` method of GitHub, you need to make the PR title follow `Conventional Commits` as well.

#### Reference

- [Conventional Commits](https://www.conventionalcommits.org/en/v1.0.0/)
- [commitizen/conventional-commit-types](https://github.com/commitizen/conventional-commit-types)
- [GitHub Docs](https://docs.github.com/en/repositories/configuring-branches-and-merges-in-your-repository/configuring-pull-request-merges/about-merge-methods-on-github#squashing-your-merge-commits)

### Add the related node names to the scope of `Conventional Commits` (advisory, non-automated)

#### Rationale

- The package maintainer can become aware of the pull request by seeing the notification.
- It can make the changelog clearer.

#### Example

```text
feat(trajectory_follower): add an awesome feature
```

### Keep a pull request small (advisory, non-automated)

#### Rationale

- Small pull requests are easy to understand for reviewers.
- Small pull requests are easy to revert for maintainers.

#### Exception

It is acceptable if it is agreed with maintainers that there is no other way but to submit a big pull request.

#### Example

- Avoid developing two features in one pull request.
- Avoid mixing `feat`, `fix`, and `refactor` in the same commit.
