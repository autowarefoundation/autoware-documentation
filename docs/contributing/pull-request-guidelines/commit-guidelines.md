# Commit guidelines

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

### Use `dash-case` for the separator of branch names (advisory, non-automated)

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
