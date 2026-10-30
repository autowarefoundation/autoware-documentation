<!-- cspell: ignore Swarmia -->

# Review guidelines

This page explains how to review a pull request in Autoware repositories.
Authors can read it too, so that they know what the reviewers look for.

Anyone can review a pull request.
In some repositories, a review from a person who is not a code owner is not enough to merge, but it helps the code owners and the maintainers to decide.
For who is asked to review and why, refer to the [code owners](code-owners.md) guidelines.

## Merge requirements

Each repository sets its own merge rules on GitHub. For example:

| Repository               | Required approvals | Code owner approval required |
| :----------------------- | :----------------- | :--------------------------- |
| `autoware_universe`      | 1                  | Yes                          |
| `autoware_core`          | 2                  | Yes                          |
| `autoware-documentation` | 1                  | No                           |

The required [CI checks](ci-checks.md) must also pass.
In `autoware-documentation`, all review conversations must also be resolved before the merge.
Most code repositories use squash merges, so the pull request becomes one commit on the default branch.

## Before you review

1. Read the pull request description, the linked issue, and the related discussion.
2. Make sure that you understand the purpose of the change. If the description does not explain it, ask the author first.
3. Look at the CI results. If a required check fails because of the change, ask the author to fix it before you review the details.
4. If you want to run the code, check out the branch with `gh pr checkout`. For this and other tools, refer to [review tips](review-tips.md).

## What to review

### Purpose and scope

- The change solves the problem in the linked issue or discussion.
- The pull request does one thing. It does not mix a feature, a fix, and a refactoring. Refer to [Keep a pull request small](index.md#keep-a-pull-request-small-advisory-non-automated).
- A large change was agreed with the maintainers before the pull request, as the [contribution guidelines](../index.md#pull-requests) require.

### Design

- The change fits the [Autoware architecture](../../design/index.md) and the interfaces of the component.
- The change uses existing libraries and utilities where they exist, for example the packages in `autoware_core` and `autoware_utils`.
- A change to a topic, a message, a service, or a parameter is necessary, and the author lists it in the `Interface changes` section of the description.
- The `Effects on system behavior` section is correct.

### Correctness

- The code does what the description says.
- The code handles edge cases: empty inputs, missing data, invalid parameters, time jumps, and coordinate frame mismatches.
- The code does not block callbacks or allocate large memory in a loop that runs at a high rate.
- Errors are reported with a clear message.

### Tests

- New behavior has tests, and fixed bugs have a test that fails without the fix. Refer to the [testing guidelines](../testing-guidelines/index.md).
- The `How was this PR tested?` section says how the author tested the change, for example with unit tests, the planning simulator, a rosbag, or a real vehicle.
- For a change that affects driving behavior, the test covers the scenarios that the change affects.

### Code quality

- The code follows the [coding guidelines](../coding-guidelines/index.md).
- Names are clear, and comments explain why, not what.
- There is no dead code, debug output, or commented-out code.

### Documentation

- The README or design document of the package matches the new behavior, parameters, and interfaces.
- The parameter schema file (`schema/*.schema.json`) matches the parameter file, if the package has one.
- The documentation follows the [documentation guidelines](../documentation-guidelines/index.md).

### Rules for the pull request

- The title follows [Conventional Commits](index.md#apply-conventional-commits-to-the-pull-request-title-required-automated).
- Every commit is signed off. Refer to the [commit guidelines](commit-guidelines.md).
- If AI tools did a substantial part of the work, the description discloses it and includes a verification statement. Refer to the [AI contribution policy](../ai-contribution-policy.md).

## How to write review comments

- Explain the reason for each comment. The author must understand why you ask for a change.
- Separate requests from suggestions. For example, start a comment that does not block the merge with `nit:` or `suggestion:`. [Conventional Comments](https://conventionalcomments.org/) describes more labels.
- For a small change, use a [suggestion block](https://docs.github.com/en/pull-requests/how-tos/review-pull-requests/reviewing-proposed-changes-in-a-pull-request#starting-a-review), so that the author can [apply it from the pull request page](https://docs.github.com/en/pull-requests/how-tos/review-pull-requests/incorporating-feedback-in-your-pull-request#applying-suggested-changes). A commit made on the GitHub web page can fail the DCO check if it has no `Signed-off-by` line. In that case, the author must amend the commit and sign it off.
- Comment on the code, not on the person.
- Give all your comments in one review when you can, so that the author can fix them in one round.
- If a discussion in comments takes long, talk with the author in a call or in a discussion, then write the result in the pull request.

## Approve or request changes

When you submit a review, choose one of these options:

- **Comment**: You give feedback, but you do not decide.
- **Approve**: The change is ready to merge. You can approve with small non-blocking comments.
- **Request changes**: The change must not merge until the author addresses your comments.

After the author updates the pull request and asks for a new review, review the new commits.
If the author has addressed your comments, approve the pull request.

## Review in time

- Code owners review assigned pull requests promptly.
- If a pull request has no review for more than a week, the author can remind the reviewers. Refer to the [code owners FAQ](code-owners.md#faq).
- If you cannot review a pull request that is assigned to you, tell the author so that they can ask another reviewer.

## After the approval

Anyone with write access can merge the pull request when the merge requirements are met.
The author is encouraged to merge their own pull request.
If the author does not have write access, a reviewer or a maintainer merges it.

## Further reading

- [Google engineering practices: code review](https://google.github.io/eng-practices/review/)
- [GitLab code review guidelines](https://docs.gitlab.com/development/code_review/)
- [Swarmia: A complete guide to code reviews](https://www.swarmia.com/blog/a-complete-guide-to-code-reviews/)
- [Rewind: Best practices for reviewing pull requests in GitHub](https://rewind.com/blog/best-practices-for-reviewing-pull-requests-in-github/)
- [GitHub Docs: About pull request reviews](https://docs.github.com/en/pull-requests/reference/pull-request-reviews)
