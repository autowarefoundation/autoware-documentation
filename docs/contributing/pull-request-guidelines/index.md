# Pull request guidelines

## General pull request workflow

Autoware uses the fork-and-pull model.
For more details about the model, refer to [GitHub Docs](https://docs.github.com/en/pull-requests/collaborating-with-pull-requests).

The following is a general example of the pull request workflow based on the fork-and-pull model.
Use this workflow as a reference when you contribute to Autoware.

1. Create an issue.
   - Discuss the approaches to the issue with maintainers.
   - Confirm the [support guidelines](../../support/support-guidelines.md) before creating an issue.
   - Follow the [discussion guidelines](../discussion-guidelines/index.md) when you discuss with other contributors.
2. Create a fork repository. (for the first time only)
3. Write code in your fork repository according to the approach agreed upon in the issue.
   - Write the tests and documentation as appropriate.
   - Follow the [coding guidelines](../coding-guidelines/index.md) guidelines when you write code.
   - Follow the [Testing guidelines](../testing-guidelines/index.md) guidelines when you write tests.
   - Follow the [Documentation guidelines](../documentation-guidelines/index.md) guidelines when you write documentation.
   - Follow the [commit guidelines](commit-guidelines.md) when you commit your changes.
4. Test the code.
   - It is recommended that you summarize the test results, because you will need to explain the test results in the later review process.
   - If you are not sure what tests should be done, discuss them with maintainers.
5. Create a pull request.
   - Follow the [pull request rules](#pull-request-rules) when you create a pull request.
6. Wait for the pull request to be reviewed.
   - The reviewers will review your code following the [review guidelines](review-guidelines.md).
     - Not only the reviewers, but also the author is encouraged to understand the review guidelines.
   - If [CI checks](ci-checks.md) have failed, fix the errors.
7. Address the review comments pointed out by the reviewers.
   - If you don't understand the meaning of a review comment, ask the reviewers until you understand it.
     - Fixing without understanding the reason is not recommended because the author should be responsible for the final content of their own pull request.
   - If you don't agree with a review comment, ask the reviewers for a rational reason.
     - The reviewers are obligated to make the author understand the meanings of each comment.
   - After you have done with the review comments, re-request a review to the reviewers and back to 6.
   - If there are no more new review comments, the reviewers will approve the pull request and proceed to 8.
8. Merge the pull request.
   - Anyone with write access can merge the pull request if there is no special request from maintainers.
     - The author is encouraged to merge the pull request to feel responsible for their own pull request.
     - If the author does not have write access, ask the reviewers or maintainers.

## Pull request rules

### Use an appropriate pull request template (required, non-automated)

#### Rationale

- The unified style of descriptions by templates can make reviews efficient.

#### Example

- There are two types of template. Select one based on the following condition.

1. Standard change:

- Complexity:
  - Moderate to high, including new features, architectural changes, or addressing multiple issues.
  - Requires deeper understanding of the codebase and collaboration.
- Impact:
  - Broader system impact, affecting multiple components, functionality, or performance.
  - Includes minor feature additions and minor bug fixes.
  - Requires thorough testing and evaluation before merging.

2. Small change:

- Complexity:
  - Low, such as documentation updates, simple refactoring, or code style adjustments.
  - Easy to understand and review without extensive collaboration or deep knowledge of the codebase.
- Impact:
  - Limited or localized system impact, affecting specific components or having minimal implications on functionality or performance.
  - Excludes minor feature additions and minor bug fixes.
  - Can be merged quickly with less rigorous testing.

##### Steps to use an appropriate pull request template

1. Select the appropriate template, as shown in [this video](https://user-images.githubusercontent.com/31987104/184344710-2adee239-799f-4fdf-bfab-be76345bfac1.mp4).
2. Read the selected template carefully and fill the required content.
3. Check the checkboxes during a review.
   - There are [pre-review checklist](https://github.com/autowarefoundation/autoware/blob/44c70d33825617b56d8de5e6fe921000238238bd/.github/PULL_REQUEST_TEMPLATE/standard-change.md#pre-review-checklist-for-the-pr-author) and [post-review checklist](https://github.com/autowarefoundation/autoware/blob/44c70d33825617b56d8de5e6fe921000238238bd/.github/PULL_REQUEST_TEMPLATE/standard-change.md#post-review-checklist-for-the-pr-author) for the author.

### Set appropriate reviewers after creating a pull request (required, partially automated)

#### Rationale

- Pull requests must be reviewed by appropriate reviewers to keep the quality of the codebase.

#### Example

- For most ROS packages, reviewers will be automatically assigned based on the `maintainer` information in `package.xml`.
- If no reviewer is assigned automatically, assign reviewers manually following the instructions in [GitHub Docs](https://docs.github.com/en/pull-requests/collaborating-with-pull-requests/proposing-changes-to-your-work-with-pull-requests/requesting-a-pull-request-review).
  - You can find the reviewers by seeing the `.github/CODEOWNERS` file of the repository.
- If you are not sure the appropriate reviewers, ask `@autoware-maintainers`.
- If you have no rights to assign reviewers, mention reviewers instead.

### Apply Conventional Commits to the pull request title (required, automated)

#### Rationale

- [Conventional Commits](https://www.conventionalcommits.org/en/v1.0.0/) can generate categorized changelogs, for example using [git-cliff](https://github.com/orhun/git-cliff).

#### Example

```text
feat(trajectory_follower): add an awesome feature
```

!!! note

    You have to start the description part (here `add an awesome feature`) with a lowercase.

If your change breaks some interfaces, use the `!` (breaking changes) mark as follows:

```text
feat(trajectory_follower)!: remove package
feat(trajectory_follower)!: change parameter names
feat(planning)!: change topic names
feat(autoware_utils)!: change function names
```

For the repositories that contain code (most repositories), use the [definition of conventional-commit-types](https://github.com/commitizen/conventional-commit-types/blob/c3a9be4c73e47f2e8197de775f41d981701407fb/index.json) for the type.

For documentation repositories such as [autoware-documentation](https://github.com/autowarefoundation/autoware-documentation), use the following definition:

- `feat`
  - Add new pages.
  - Add contents to the existing pages.
- `fix`
  - Fix the contents in the existing pages.
- `refactor`
  - Move contents to different pages.
- `docs`
  - Update documentation for the documentation repository itself.
- `build`
  - Update the settings of the documentation site builder.
- `!` (breaking changes)
  - Remove pages.
  - Change the URL of pages.

`perf` and `test` are generally unused.
Other types have the same meaning as the code repositories.

### Add the related component names to the scope of Conventional Commits (advisory, non-automated)

#### Rationale

- It helps contributors find pull requests that are relevant to them.
- It makes the changelog clearer.

#### Example

For ROS packages, adding the package name or component name is good.

```text
feat(trajectory_follower): add an awesome feature
refactor(planning, control): use common utils
```

### Keep a pull request small (advisory, non-automated)

#### Rationale

- Small pull requests are easy to understand for reviewers.
- Small pull requests are easy to revert for maintainers.

#### Exception

It is acceptable if it is agreed with maintainers that there is no other way but to submit a big pull request.

#### Example

- Avoid developing two features in one pull request.
- Avoid mixing different types (`feat`, `fix`, `refactor`, etc.) of changes in the same commit.

### Remind reviewers if there is no response for more than a week (advisory, non-automated)

#### Rationale

- It is the author's responsibility to care about their own pull request until it is merged.

#### Example

```text
@{some-of-developers} Would it be possible for you to review this PR?
@autoware-maintainers friendly ping.
```
