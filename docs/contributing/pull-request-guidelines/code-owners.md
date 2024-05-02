# Code owners

The Autoware project uses multiple `CODEOWNERS` files to specify owners throughout the repository. For a detailed understanding of code owners, visit the [GitHub documentation on code owners](https://docs.github.com/en/repositories/managing-your-repositorys-settings-and-features/customizing-your-repository/about-code-owners).

## Purpose and function of the `CODEOWNERS` file

The `CODEOWNERS` file plays a vital role in managing pull requests (PRs) by:

- **Automating Review Requests**: Automatically assigns PRs to responsible individuals or teams.
- **Enforcing Merge Approval**: Prevents PR merging without approvals from designated code owners or repository maintainers, ensuring thorough review.
- **Maintaining Quality Control**: Helps sustain code quality and consistency by requiring review from knowledgeable individuals or teams.

## Locating `CODEOWNERS` files

`CODEOWNERS` files are found in the `.github` directory across multiple repositories of the Autoware project. The [`autoware.repos` file](https://github.com/autowarefoundation/autoware/blob/main/autoware.repos) lists these repositories and their directories.

## Maintenance of `CODEOWNERS`

Generally, repository maintainers handle the updates to `CODEOWNERS` files. To propose changes, submit a PR to modify the file.

### Special case for the Autoware Universe repository

In the [autoware.universe](https://github.com/autowarefoundation/autoware.universe) repository, maintenance of the `CODEOWNERS` file is automated by the CI.

[This workflow](https://github.com/autowarefoundation/autoware.universe/actions/workflows/update-codeowners-from-packages.yaml) updates the `CODEOWNERS` file based on the `maintainer` information in the `package.xml` files of the packages in the repository.

In order to change the code owners for a package in the `autoware.universe` repository:

1. Modify the `maintainer` information in the `package.xml` file via a PR.
2. Once merged, the CI workflow runs at midnight UTC (or can be triggered manually by a maintainer) to update the `CODEOWNERS` file and create a PR.
3. A maintainer then needs to merge the CI-generated PR to finalize the update.
   - **Example Automated PR:** [chore: update CODEOWNERS #6866](https://github.com/autowarefoundation/autoware.universe/pull/6866)

## Responsibilities of code owners

Code owners should review assigned PRs promptly.
If a PR remains unreviewed for **over a week**, maintainers may intervene to review and possibly merge it.

## FAQ

### Unreviewed pull requests

If your PR hasn't been reviewed:

- 🏹 **Directly Address Code Owners**: Comment on the PR to alert the owners.
- ⏳ **Follow Up After a Week**: If unreviewed after a week, add a comment under the PR and tag the `@autoware-maintainers`.
- 📢 **Escalate if Necessary**: If your requests continue to go unanswered, you may escalate the issue by posting a message in the [Autoware Discord channel](../../support/support-guidelines.md#discord) 🚨. Remember, maintainers often juggle numerous responsibilities, so patience is appreciated🙇.

### PR author is the only code owner

If you, as the only code owner, authored a PR:

- Request a review by tagging `@autoware-maintainers`.
- The maintainers will consider appointing additional maintainers to avoid such conflicts.

### Non-code owners reviewing PRs

Anyone can review a PR:

- You can review any pull request and provide your feedback.
- Your review might not be enough to merge the pull request, but it will help the code owners and maintainers to make a decision.
- If you think the pull request is ready to merge, you can mention the code owners and maintainers in a comment on the pull request.
