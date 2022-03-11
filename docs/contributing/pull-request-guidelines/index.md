# Pull request guidelines

## General pull request workflow

Autoware uses the fork-and-pull model.
Refer to [GitHub Docs](https://docs.github.com/en/pull-requests/collaborating-with-pull-requests) for the detailed explanation of the model.

The following is a general example of our pull request workflow.

1. Create a fork. (for the first time only)
2. Create an issue.
   - Discuss the approach to the issue with maintainers.
3. Write code in the agreed approach.
   - Write the tests and documentation if necessary.
   - Follow our guidelines when you write code, tests, and documentation.
     - [Coding guidelines](../coding-guidelines/index.md)
     - [Testing guidelines](../testing-guidelines/index.md)
     - [Documentation guidelines](../documentation-guidelines/index.md)
   - Follow our [Commit guidelines](commit-guidelines.md) when commit your change.
4. Test the code.
   - In the later review process, you will need to write what tests you have done.
5. Create a pull request.
   - Read the pull request template carefully and fill the required content.
   - Assign reviewers after creating a pull request.
     - If you have no rights to assign reviewers, just mention them instead.
     - If you are not sure who to set reviewers, contact to the people written as the `maintainer` tag in `package.xml`.
6. Wait for the pull request to be reviewed.
   - The reviewers will review your code following our [review guidelines](review-guidelines.md).
     - Take a look at the guidelines as well because it is good to understand the thoughts of the reviewer's side.
   - If [CI checks](#ci-checks) have failed, fix the errors.
7. Address the review comments pointed out by the reviewers.
   - If you cannot understand or agree with a review comment, discuss it with the reviewers and find a rational reason.
     - The author should be responsible for the final content of their pull request.
     - The reviewers are obligated to make the author understand the meanings of each comment.
   - After you have done with them, re-request a review to the reviewers and back to 6.
   - If there are no more new review comments, the reviewers will approve the pull request and go to 6.
8. Merge the pull request.
   - Confirm all items of the checklist are checked off before merging.
   - Anyone with write access can merge the pull request if there is no special request from maintainers.
     - Generally, the author is expected to merge it to feel responsible for their pull request.
     - If the author does not have write access, ask the reviewers or maintainers.
     - It is the author's responsibility to care about their own pull request until it is merged.

## CI checks

Autoware has several checks for a pull request.
The results are shown at the bottom of the pull request page as below.

![ci-checks](images/ci-checks.png)

If the ❌ mark is shown, click the `Details` button and investigate the failure reason.

If the `Required` mark is shown, you cannot merge the pull request unless you resolve the error.
If not, it is optional, but preferably it should be fixed.

The following sections explain about common CI checks in Autoware.  
Note that some repositories may have different settings.

### DCO

The Developer Certificate of Origin (DCO) is a lightweight way for contributors to certify that they wrote or otherwise have the right to submit the code they are contributing to the project.

This workflow checks whether the pull request fulfills `DCO`.  
You need to confirm the [required items](https://developercertificate.org/) and commit with `git commit -s`.

Refer to the [GitHub App page](https://github.com/apps/dco) for more information.

### semantic-pull-request

Autoware uses [Conventional Commits](https://www.conventionalcommits.org/en/v1.0.0/) with the settings of [commitizen/conventional-commit-types](https://github.com/commitizen/conventional-commit-types).

This workflow checks whether the pull request follows `Conventional Commits`.

Note that if there is only one commit in the pull request, you need to make the commit title semantic as well.  
This is due to GitHub's non-intuitive behavior when there is only one commit.
GitHub uses the commit title for the squashed commit message, instead of the pull request title.

Refer to [amannn/action-semantic-pull-request](https://github.com/amannn/action-semantic-pull-request) for more detailed behaviors of this workflow.

See our [commit guidelines](commit-guidelines.md) for the detailed rules.

### pre-commit

[pre-commit](https://pre-commit.com/) is a tool to run formatters or linters when you commit.

This workflow checks whether the pull request has no error with `pre-commit`.

In the workflow `pre-commit.ci - pr` is enabled in the repository, it will automatically fix errors by [pre-commit.ci](https://pre-commit.ci/) as many as possible.  
If there are some errors remain, fix them manually.

You can run `pre-commit` in your local environment by the following command:

```bash
pre-commit run -a
```

Or you can install `pre-commit` to the repository and automatically run it before committing:

```bash
pre-commit install
```

Since it is difficult to detect errors with no false positives, some jobs are split into another config file and marked as optional.  
To check them, use the `--config` option:

```bash
pre-commit run -a --config .pre-commit-config-optional.yaml
```

### spell-check-differential

This workflow detects spelling mistakes using [CSpell](https://github.com/streetsidesoftware/cspell) with [our dictionary file](https://github.com/tier4/autoware-spell-check-dict/blob/main/.cspell.json).  
You can submit pull requests to [tier4/autoware-spell-check-dict](https://github.com/tier4/autoware-spell-check-dict) to update the dictionary.

Since it is difficult to detect errors with no false positives, it is an optional workflow, but it is preferable to remove spelling mistakes as many as possible.

### build-and-test-differential

This workflow checks `colcon build` and `colcon test` for the pull request.  
To make the CI faster, it doesn't check all packages but only modified packages and the dependencies.

### build-and-test-differential-self-hosted

This workflow is the `ARM64` version of `build-and-test-differential`.  
You need to add the `ARM64` label to run this workflow.

For reference information, since ARM machines are not supported by GitHub-hosted runners, we use self-hosted runners prepared by the AWF.  
Refer to [GitHub Docs](https://docs.github.com/en/actions/hosting-your-own-runners/about-self-hosted-runners) for the details about self-hosted runners.

### deploy-docs

This workflow deploys the preview documentation site for the pull request.  
You need to add the `documentation` label to run this workflow.
