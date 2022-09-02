# Pull request guidelines

## General pull request workflow

Autoware uses the fork-and-pull model.
For the detailed explanation of the model, refer to [GitHub Docs](https://docs.github.com/en/pull-requests/collaborating-with-pull-requests).

The following is a general example of the pull request workflow.

1. Create a fork. (for the first time only)
2. Create an issue.
   - Discuss the approach to the issue with maintainers.
3. Write code in the agreed approach.
   - Write the tests and documentation if necessary.
   - Follow the guidelines when you write code, tests, and documentation.
     - [Coding guidelines](../coding-guidelines/index.md)
     - [Testing guidelines](../testing-guidelines/index.md)
     - [Documentation guidelines](../documentation-guidelines/index.md)
   - Follow the [Commit guidelines](commit-guidelines.md) when commit your change.
4. Test the code.
   - In the later review process, you will need to write what tests you have done.
5. Create a pull request.
   - Read the pull request template carefully and fill the required content.
   - Assign reviewers after creating a pull request.
     - If you have no rights to assign reviewers, just mention them instead.
     - If you are not sure who to set reviewers, contact to the people written as the `maintainer` tag in `package.xml`.
6. Wait for the pull request to be reviewed.
   - The reviewers will review your code following the [review guidelines](review-guidelines.md).
     - Take a look at the guidelines as well because it is good to understand the thoughts of the reviewer's side.
   - If [CI checks](ci-checks) have failed, fix the errors.
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
