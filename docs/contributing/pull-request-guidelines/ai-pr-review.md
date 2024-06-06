# AI PR Review

We have [Codium-ai/pr-agent](https://github.com/Codium-ai/pr-agent/tree/main) enabled for Autoware Universe repository.

## The workflow

Workflow: [pr-agent.yaml](https://github.com/autowarefoundation/autoware.universe/blob/main/.github/workflows/pr-agent.yaml)

### Additional links for the workflow maintainers

- [Available models list](https://github.com/Codium-ai/pr-agent/blob/main/pr_agent/algo/__init__.py)

## How to use

When you create the PR, or within the PR add the label `tag:pr-agent`.

Wait until both PR-Agent jobs are completed successfully:

- `prevent-no-label-execution-pr-agent / prevent-no-label-execution`
- `Run pr agent on every pull request, respond to user comments`

!!! warning

    If you add multiple labels at the same time, `prevent-no-label-execution` can get confused.
    For example, first add `tag:pr-agent`, wait until it is ready, then add `tag:run-build-and-test-differential` if you need it.

Then you can pick one of the following commands:

```text
/review: Request a review of your Pull Request.
/describe: Update the PR description based on the contents of the PR.
/improve: Suggest code improvements.
/ask Could you propose a better name for this parameter?: Ask a question about the PR or anything really.
/update_changelog: Update the changelog based on the PR's contents.
/add_docs: Generate docstring for new components introduced in the PR.
/help: Get a list of all available PR-Agent tools and their descriptions.
```

- [Here is the official documentation](https://pr-agent-docs.codium.ai/tools/).
- [Usage Guide](https://pr-agent-docs.codium.ai/usage-guide/automations_and_usage/#online-usage)

To use it, [drop a comment post within your PR like this](https://github.com/Codium-ai/pr-agent/pull/229#issuecomment-1695021901).

Within a minute, you should see 👀 reaction under your comment post.

Then the bot will drop a response with reviews, description or an answer.

!!! info

    Please drop a single PR-Agent related comment at a time.
