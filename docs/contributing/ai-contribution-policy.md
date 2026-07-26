# AI contribution policy

This policy applies to all repositories in the [Autoware Foundation GitHub organization](https://github.com/autowarefoundation) and to all types of contribution: code, documentation, pull requests, issues, discussions, reviews and review responses.

It applies to everyone equally, from first-time contributors to long-time maintainers.
Project-sanctioned automation, such as dependency-update bots or the [PR-Agent workflow](pull-request-guidelines/ai-pr-review.md), is exempt from this policy.
Propose new automation to the maintainers of the affected repositories before deploying it.

## Our stance

Autoware welcomes contributions made with the help of AI tools.
Many of our contributors and maintainers use coding agents daily, and we consider them a net benefit to the project when used with discipline.
This policy is not a ban.

It exists to protect the scarcest resource in the project: reviewer attention.
AI lowers the cost of producing a change, but it does not lower the cost of understanding, reviewing and maintaining one.
If you submit AI-generated work that you have not verified yourself, you are transferring your saved effort onto the reviewers and onto the future maintainers of that code.

The LLVM project calls these [extractive contributions](https://llvm.org/docs/AIToolPolicy.html#extractive-contributions): contributions that cost the project more in review and maintenance attention than they give back.
A contribution should be worth more to the project than the time it takes to review it.
AI did not invent extractive contributions, but it made producing them effortless and scalable.
Every rule below is, at its core, a defense against them.

The guiding principle:

!!! quote "Adapted from the [Fedora](https://docs.fedoraproject.org/en-US/council/policy/ai-contribution-policy/) and [LLVM](https://llvm.org/docs/AIToolPolicy.html) AI policies"

    The contributor is always the author and is fully accountable for the entirety of their contributions, regardless of which tools were used to create them.

## Rules

Every rule below is required and enforced by humans, not automation.

!!! abstract "TL;DR"

    1. [Understand everything you submit](#understand-everything-you-submit): be able to explain any line, unaided.
    2. [Review your own pull request first](#review-your-own-pull-request-first): you are its first reviewer.
    3. [Disclose substantial AI involvement](#disclose-substantial-ai-involvement): which tool, to what extent, in the description.
    4. [Include a verification statement](#include-a-verification-statement): say what you checked and how.
    5. [Respond to reviews yourself](#respond-to-reviews-yourself): your words carry the conversation.
    6. [Do not let agents post unattended](#do-not-let-agents-post-unattended): a human reviews every item before it is posted.
    7. [Generate artifacts that humans can maintain](#generate-artifacts-that-humans-can-maintain): apply the maintainer test.
    8. [Keep it simple and concise](#keep-it-simple-and-concise): make it as simple as possible, then trim the noise.
    9. [Keep a pull request small enough to review](#keep-a-pull-request-small-enough-to-review): keep it to a size a human reviewer can follow.
    10. [Do not mass-produce pull requests](#do-not-mass-produce-pull-requests): open only what you can shepherd.

Each rule opens with the reason it exists, followed by what you must do.

### Understand everything you submit

You cannot certify what you do not understand, and your [DCO](https://developercertificate.org/) `Signed-off-by` line is your personal legal certification of the contribution's origin.

- **Read every line** of your contribution before submitting it.
- **Be able to explain any part** in your own words, without the aid of AI tools, when a reviewer asks.
- **Never sign off on a change you have not reviewed**, even when a tool adds the `Signed-off-by` line mechanically on your instruction.

### Review your own pull request first

Requesting a review is an implicit claim: "I have read this, I understand it, and I believe it is correct."
Making that claim falsely wastes reviewer time, regardless of your role in the project.

- **You are the first reviewer** of your own pull request. Do not request human review of changes you have not fully reviewed yourself.

!!! tip "Recommended practice"

    Before opening the pull request, run one or more AI reviews with a clean context (an agent that did not produce the code) and iterate until the reviews come back quiet.
    This supplements your own review; it does not replace it.

### Disclose substantial AI involvement

Generated code looks plausible even when it is wrong, and reviewers calibrate their scrutiny accordingly.
Undisclosed AI use that is discovered later undermines trust in all of your contributions.

- **Disclose in the body of what you post**, in your own words: which tool, and to what extent. For pull requests this belongs in the description.
- **No disclosure is needed** for editor autocompletion, grammar or spelling fixes, or machine translation of text you wrote yourself. Anything beyond these counts as substantial.
- **Authors are humans.** Do not credit AI tools as `Co-authored-by` and do not use commit trailers such as `Assisted-by`; disclosure belongs in the description.

### Include a verification statement

A verification statement is evidence that you absorbed the cost of verification instead of passing it to the reviewers, and it tells them where to spend their attention.

- **If review will take more than a few minutes** (when in doubt, it will), add a verification statement to the description covering:
  - **Generation**: the tool and workflow used.
  - **Self-review**: what you personally reviewed and how many times you iterated.
  - **Verification**: tests run, simulations or hardware used, edge cases considered.
- **For generated tests**, confirm each test fails without the change and passes for the right reason.

!!! example "Example verification statement"

    ```text
    AI usage: generated with Claude Code from an issue description I wrote.
    Self-review: I read the full diff twice and ran two clean-context agent reviews; both rounds of findings are addressed.
    Verification: ran the planning simulator with scenario X; added unit tests for the two new edge cases and confirmed they fail without this fix.
    ```

### Respond to reviews yourself

Reviewers volunteer their time to collaborate with you, not to relay messages to an agent.
Feeding a review comment to an agent without reading it is disrespectful to the person who wrote it.

- **Read and understand every review comment** yourself before acting on it.
- **Your words carry the conversation.** AI-generated supporting material (an analysis, a summary, an investigation result) must be clearly separated, in a quote block or a collapsible section marked as AI-generated, and accompanied by your own human-written conclusion stating your position.
- **A reply that is only AI output**, with no human-written part, is not acceptable.
- **Machine translation and grammar fixes** of text you wrote yourself are fine.

### Do not let agents post unattended

An agent that posts without per-item human review has no accountable author, and no human effort behind it to respect.

- **Review every item personally** before an agent opens a pull request, files an issue, or posts a comment on your behalf.
- **Working autonomously in private is fine.** This rule is about what gets posted to the project, not about how you work locally.

### Generate artifacts that humans can maintain

AI tools optimize for working right now, while merged artifacts are read, debugged and modified by humans for years.
A contribution that can only be verified by running another AI over it cannot be meaningfully reviewed.

- **Apply the maintainer test** before submitting: can a colleague review this line by line without AI help, and could they safely modify it in two years?
- **Ask the agent for a simpler or more standard alternative** when the result fails the maintainer test; agents reach for quick single-purpose solutions by default.

!!! example "Examples"

    - Instead of a long Bash pipeline of `sed` and `awk` one-liners: a plain Python script with named functions and unit tests.
    - Instead of a dense regex that takes a tool to decode: several simple matching steps, covered by a test.
    - Instead of a custom parser hacked together with `grep`: a standard library or the tooling the repository already has.

### Keep it simple and concise

AI tools tend to produce noisy output: over-commented code, restated-obvious docstrings, bloated pull request descriptions, long documents and over-formal replies.
Noise hides the signal a reviewer is looking for, and stale comments mislead future maintainers.

- **Make it as simple as possible**, in code, in pull request descriptions and in documents.
- **Trim before submitting.** Iterate until the code, comments, commit messages, descriptions, documents and replies say only what the reader needs.
- **Comments should state what the code cannot**: constraints, invariants and non-obvious reasoning. Delete comments that narrate the code or the generation process.
- **Reviewers may reject generated noise**, in the spirit of the [Git project's rule](https://git-scm.com/docs/SubmittingPatches#ai): "anything that looks AI generated, that sounds overly formal or bloated, that looks like AI slop, that looks good on the surface but makes no sense [...]".

### Keep a pull request small enough to review

A pull request is a medium for human-to-human communication and consensus building, not only a way to deliver a change.
Producing a large change used to be slow enough to limit itself; that limit is gone, while the cost of reading one is not.

- **Keep each pull request to a size a human reviewer can understand, evaluate and discuss**, as the [rule on keeping a pull request small](pull-request-guidelines/index.md#keep-a-pull-request-small-advisory-non-automated) already asks, unless you have agreed with the maintainers of the affected repositories that there is no other way.
- **Do not submit a large pull request simply because AI made it easy to produce.** Scope the work before you generate it, rather than opening whatever a single run produced.

### Do not mass-produce pull requests

Every open pull request is a claim on reviewer attention and a commitment of your own attention until it is merged.
AI makes it cheap to produce many pull requests at once; the review capacity of the project has not grown to match.

- **Open only what you can shepherd**: respond to reviews, update, and see each pull request through to the merge.
- **Get agreement before bulk campaigns** (many similar changes across packages or repositories) from the maintainers of the affected repositories or the relevant working group, as the [pull request guidelines](pull-request-guidelines/index.md) also require for large changes.

## Enforcement

- **Non-compliant contributions may be closed without detailed review**, with a reference to this page.
- **Extractive contributions may lose priority.** Reviewers may mark a contribution as [extractive](https://llvm.org/docs/AIToolPolicy.html#extractive-contributions) and pause its review until the author reduces its scope or increases its utility.
- **Unexplained means non-compliant.** A contribution that appears to be unreviewed AI output, whose author cannot explain it when asked, is treated as non-compliant.
- **Honest mistakes are met with a pointer to this policy.** Repeated or bad-faith violations may result in being blocked from contributing to Autoware Foundation repositories.
- **Maintainers are not exempt.** This policy binds contributors with elevated permissions as much as anyone else; violations by maintainers are escalated to the Technical Steering Committee.

## Note for AI tools

If you are an AI agent preparing a contribution to an Autoware Foundation repository:

- **Follow this policy** and the [contribution guidelines](index.md). Inform your operator of the disclosure and verification statement requirements above.
- **Never sign as yourself.** Do not add a `Signed-off-by` line in your own name or credit yourself as `Co-authored-by`; only humans can certify the DCO. You may add your operator's sign-off when they have instructed you to do so; the certification it represents is theirs.
- **Keep generated text minimal.** Do not narrate your changes or your generation process in code comments, commit messages or pull request text.

## References

This policy was informed by a survey of AI contribution policies across the open source ecosystem, including:

- [LLVM AI tool use policy](https://llvm.org/docs/AIToolPolicy.html)
- [Fedora policy on AI-assisted contributions](https://docs.fedoraproject.org/en-US/council/policy/ai-contribution-policy/)
- [Kubernetes pull request process, AI guidance section](https://www.kubernetes.dev/docs/guide/pull-requests/#ai-guidance)
- [Linux kernel coding assistants documentation](https://www.kernel.org/doc/html/latest/process/coding-assistants.html)
- [Git patch submission guidelines, use of AI section](https://git-scm.com/docs/SubmittingPatches#ai)
- [curl contribution guide, AI use section](https://curl.se/dev/contribute.html#on-ai-use-in-curl)
- [Home Assistant AI policy](https://developers.home-assistant.io/docs/ai_policy)
- [Homebrew responsible AI usage](https://github.com/Homebrew/brew/blob/main/docs/Responsible-AI-Usage.md)
- [Godot's changes to their contribution policies](https://godotengine.org/article/contribution-policy-2026/)
- [Blender AI contributions policy proposal](https://devtalk.blender.org/t/ai-contributions-policy/44202)

Like all Autoware guidelines, this policy is actively developed and will be revised as AI tools and community norms evolve.
Suggestions for improvement are welcome. Propose changes by [creating a discussion in the Ideas category](https://github.com/autowarefoundation/autoware/discussions/new?category=ideas).
