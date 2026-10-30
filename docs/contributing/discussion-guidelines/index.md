# Discussion guidelines

Autoware uses [GitHub Discussions](https://github.com/orgs/autowarefoundation/discussions) for questions, ideas, feature requests, and design topics. This page explains how to start a discussion and how to take part in one. For the list of all support channels, refer to the [support guidelines](../../community/support/support-guidelines.md).

## Choose the right place

Use GitHub Discussions for topics that are not yet confirmed bugs or tasks. Use GitHub Issues only for confirmed bugs and confirmed tasks. If an issue is a question or an unconfirmed bug, the maintainers can move it to Discussions.

The discussions of the Autoware Foundation organization have these categories:

| Category                                                                                                               | Use it for                                  | Can have an answer |
| :--------------------------------------------------------------------------------------------------------------------- | :------------------------------------------ | :----------------- |
| [Q&A](https://github.com/orgs/autowarefoundation/discussions/categories/q-a)                                           | Questions and unconfirmed bugs              | Yes                |
| [Feature requests](https://github.com/orgs/autowarefoundation/discussions/categories/feature-requests)                 | Requests for new features and improvements  | Yes                |
| [Ideas](https://github.com/orgs/autowarefoundation/discussions/categories/ideas)                                       | Ideas for new features                      | Yes                |
| [Design](https://github.com/orgs/autowarefoundation/discussions/categories/design)                                     | The system and software design of Autoware  | No                 |
| [General](https://github.com/orgs/autowarefoundation/discussions/categories/general)                                   | Other topics about Autoware                 | No                 |
| [Show and tell](https://github.com/orgs/autowarefoundation/discussions/categories/show-and-tell)                       | Projects that you made with Autoware        | No                 |
| [Polls](https://github.com/orgs/autowarefoundation/discussions/categories/polls)                                       | Votes from the community                    | No                 |
| [Announcements](https://github.com/orgs/autowarefoundation/discussions/categories/announcements)                       | Updates from the maintainers                | No                 |
| [TSC meetings](https://github.com/orgs/autowarefoundation/discussions/categories/tsc-meetings)                         | Minutes of the Technical Steering Committee | No                 |
| [Working group activities](https://github.com/orgs/autowarefoundation/discussions/categories/working-group-activities) | Activities of the working groups            | No                 |
| [Working group meetings](https://github.com/orgs/autowarefoundation/discussions/categories/working-group-meetings)     | Minutes of the working group meetings       | No                 |

## Before you start a discussion

1. Search the [documentation](../../home/index.md) and the [troubleshooting pages](../../community/support/troubleshooting/index.md) for your problem.
2. Search the existing [discussions](https://github.com/orgs/autowarefoundation/discussions) and the issues of the related repositories. The [`autoware.repos` file](https://github.com/autowarefoundation/autoware/blob/main/repositories/autoware.repos) lists the repositories.
3. If you find a discussion about the same topic, add your information to it. Do not start a new one.

## Start a discussion

The General category uses a form. The form asks you to confirm that you read the [Code of Conduct](https://github.com/autowarefoundation/autoware/blob/main/CODE_OF_CONDUCT.md) and the support guidelines. Then you write your post in the `Contents` field. The other categories have a plain text box.

Write a post that other people can answer without asking for more information:

- Write a title that tells the problem or the proposal in one line. For example, "EKF localizer diverges after the vehicle stops" is better than "Help with localization".
- Write what you want to do, what you did, what you expected, and what happened.
- Give the version of Autoware (for example `2.0.0` or the commit of `main`), the ROS 2 distribution, the operating system, and how you installed Autoware (source or Docker).
- Give the steps to reproduce the problem. Use the smallest setup that shows it, for example the planning simulator with the sample map.
- Put commands, logs, and error messages in [code blocks](https://docs.github.com/en/get-started/writing-on-github/working-with-advanced-formatting/creating-and-highlighting-code-blocks). Do not post screenshots of text.
- Add screenshots or videos only for things that text cannot show, for example a view in RViz.
- Write one topic per discussion. If you have two unrelated questions, start two discussions.

For a feature request or an idea, also write:

- The problem that the feature solves and who has this problem.
- The solution that you propose, and the other solutions that you considered.
- Whether you plan to implement it yourself.

## Take part in a discussion

- Follow the [Code of Conduct](https://github.com/autowarefoundation/autoware/blob/main/CODE_OF_CONDUCT.md). Be respectful, also when you disagree.
- Reply in the thread of the comment that you answer, so that the conversation stays easy to follow.
- Give reasons and evidence for your opinion, for example logs, measurements, or links to code.
- Write in English, so that the whole community can read and search your post. You can use machine translation for text that you wrote yourself.
- Follow the [AI contribution policy](../ai-contribution-policy.md). Your own words carry the conversation, and you must mark AI-generated material as such.
- Do not mention many people only to get attention. Mention the people or teams that can help with the topic.

## Close a discussion

- In Q&A, Feature requests, and Ideas, [mark the comment that answers your question as the answer](https://docs.github.com/en/discussions/collaborating-with-your-community-using-discussions/participating-in-a-discussion#marking-a-comment-as-an-answer). This helps other people who have the same problem.
- If you found the solution yourself, post it in the discussion and mark it as the answer.
- If you get no reply within a week, mention `@autoware-maintainers` in a comment to remind them.

## From a discussion to an issue and a pull request

A discussion is the first step of a larger change:

1. A discussion confirms a bug, or the community and the maintainers agree on a proposal.
2. A maintainer or the author [creates an issue from the discussion](https://docs.github.com/en/issues/tracking-your-work-with-issues/using-issues/creating-an-issue#creating-an-issue-from-discussion) in the related repository.
3. A contributor makes a pull request that refers to the issue. Refer to the [pull request guidelines](../pull-request-guidelines/index.md).

For more details on this process, refer to [How can I contribute?](../index.md#how-can-i-contribute).

## Further reading

- [GitHub Docs: Best practices for community conversations on GitHub](https://docs.github.com/en/discussions/guides/best-practices-for-community-conversations-on-github)
- [Open Source Guides: Communicating effectively](https://opensource.guide/how-to-contribute/#communicating-effectively)
