# Support guidelines

This page explains the support mechanisms we provide.

!!! warning

    Before asking for help, search and read this documentation site carefully.
    Also, follow the [discussion guidelines](../contributing/discussion-guidelines/index.md) for discussions.

Choose appropriate resources depending on what kind of help you need and read the detailed description in the sections below.

- [Documentation sites](#documentation-sites)
  - Gathering information
- [GitHub Discussions](#github-discussions)
  - Questions or unconfirmed bugs -> [Q&A](https://github.com/orgs/autowarefoundation/discussions/categories/q-a)
  - [Feature requests](https://github.com/orgs/autowarefoundation/discussions/categories/feature-requests)
  - [Design discussions](https://github.com/orgs/autowarefoundation/discussions/categories/design)
- [GitHub Issues](#github-issues)
  - Confirmed bugs
  - Confirmed tasks
- [Discord](#discord)
  - Instant messaging between contributors
- [ROS Discourse](#ros-discourse)
  - General topics that should be widely announced

## Guidelines for Autoware community support

If you encounter a problem with Autoware, please follow these steps to seek help:

### 1. Search for existing Issues and Questions

Before creating a new issue or question, check if someone else has already reported or asked about the problem. Use the following resources:

- **[Issues](https://github.com/autowarefoundation/autoware/issues)**

  Note that Autoware has multiple repositories listed in [autoware.repos](https://github.com/autowarefoundation/autoware/blob/main/autoware.repos).
  It is recommended to search across all repositories.

- **[Questions](https://github.com/autowarefoundation/autoware/discussions/categories/q-a)**

### 2. Create a new question thread

If you don't find an existing issue or question that addresses your problem, create a new question thread:

- **[Ask a Question](https://github.com/autowarefoundation/autoware/discussions/categories/q-a)**

  If your question is not answered within a week, mention `@autoware-maintainers` in a post to remind them.

### 3. Participate in other discussions

You are also welcome to open or join discussions in other categories:

- **[Feature requests](https://github.com/autowarefoundation/autoware/discussions/categories/feature-requests)**
- **[Design discussions](https://github.com/autowarefoundation/autoware/discussions/categories/design)**

### Additional resources

If you are unsure how to create a discussion, refer to the [GitHub Docs on creating a new discussion](https://docs.github.com/en/discussions/quickstart#creating-a-new-discussion).

## Documentation sites

[Docs guide](docs-guide.md) shows the list of useful documentation sites.
Visit them and see if there is any information related to your problem.

Note that the documentation sites aren't always up-to-date and perfect.
If you find out that some information is wrong, unclear, or missing in Autoware docs, feel free to submit a pull request following the [contribution guidelines](../contributing/index.md).

## GitHub Discussions

[GitHub discussions page](https://github.com/orgs/autowarefoundation/discussions) is the primary place for asking questions and discussing topics related to Autoware.

| Category                                                                                                               | Description                                                             |
| :--------------------------------------------------------------------------------------------------------------------- | :---------------------------------------------------------------------- |
| [Announcements](https://github.com/orgs/autowarefoundation/discussions/categories/announcements)                       | Official updates and news from the Autoware maintainers                 |
| [Design](https://github.com/orgs/autowarefoundation/discussions/categories/design)                                     | Discussions on Autoware system and software design                      |
| [Feature requests](https://github.com/orgs/autowarefoundation/discussions/categories/feature-requests)                 | Suggestions for new features and improvements                           |
| [General](https://github.com/orgs/autowarefoundation/discussions/categories/general)                                   | General discussions about Autoware                                      |
| [Ideas](https://github.com/orgs/autowarefoundation/discussions/categories/ideas)                                       | Brainstorming and sharing innovative ideas                              |
| [Polls](https://github.com/orgs/autowarefoundation/discussions/categories/polls)                                       | Community polls and surveys                                             |
| [Q&A](https://github.com/orgs/autowarefoundation/discussions/categories/q-a)                                           | Questions and answers from the community and developers                 |
| [Show and tell](https://github.com/orgs/autowarefoundation/discussions/categories/show-and-tell)                       | Showcase of projects and achievements                                   |
| [TSC meetings](https://github.com/orgs/autowarefoundation/discussions/categories/tsc-meetings)                         | Minutes and discussions from TSC(Technical Steering Committee) meetings |
| [Working group activities](https://github.com/orgs/autowarefoundation/discussions/categories/working-group-activities) | Updates on working group activities                                     |
| [Working group meetings](https://github.com/orgs/autowarefoundation/discussions/categories/working-group-meetings)     | Minutes and discussions from working group meetings                     |

!!! warning

    GitHub Discussions is not the right place to track tasks or bugs. Use GitHub Issues for that purpose.

## GitHub Issues

GitHub Issues is the designated platform for tracking confirmed bugs, tasks, and enhancements within Autoware's various repositories.

Follow these guidelines to ensure efficient issue tracking and resolution:

### Reporting bugs

If you encounter a confirmed bug, please report it by creating an issue in the appropriate Autoware repository.
Include detailed information such as steps to reproduce, expected outcomes, and actual results to assist maintainers in addressing the issue promptly.

### Tracking tasks

GitHub Issues is also the place for managing tasks including:

- **Refactoring:** Propose refactoring existing code to improve efficiency, readability, or maintainability. Clearly describe what and why you propose to refactor.
- **New Features:** If you have confirmed the need for a new feature through discussions, use Issues to track its development. Outline the feature's purpose, potential designs, and its intended impact.
- **Documentation:** Propose changes to documentation to fix inaccuracies, update outdated content, or add new sections. Specify what changes are needed and why they are important.

### Creating an issue

When creating a new issue, use the following guidelines:

1. **Choose the Correct Repository**: If unsure which repository is appropriate, start a discussion in the [Q&A category](https://github.com/autowarefoundation/autoware/discussions/categories/q-a) to seek guidance from maintainers.
2. **Use Clear, Concise Titles**: Clearly summarize the issue or task in the title for quick identification.
3. **Provide Detailed Descriptions**: Include all necessary details to understand the context and scope of the issue. Attach screenshots, error logs, and code snippets where applicable.
4. **Tag Relevant Contributors**: Mention contributors or teams that might be impacted by or interested in the issue.

### Linking issues and pull requests

When you start working on an issue, link the related pull request to the issue by mentioning the issue number.
This helps maintain a clear and traceable development history.

!!! warning

    GitHub Issues is not for questions or unconfirmed bugs. If an issue is created for such purposes,
    it will likely be transferred to GitHub Discussions for further clarification.

## Discord

[![Discord](https://img.shields.io/discord/953808765935816715?label=Join%20Autoware%20Discord&style=for-the-badge)](https://discord.gg/Q94UsPvReQ)

Autoware has a Discord server for casual communication between contributors.

The Autoware Discord server is a good place for the following activities:

- Introduce yourself to the community.
- Chat with contributors.
- Take a quick straw poll.

Note that it is not the right place to get help for your issues.

## ROS Discourse

If you want to widely discuss a topic with the general Autoware and ROS community or ask a question not related to Autoware's bugs, post to [the Autoware category on ROS Discourse](https://discourse.ros.org/c/autoware).

!!! warning

    Do not post questions about bugs to ROS Discourse!
