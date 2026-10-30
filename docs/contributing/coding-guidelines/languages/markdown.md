# Markdown

This page describes the rules for Markdown files in Autoware, such as the `README.md` of a package and the pages of this documentation site. pre-commit checks Markdown files with markdownlint and formats them with Prettier. A CI workflow checks the spelling with CSpell.

For the writing style of the text, see the [documentation guidelines](../../documentation-guidelines/index.md).

## References

- [markdownlint rules](https://github.com/DavidAnson/markdownlint/blob/main/doc/Rules.md)
- The Markdown section of the ROS 2 code style: <https://docs.ros.org/en/humble/The-ROS2-Project/Contributing/Code-Style-Language-Versions.html#markdown-restructured-text-docblocks>

## markdownlint

pre-commit runs [markdownlint](https://github.com/DavidAnson/markdownlint) through [markdownlint-cli](https://github.com/igorshubovych/markdownlint-cli) with the `--fix` option. The tool fixes many problems automatically. The configuration is the [`.markdownlint.yaml`](https://github.com/autowarefoundation/sync-file-templates/blob/main/sources/.markdownlint.yaml) file at the root of each repository. It turns on all rules, with these changes:

| Rule    | Setting          | Effect                                                              |
| ------- | ---------------- | ------------------------------------------------------------------- |
| `MD013` | Off              | Lines can have any length.                                          |
| `MD024` | `siblings_only`  | Two headings can have the same text if they have different parents. |
| `MD029` | `style: ordered` | Number ordered lists `1.`, `2.`, `3.`, not `1.`, `1.`, `1.`.        |
| `MD033` | Off              | Inline HTML is allowed.                                             |
| `MD041` | Off              | The first line does not have to be a top-level heading.             |
| `MD045` | Off              | Images do not need alternative text.                                |
| `MD046` | Off              | Code blocks can be fenced or indented.                              |
| `MD049` | Off              | Emphasis can use `*` or `_`.                                        |
| `MD059` | Off              | Link text is not checked. Still, use descriptive link text.         |

The rules that stay on include these:

- Use one heading style in a file. Autoware uses ATX headings (`# Heading`).
- Increase the heading level one step at a time.
- Put a blank line before and after headings, lists, and code blocks.
- Give each fenced code block a language, for example `bash`, `cpp`, or `yaml`. Use `txt` for plain text.
- Do not leave trailing spaces or more than one blank line in a row.

## Prettier

pre-commit formats Markdown, YAML, JSON, and other supported files with [Prettier](https://prettier.io/). The configuration is the [`.prettierrc.yaml`](https://github.com/autowarefoundation/sync-file-templates/blob/main/sources/.prettierrc.yaml) file at the root of each repository. Prettier aligns the columns of tables, sets the indentation of nested lists, and uses `-` for list items. It does not wrap the lines of a paragraph, so it keeps your line breaks.

## Spell check

The `spell-check-differential` workflow checks the spelling of the changed files with [CSpell](https://cspell.org/) and the shared [autoware-spell-check-dict](https://github.com/autowarefoundation/autoware-spell-check-dict). If you must use a word that the dictionary does not know, add it to a comment at the top of the file:

```markdown
<!-- cspell:ignore Wpedantic -->
```

For the other options, see [CI checks](../../pull-request-guidelines/ci-checks.md#spell-check-differential).

## Links

The optional pre-commit configuration `.pre-commit-config-optional.yaml` checks the links in Markdown files with [markdown-link-check](https://github.com/tcort/markdown-link-check). To run it, use this command:

```bash
pre-commit run -a --config .pre-commit-config-optional.yaml
```

Use relative links to other Markdown files in the same repository, for example `[CMake](cmake.md)`, so that the links also work on GitHub.
