# Tips

This page introduces tips around pull requests or reviews.

## Toggle annotations or review comments in the diff view

There will be some annotations or review comments in the diff view during your review.

Press the `A` key to toggle annotations.

Before:

![before-press-a](images/before-press-a.png)

After:

![after-press-a](images/after-press-a.png)

Also, press the `I` key to toggle review comments.

Refer to [GitHub Docs](https://docs.github.com/en/get-started/using-github/keyboard-shortcuts) for other keyboard shortcuts.

## View code in the web-based Visual Studio Code

You can open `Visual Studio Code` from your browser to view code in a rich UI.
Press the `.` key on any repository or pull request.

Refer to [github/dev](https://github.com/github/dev) for more detailed usage.

## Check out the branch of a pull request quickly

If you want to check out the branch of a pull request, it's generally troublesome with the fork-and-pull model.

```bash
# Copy the user name and the fork URL.
git remote add {user-name} {fork-url}
git checkout {user-name}/{branch-name}
git remote rm {user-name} # To clean up
```

Instead, you can use [GitHub CLI](https://cli.github.com/) to simplify the steps, just run `gh pr checkout {pr-number}`.

You can copy the command from the top right of the pull request page.

![gh-pr-checkout](images/gh-pr-checkout.png)
