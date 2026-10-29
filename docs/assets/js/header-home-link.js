// The site name in the header links to the homepage, as the logo next to it
// does. The theme renders the site name as plain text, so this script wraps it
// in a copy of the logo link.

document$.subscribe(() => {
  const logo = document.querySelector(".md-header__button.md-logo");
  const name = document.querySelector(".md-header__topic:first-child > .md-ellipsis");
  if (!logo || !name || name.closest("a")) return;

  const link = document.createElement("a");
  link.href = logo.href;
  link.className = "aw-header-home";
  name.replaceWith(link);
  link.append(name);
});
