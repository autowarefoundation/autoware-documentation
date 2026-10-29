// A page that the .nav.yml files leave out of the navigation has no active
// item in the sidebar, so the sidebar, the active tab and the breadcrumb are
// empty. Under MkDocs, the awesome-pages plugin kept such a page in the
// context of its parent section. This script restores that context: it marks
// the deepest section whose pages share a parent directory with this page.
document$.subscribe(() => {
  const primary = document.querySelector(".md-nav--primary");
  if (!primary || primary.querySelector(".md-nav__link--active")) return;

  const here = location.pathname;
  const pathOf = (link) => new URL(link.href, location.href).pathname;

  // The directory of a section is the common directory of all its pages.
  const directoryOf = (item) => {
    let common = null;
    for (const link of item.querySelectorAll("a.md-nav__link[href]")) {
      const parts = pathOf(link).split("/").slice(0, -1);
      if (common === null) {
        common = parts;
      } else {
        let i = 0;
        while (i < common.length && i < parts.length && common[i] === parts[i]) i++;
        common = common.slice(0, i);
      }
    }
    return common === null ? null : common.join("/") + "/";
  };

  let section = null;
  let sectionDirectory = "";
  for (const item of primary.querySelectorAll("li.md-nav__item--nested")) {
    const directory = directoryOf(item);
    if (
      directory &&
      directory.length > sectionDirectory.length &&
      here.startsWith(directory) &&
      here !== directory
    ) {
      section = item;
      sectionDirectory = directory;
    }
  }
  if (!section) return;

  // Expand and mark every section from the match up to the top level.
  const chain = [];
  for (
    let item = section;
    item && primary.contains(item);
    item = item.parentElement.closest("li.md-nav__item")
  ) {
    chain.unshift(item);
    item.classList.add("md-nav__item--active");
    const toggle = item.querySelector(":scope > input.md-nav__toggle");
    if (toggle) toggle.checked = true;
    const nav = item.querySelector(":scope > nav.md-nav");
    if (nav) nav.setAttribute("aria-expanded", "true");
  }

  // Mark the tab of the top-level section.
  const topLevel = Array.from(primary.querySelector(":scope > .md-nav__list").children);
  const tab = document.querySelectorAll(".md-tabs__item")[topLevel.indexOf(chain[0])];
  if (tab) tab.classList.add("md-tabs__item--active");

  // Show the sections as the breadcrumb. As in the theme, show it only if the
  // page has more than one parent, and count the hidden directories too.
  const hiddenDirectories = here.split("/").length - sectionDirectory.split("/").length;
  const content = document.querySelector(".md-content");
  if (chain.length + hiddenDirectories < 3 || !content || content.querySelector(".md-path")) return;
  const list = document.createElement("ol");
  list.className = "md-path__list";
  for (const item of chain) {
    const link = item.querySelector(":scope > .md-nav__container > a, a.md-nav__link[href]");
    const label = item.querySelector(
      ":scope > label .md-ellipsis, :scope > .md-nav__container .md-ellipsis",
    );
    if (!link || !label) continue;
    const entry = document.createElement("li");
    entry.className = "md-path__item";
    const anchor = document.createElement("a");
    anchor.className = "md-path__link";
    anchor.href = link.href;
    const text = document.createElement("span");
    text.className = "md-ellipsis";
    text.textContent = label.textContent.trim();
    anchor.append(text);
    entry.append(anchor);
    list.append(entry);
  }
  const path = document.createElement("nav");
  path.className = "md-path";
  path.setAttribute("aria-label", "Navigation");
  path.append(list);
  content.prepend(path);
});
