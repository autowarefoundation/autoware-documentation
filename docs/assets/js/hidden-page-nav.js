// A page that the .nav.yml files leave out of the navigation has no active
// item in the sidebar, so the sidebar, the active tab and the breadcrumb are
// empty. Under MkDocs, the awesome-pages plugin kept such a page in the
// context of its parent section. This script restores that context: it marks
// the deepest section whose pages share a parent directory with this page.
//
// The theme renders only the visible part of the sidebar (navigation.prune),
// so the sidebar of a hidden page has no sections to mark. The script borrows
// the sidebar of a nearby page that is in the navigation: a parent directory,
// or the previous or next page, which is often in the same directory.

const here = () => location.pathname;

// The directory of a section is the common directory of all its pages.
function directoryOf(item) {
  let common = null;
  for (const link of item.querySelectorAll("a.md-nav__link[href]")) {
    const parts = new URL(link.href, location.href).pathname.split("/").slice(0, -1);
    if (common === null) {
      common = parts;
    } else {
      let i = 0;
      while (i < common.length && i < parts.length && common[i] === parts[i]) i++;
      common = common.slice(0, i);
    }
  }
  return common === null ? null : common.join("/") + "/";
}

// Return the deepest section of a sidebar list that contains this page.
function findSection(list) {
  let section = null;
  let directory = "";
  for (const item of list.querySelectorAll("li.md-nav__item--nested")) {
    const candidate = directoryOf(item);
    if (candidate && candidate.length > directory.length && here().startsWith(candidate)) {
      section = item;
      directory = candidate;
    }
  }
  return { section, directory };
}

// Return the sidebar list of a page that is in the navigation, with absolute
// links, or null.
async function fetchSidebar(address) {
  let response;
  try {
    response = await fetch(address);
  } catch {
    return null;
  }
  if (!response.ok) return null;
  const page = new DOMParser().parseFromString(await response.text(), "text/html");
  const list = page.querySelector(".md-nav--primary > .md-nav__list");
  if (!list || !list.querySelector(".md-nav__link--active")) return null;
  for (const link of list.querySelectorAll("a[href]")) {
    link.setAttribute("href", new URL(link.getAttribute("href"), response.url).href);
  }
  for (const link of list.querySelectorAll(".md-nav__link--active")) {
    link.classList.remove("md-nav__link--active");
  }
  return list;
}

// Borrow the nearby sidebar that places this page the deepest. On a tie, the
// sidebar of a parent page wins, because it shows the sections expanded.
async function borrowSidebar(primary) {
  const addresses = [];
  const parts = here().split("/").filter(Boolean);
  // The first directory is the version, whose own page only redirects.
  for (let length = parts.length - 1; length > 1; length--) {
    addresses.push(new URL(`/${parts.slice(0, length).join("/")}/`, location.href).href);
  }
  for (const link of document.querySelectorAll(".md-footer__link[href]")) {
    addresses.push(new URL(link.getAttribute("href"), location.href).href);
  }
  const lists = await Promise.all(addresses.map(fetchSidebar));
  let best = null;
  let bestDirectory = "";
  for (const list of lists) {
    if (!list) continue;
    const { section, directory } = findSection(list);
    if (section && directory.length > bestDirectory.length) {
      best = list;
      bestDirectory = directory;
    }
  }
  if (best) primary.querySelector(":scope > .md-nav__list").replaceWith(document.adoptNode(best));
}

document$.subscribe(async () => {
  const primary = document.querySelector(".md-nav--primary");
  if (!primary || primary.querySelector(".md-nav__link--active")) return;
  await borrowSidebar(primary);

  const list = primary.querySelector(":scope > .md-nav__list");
  const { section, directory: sectionDirectory } = findSection(list);
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
  const topLevel = Array.from(list.children);
  const tab = document.querySelectorAll(".md-tabs__item")[topLevel.indexOf(chain[0])];
  if (tab) tab.classList.add("md-tabs__item--active");

  // Show the sections as the breadcrumb, followed by the hidden directories
  // between the section and the page. A hidden directory is named as MkDocs
  // named it, so "fail_safe" becomes "Fail safe". As in the theme, the
  // breadcrumb is shown only if it has more than one entry.
  const hidden = here().slice(sectionDirectory.length).split("/").filter(Boolean).slice(0, -1);
  const content = document.querySelector(".md-content");
  if (chain.length + hidden.length < 2 || !content || content.querySelector(".md-path")) return;
  const path = document.createElement("ol");
  path.className = "md-path__list";
  const addEntry = (title, address) => {
    const entry = document.createElement("li");
    entry.className = "md-path__item";
    const text = document.createElement("span");
    text.className = "md-ellipsis";
    text.textContent = title;
    if (address) {
      const anchor = document.createElement("a");
      anchor.className = "md-path__link";
      anchor.href = address;
      anchor.append(text);
      entry.append(anchor);
    } else {
      entry.append(text);
    }
    path.append(entry);
  };
  for (const item of chain) {
    const link = item.querySelector(":scope > .md-nav__container > a, a.md-nav__link[href]");
    const label = item.querySelector(
      ":scope > label .md-ellipsis, :scope > .md-nav__container .md-ellipsis, :scope > a .md-ellipsis",
    );
    if (link && label) addEntry(label.textContent.trim(), link.href);
  }
  for (const directory of hidden) {
    const title = decodeURIComponent(directory).replace(/[-_]/g, " ");
    addEntry(title.charAt(0).toUpperCase() + title.slice(1), null);
  }
  const nav = document.createElement("nav");
  nav.className = "md-path";
  nav.setAttribute("aria-label", "Navigation");
  nav.append(path);
  content.prepend(nav);
});
