// The highlighter of the code blocks colors only a few parts of a shell command,
// for example `cd`, strings, and variables. This script colors the rest: the
// program, its subcommands, flags, `NAME=value` and `name:=value` arguments,
// packages, files, URLs, placeholders such as `<YOUR-DIR>`, and the operators
// that join commands or redirect them. A `ros2` command also gets its launch
// file or executable, its topics and other names, and its message types. The
// script changes only shell blocks, and it keeps the spans that the highlighter
// made for strings and variables, for example `$HOME`.

// cspell:ignore nohup

const SHELL_BLOCKS = [
  ".language-bash",
  ".language-console",
  ".language-sh",
  ".language-shell",
  ".language-zsh",
]
  .map((language) => `.md-typeset ${language}.highlight code`)
  .join(", ");

const ROS2_VERBS_WITH_SUBCOMMAND = new Set([
  "action",
  "bag",
  "component",
  "daemon",
  "interface",
  "lifecycle",
  "node",
  "param",
  "pkg",
  "service",
  "topic",
]);
// The programs that take a subcommand. The set holds the subcommands that take
// one more subcommand, for example `git remote add`. `*` means all of them.
const SUBCOMMANDS = new Map([
  ["ansible-galaxy", new Set(["*"])],
  ["apt", new Set()],
  ["apt-get", new Set()],
  ["apt-mark", new Set()],
  ["aw-index-cli", new Set()],
  ["aws", new Set(["*"])],
  ["catkin", new Set()],
  ["colcon", new Set()],
  ["conda", new Set()],
  ["devcontainer", new Set()],
  ["docker", new Set(["buildx", "compose", "container", "image", "network", "volume"])],
  ["gh", new Set(["*"])],
  ["git", new Set(["lfs", "remote", "stash", "submodule", "worktree"])],
  ["ip", new Set(["*"])],
  ["make", new Set()],
  ["npm", new Set()],
  ["nvidia-ctk", new Set(["*"])],
  ["pip", new Set()],
  ["pip3", new Set()],
  ["pipx", new Set()],
  ["pre-commit", new Set()],
  ["ros2", ROS2_VERBS_WITH_SUBCOMMAND],
  ["rosbag", new Set()],
  ["rosdep", new Set()],
  ["rosparam", new Set()],
  ["rosservice", new Set()],
  ["rostopic", new Set()],
  ["snap", new Set()],
  ["systemctl", new Set()],
  ["vcs", new Set()],
  ["zensical", new Set()],
]);
// The programs that run the command that follows them.
const WRAPPERS = new Set(["env", "exec", "nohup", "sudo", "time"]);
const ROS_PROGRAMS = new Set([
  "ros2",
  "rosbag",
  "roslaunch",
  "rosparam",
  "rosrun",
  "rosservice",
  "rostopic",
]);
// The flags whose values are package names, and other flags that take a value.
const PACKAGE_FLAGS = new Set([
  "--packages-ignore",
  "--packages-select",
  "--packages-skip",
  "--packages-up-to",
]);
const VALUE_FLAGS = new Set([
  "--base-paths",
  "--event-handlers",
  "--from-paths",
  "--rosdistro",
  "--skip-tags",
  "--tags",
]);
// Shell keywords after which a command starts, and keywords that start a line
// that is not a simple command.
const KEYWORDS_BEFORE_COMMAND = new Set([
  "!",
  "do",
  "elif",
  "else",
  "if",
  "then",
  "until",
  "while",
]);
const KEYWORDS_WITHOUT_COMMAND = new Set([
  "case",
  "done",
  "esac",
  "fi",
  "for",
  "function",
  "select",
]);
const SEPARATORS = new Set(["&&", ";", "|", "||"]);
// Highlighter tokens whose color stays: comments, strings, and variables.
const KEPT_TOKENS = ["c", "c1", "nv", "s", "s1", "s2", "sb", "sd", "se", "sh", "si", "sx"];

const COMMAND = /^(?:~\/|\.{1,2}\/)?[A-Za-z_][\w.+-]*(?:\/[\w.+-]+)*$/;
// A word of a sentence, for example `Change` in `Change line 383 in ...`.
const PROSE_WORD = /^[A-Z][a-z]+$/;
// `NAME=value`, but not `x==1.2` or `author={...}` of a BibTeX entry.
const ASSIGNMENT = /^[A-Za-z_]\w*=(?![={])/;
const REDIRECT = /^(?:\d*>>?|&>|<<?<?|\d*>&\d*)$/;
const URL_SCHEME = /^(?:https?|ftp|file|git|s3|ssh):\/\//;
const PLACEHOLDER = /<[A-Za-z][^<>\s]*>/g;
const LAUNCH_FILE = /\.(launch(\.(xml|py|yaml))?|xml|py|yaml)$/;
const FILE =
  /\.(bash|bashrc|cfg|conf|db3|deb|ini|json|launch|mcap|md|msg|osm|pcd|pth|py|repos|rules|service|sh|srv|tar(\.\w+)?|toml|txt|xml|ya?ml|zip)$/;
const MESSAGE_TYPE = /^[a-z0-9_]+\/(msg|srv|action)\/[A-Za-z0-9_]+$/;

const isElement = (node, ...classNames) =>
  node.nodeType === Node.ELEMENT_NODE &&
  classNames.some((className) => node.classList.contains(className));

// A console block without a prompt puts each line into one output span. Such a
// block holds only commands, so take each line apart like the highlighter of a
// Bash block does: comments, words, and spaces. A console block with prompts
// keeps its output lines, but a line that starts with `ros2` is a command.
function splitOutputLines(code) {
  const hasPrompt = code.querySelector(":scope > span.gp") !== null;
  for (const node of [...code.querySelectorAll(":scope > span.go")]) {
    if (hasPrompt && !/^\s*ros2\s/.test(node.textContent)) continue;
    if (node.textContent.trimStart().startsWith("#")) {
      node.className = "c1";
      continue;
    }
    const parts = node.textContent
      .split(/( +)/)
      .filter(Boolean)
      .map((token) => {
        if (!/^ +$/.test(token)) return document.createTextNode(token);
        const space = document.createElement("span");
        space.className = "w";
        space.textContent = token;
        return space;
      });
    node.replaceWith(...parts);
  }
}

// The highlighter of a console block reads `# ` as the prompt of the root user.
// In a block that also has `$ ` prompts, such a line is a comment.
function commentLines(code) {
  const prompts = [...code.querySelectorAll(":scope > span.gp")];
  if (!prompts.some((node) => node.textContent.trim().endsWith("$"))) return;
  for (const prompt of prompts.filter((node) => node.textContent.trim() === "#")) {
    const comment = document.createElement("span");
    comment.className = "c1";
    prompt.before(comment);
    while (comment.nextSibling && comment.nextSibling.textContent !== "\n")
      comment.append(comment.nextSibling);
    comment.textContent = comment.textContent;
  }
}

// Splits the text nodes at line breaks, so that each word lies in its own nodes.
function splitLines(code) {
  for (const node of [...code.childNodes]) {
    if (node.nodeType !== Node.TEXT_NODE || !node.data.includes("\n")) continue;
    const parts = node.data.split(/(\n)/).filter(Boolean);
    node.replaceWith(...parts.map((part) => document.createTextNode(part)));
  }
}

// Groups the nodes into words. A word ends at a space, a line break, a line
// continuation, or an operator that joins two commands, for example `&&`. A line
// break without a continuation before it ends the command.
function words(code) {
  const result = [];
  let current = [];
  let continued = false;
  const end = (newCommand) => {
    if (current.length) result.push({ nodes: current, newCommand: false });
    current = [];
    if (newCommand) result.push({ nodes: [], newCommand: true });
  };
  for (const node of code.childNodes) {
    if (node.nodeType === Node.TEXT_NODE && node.data === "\n") {
      end(!continued);
      continued = false;
    } else if (isElement(node, "se") && node.textContent.startsWith("\\")) {
      end(false);
      continued = true;
    } else if (isElement(node, "w", "gp")) {
      // A prompt (`$ `) separates words like a space.
      end(false);
    } else if (isElement(node, "o", "p") && SEPARATORS.has(node.textContent)) {
      end(false);
      result.push({ nodes: [node], newCommand: false });
      end(false);
    } else {
      current.push(node);
      continued = false;
    }
  }
  end(true);
  return result;
}

const textOf = (word) => word.nodes.map((node) => node.textContent).join("");

function splitNode(node, index) {
  if (node.nodeType === Node.TEXT_NODE) return node.splitText(index);
  const tail = node.cloneNode(false);
  tail.textContent = node.textContent.slice(index);
  node.textContent = node.textContent.slice(0, index);
  node.after(tail);
  return tail;
}

// Splits the nodes of a word at the character offsets, and returns the nodes of
// each part. The offsets must be in ascending order.
function splitWord(word, offsets) {
  const parts = offsets.map(() => []).concat([[]]);
  let part = 0;
  let start = 0;
  for (let node of word.nodes) {
    for (;;) {
      while (part < offsets.length && offsets[part] <= start) part++;
      const length = node.textContent.length;
      const cut = offsets[part] - start;
      if (!(cut < length)) {
        parts[part].push(node);
        start += length;
        break;
      }
      const tail = splitNode(node, cut);
      parts[part].push(node);
      start += cut;
      node = tail;
    }
  }
  return parts.filter((nodes) => nodes.length);
}

function wrap(node, className, keep) {
  if (node.nodeType === Node.ELEMENT_NODE) {
    if (!keep || !KEPT_TOKENS.some((token) => node.classList.contains(token)))
      node.classList.add(className);
    return;
  }
  const span = document.createElement("span");
  span.className = className;
  node.replaceWith(span);
  span.append(node);
}

const paint = (nodes, className, keep = false) =>
  nodes.forEach((node) => wrap(node, className, keep));

// Colors a word. A placeholder in it, for example `<YOUR-DIR>`, gets its own
// color. The word keeps the color of its strings and variables.
function colorWord(word, className) {
  const text = textOf(word);
  const offsets = [...text.matchAll(PLACEHOLDER)].flatMap((match) => [
    match.index,
    match.index + match[0].length,
  ]);
  for (const nodes of splitWord(word, offsets)) {
    const part = nodes.map((node) => node.textContent).join("");
    if (className) paint(nodes, className, true);
    if (/^<[A-Za-z][^<>\s]*>$/.test(part)) paint(nodes, "aw-sh-ph");
  }
}

// Colors `name=value` or `name:=value`: the name, the operator, and the value.
function colorAssignment(word, text, index, length, nameClass = "aw-sh-arg") {
  const [name, operator, value] = splitWord(word, [index, index + length]);
  paint(name, nameClass);
  paint(operator, "aw-sh-op");
  if (value) paint(value, "aw-sh-val");
}

// Starts a command at the word, and returns the state of the command, or `null`
// if the word does not start one.
function startCommand(word, text, next) {
  if (word.nodes.some((node) => isElement(node, "c", "c1", "go"))) return { skip: true };
  if (isElement(word.nodes[0], "k") || KEYWORDS_BEFORE_COMMAND.has(text)) {
    return KEYWORDS_WITHOUT_COMMAND.has(text) ? { skip: true } : null;
  }
  if (KEYWORDS_WITHOUT_COMMAND.has(text)) return { skip: true };
  if (ASSIGNMENT.test(text)) {
    colorAssignment(word, text, text.indexOf("="), 1);
    return null;
  }
  // Output of a command, for example `Bag size: 12.4 GiB`, a sentence, or a
  // `title = {...}` line of a BibTeX entry is not a command.
  const nextText = next && !next.newCommand ? textOf(next) : "";
  const isOutput = nextText.endsWith(":") && !URL_SCHEME.test(nextText);
  if (!COMMAND.test(text) || PROSE_WORD.test(text) || isOutput || nextText.startsWith("="))
    return { skip: true };
  if (WRAPPERS.has(text)) {
    paint(word.nodes, "aw-sh-sudo");
    return { wrapper: true };
  }
  paint(word.nodes, "aw-sh-cmd");
  return newState(text);
}

function newState(program) {
  return {
    program,
    subcommands: SUBCOMMANDS.get(program) ?? null,
    verb: program === "roslaunch" ? "launch" : program === "rosrun" ? "run" : null,
    positional: 0,
    values: null,
  };
}

function colorArgument(word, text, state) {
  if (REDIRECT.test(text)) {
    paint(word.nodes, "aw-sh-sep");
    state.redirect = /^(?:\d*>>?|<)$/.test(text);
    return;
  }
  if (state.redirect) {
    state.redirect = false;
    colorWord(word, "aw-sh-file");
    return;
  }
  const index = text.indexOf(":=");
  if (index > 0) {
    colorAssignment(word, text, index, 2);
    return;
  }
  if (text.length > 1 && text.startsWith("-") && !/^-\d/.test(text)) {
    const equals = text.indexOf("=");
    const flag = equals > 1 ? text.slice(0, equals) : text;
    if (equals > 1) colorAssignment(word, text, equals, 1, "aw-sh-flag");
    else paint(word.nodes, "aw-sh-flag");
    state.values =
      equals > 1 ? null : PACKAGE_FLAGS.has(flag) ? "pkg" : VALUE_FLAGS.has(flag) ? "val" : null;
    state.module = /^python\d*$/.test(state.program) && flag === "-m";
    return;
  }
  if (state.module) {
    // `python3 -m pip install`: the module acts as the program.
    paint(word.nodes, "aw-sh-sub");
    Object.assign(state, newState(text), { module: false });
    return;
  }
  if (state.values) {
    colorWord(word, `aw-sh-${state.values}`);
    if (state.values === "val") state.values = null;
    return;
  }
  if (state.subcommands) {
    const nested = state.subcommands.has("*") || state.subcommands.has(text);
    paint(word.nodes, "aw-sh-sub");
    state.subcommands = nested ? new Set() : null;
    if (state.program === "ros2" && !state.verb) state.verb = text;
    return;
  }
  colorPositional(word, text, state);
}

function colorPositional(word, text, state) {
  if (MESSAGE_TYPE.test(text)) {
    colorWord(word, "aw-sh-type");
  } else if (URL_SCHEME.test(text)) {
    colorWord(word, "aw-sh-url");
  } else if (ROS_PROGRAMS.has(state.program) && text.startsWith("/")) {
    colorWord(word, "aw-sh-name");
  } else if ((state.verb === "launch" || state.verb === "run") && state.positional < 2) {
    const isFile = state.verb === "launch" ? LAUNCH_FILE.test(text) : state.positional === 1;
    colorWord(word, isFile ? "aw-sh-file" : "aw-sh-pkg");
    state.positional += isFile && state.positional === 0 ? 2 : 1;
  } else if (state.program === "ansible-playbook" && state.positional++ === 0) {
    colorWord(word, "aw-sh-pkg");
  } else if (ASSIGNMENT.test(text)) {
    colorAssignment(word, text, text.indexOf("="), 1);
  } else {
    colorWord(word, FILE.test(text) ? "aw-sh-file" : null);
  }
}

function highlight(code) {
  // A block that holds an interface definition has its colors already (see
  // interface-highlight.js).
  if (code.dataset.awShell || code.dataset.awInterface) return;
  code.dataset.awShell = "1";
  // A directory tree is not a list of commands.
  if (/[├└]──/.test(code.textContent)) return;
  splitOutputLines(code);
  splitLines(code);
  commentLines(code);
  const list = words(code);
  // A Bash block can show a prompt (`$ `) before each command, and the output
  // of the commands on the other lines.
  const hasPrompts = !code.querySelector(":scope > span.gp") && /^\s*\$ /m.test(code.textContent);
  let state = null;
  let lineStart = true;
  list.forEach((word, index) => {
    if (word.newCommand) {
      state = null;
      lineStart = true;
      return;
    }
    if (state?.skip) return;
    const text = textOf(word);
    if (lineStart && hasPrompts) {
      lineStart = false;
      if (text === "$") paint(word.nodes, "gp");
      else state = { skip: true };
      return;
    }
    lineStart = false;
    if (SEPARATORS.has(text)) {
      paint(word.nodes, "aw-sh-sep");
      state = null;
      return;
    }
    // A number inside a word, for example the `2` of `2.0.0`, takes the color
    // of the rest of the word.
    if (word.nodes.length > 1)
      word.nodes.forEach((node) => isElement(node, "m") && node.classList.remove("m"));
    if (!state) {
      state = startCommand(word, text, list[index + 1]);
    } else if (state.wrapper) {
      if (text.startsWith("-")) paint(word.nodes, "aw-sh-flag");
      else state = startCommand(word, text, list[index + 1]);
    } else {
      colorArgument(word, text, state);
    }
  });
}

document$.subscribe(() => {
  document.querySelectorAll(SHELL_BLOCKS).forEach(highlight);
});
