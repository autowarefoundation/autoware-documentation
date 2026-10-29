// The highlighter of the code blocks reads a `ros2` command as plain text. This
// script colors its parts: the verb (for example `launch`), the package, the
// launch file or executable, topics and other names, message types, flags, and
// the `name:=value` arguments. It changes only blocks that contain `ros2`, and it
// keeps the spans that the highlighter made, for example `$HOME`.

const VERBS_WITH_SUBCOMMAND = new Set([
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
const LAUNCH_FILE = /\.(launch\.(xml|py|yaml)|xml|py|yaml)$/;
const MESSAGE_TYPE = /^[a-z0-9_]+\/(msg|srv|action)\/[A-Za-z0-9_]+$/;

// A console block without a prompt puts a whole line into one output span. Such
// a line that starts with `ros2` is a command, so take it apart like the
// highlighter of a Bash block does: words, spaces, and the `=` of `:=`.
function splitOutputLines(code) {
  for (const node of [...code.querySelectorAll(":scope > span.go")]) {
    if (!/^\s*ros2\s/.test(node.textContent)) continue;
    const parts = [];
    for (const token of node.textContent.split(/( +)/).filter(Boolean)) {
      if (/^ +$/.test(token)) {
        const space = document.createElement("span");
        space.className = "w";
        space.textContent = token;
        parts.push(space);
      } else if (token.includes(":=")) {
        const [name, ...value] = token.split(":=");
        const equals = document.createElement("span");
        equals.className = "o";
        equals.textContent = "=";
        parts.push(
          document.createTextNode(`${name}:`),
          equals,
          document.createTextNode(value.join(":=")),
        );
      } else {
        parts.push(document.createTextNode(token));
      }
    }
    node.replaceWith(...parts);
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

// Groups the nodes into words. A word ends at a space, a line break, or a line
// continuation. A line break without a continuation before it ends the command.
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
    // A prompt (`$ `) separates words like a space.
    const isSpace =
      node.nodeType === Node.ELEMENT_NODE &&
      (node.classList.contains("w") || node.classList.contains("gp"));
    const isContinuation = node.nodeType === Node.ELEMENT_NODE && node.classList.contains("se");
    if (node.nodeType === Node.TEXT_NODE && node.data === "\n") {
      end(!continued);
      continued = false;
    } else if (isContinuation) {
      end(false);
      continued = true;
    } else if (isSpace) {
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

function wrap(node, className) {
  if (node.nodeType === Node.ELEMENT_NODE) {
    node.classList.add(className);
    return;
  }
  const span = document.createElement("span");
  span.className = className;
  node.replaceWith(span);
  span.append(node);
}

// Colors `name:=value`: the name, the `:=`, and the value. The highlighter puts
// the `=` in its own span, so the `:` ends the text node of the name.
function colorArgument(word) {
  const index = word.nodes.findIndex(
    (node) =>
      node.nodeType === Node.TEXT_NODE &&
      node.data.endsWith(":") &&
      node.nextSibling?.textContent === "=",
  );
  if (index < 0) return false;
  const nameNode = word.nodes[index];
  const name = document.createTextNode(nameNode.data.slice(0, -1));
  const colon = document.createTextNode(":");
  nameNode.replaceWith(name, colon);
  word.nodes.slice(0, index).forEach((node) => wrap(node, "aw-ros-arg"));
  wrap(name, "aw-ros-arg");
  wrap(colon, "aw-ros-op");
  wrap(word.nodes[index + 1], "aw-ros-op");
  word.nodes.slice(index + 2).forEach((node) => wrap(node, "aw-ros-val"));
  return true;
}

function colorWord(word, className) {
  word.nodes.forEach((node) => wrap(node, className));
}

function highlight(code) {
  if (code.dataset.awRos || !code.textContent.includes("ros2")) return;
  code.dataset.awRos = "1";
  splitOutputLines(code);
  splitLines(code);
  let state = null;
  for (const word of words(code)) {
    if (word.newCommand) {
      state = null;
      continue;
    }
    const text = textOf(word);
    if (!state) {
      if (text === "ros2") {
        colorWord(word, "aw-ros-bin");
        state = { verb: null, subcommand: false, positional: 0 };
      }
      continue;
    }
    if (colorArgument(word)) continue;
    if (!state.verb) {
      state.verb = text;
      state.subcommand = VERBS_WITH_SUBCOMMAND.has(text);
      colorWord(word, "aw-ros-verb");
    } else if (state.subcommand) {
      state.subcommand = false;
      colorWord(word, "aw-ros-verb");
    } else if (text.startsWith("-")) {
      colorWord(word, "aw-ros-flag");
    } else if (MESSAGE_TYPE.test(text)) {
      colorWord(word, "aw-ros-type");
    } else if (text.startsWith("/")) {
      colorWord(word, "aw-ros-name");
    } else if ((state.verb === "launch" || state.verb === "run") && state.positional < 2) {
      const isFile = state.verb === "launch" ? LAUNCH_FILE.test(text) : state.positional === 1;
      colorWord(word, isFile ? "aw-ros-file" : "aw-ros-pkg");
      state.positional += isFile && state.positional === 0 ? 2 : 1;
    }
  }
}

document$.subscribe(() => {
  document.querySelectorAll(".md-typeset .highlight code").forEach(highlight);
});
