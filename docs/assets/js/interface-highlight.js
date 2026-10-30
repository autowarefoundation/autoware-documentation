// The highlighter of the code blocks has no language for ROS interface
// definitions (`.msg`, `.srv`, and `.action` files), so it shows them as plain
// text. This script colors such a block: the comments, the field types, the
// field and constant names, the values, and the `---` lines. It changes a text
// or shell block only if each line of it is a field, a constant, a comment, or a
// `---` line.

const INTERFACE_BLOCKS = [".language-bash", ".language-sh", ".language-text", ".language-txt"]
  .map((language) => `.md-typeset ${language}.highlight code`)
  .join(", ");

const PRIMITIVE_TYPES = new Set([
  "bool",
  "byte",
  "char",
  "duration",
  "float32",
  "float64",
  "int8",
  "int16",
  "int32",
  "int64",
  "string",
  "time",
  "uint8",
  "uint16",
  "uint32",
  "uint64",
  "wstring",
]);
// A field or a constant: the type with an optional bound and array suffix, the
// name, an optional value, and an optional comment.
const FIELD =
  /^(\s*)([A-Za-z_]\w*(?:\/[A-Za-z_]\w*){0,2})((?:<=\d+)?(?:\[(?:<=)?\d*\])?)(\s+)([A-Za-z_]\w*)(?:(\s*=\s*|\s+)([^#\s][^#]*?))?(\s*)(#.*)?$/;
const OTHER_LINE = /^\s*(#.*|---\s*)?$/;

function interfaceSpan(className, text) {
  const node = document.createElement("span");
  node.className = className;
  node.textContent = text;
  return node;
}

function interfaceLine(text) {
  const field = FIELD.exec(text);
  if (!field) {
    const trimmed = text.trim();
    if (!trimmed) return [text];
    const className = trimmed.startsWith("#") ? "c1" : "aw-sh-sep";
    return [text.slice(0, text.indexOf(trimmed)), interfaceSpan(className, trimmed)];
  }
  const [, indent, type, suffix, space, name, operator, value, gap, comment] = field;
  const parts = [indent, interfaceSpan(PRIMITIVE_TYPES.has(type) ? "kt" : "aw-sh-type", type)];
  if (suffix) parts.push(interfaceSpan("p", suffix));
  const isConstant = operator?.includes("=");
  parts.push(space, interfaceSpan(isConstant ? "no" : "n", name));
  if (value) {
    parts.push(isConstant ? interfaceSpan("o", operator) : operator);
    parts.push(interfaceSpan(/^["']/.test(value) ? "s2" : "m", value));
  }
  parts.push(gap);
  if (comment) parts.push(interfaceSpan("c1", comment));
  return parts;
}

// A field type that cannot be a command: a primitive type, a type with its
// package, for example `std_msgs/Header`, or a type of the same package, for
// example `RadarReturn`.
const isKnownType = (text) => {
  const type = FIELD.exec(text)?.[2];
  return (
    type !== undefined && (PRIMITIVE_TYPES.has(type) || type.includes("/") || /^[A-Z]/.test(type))
  );
};

function highlightInterface(code) {
  if (code.dataset.awInterface) return;
  const lines = code.textContent.replace(/\n$/, "").split("\n");
  const fields = lines.filter((text) => FIELD.test(text));
  // A shell block holds commands such as `rosdep update`, so it must use only
  // known types. A text block can also use types of its own package.
  const isShell = code.closest(".language-bash, .language-sh") !== null;
  const isInterface =
    fields.some(isKnownType) &&
    (!isShell || fields.every(isKnownType)) &&
    lines.every((text) => FIELD.test(text) || OTHER_LINE.test(text));
  if (!isInterface) return;
  code.dataset.awInterface = "1";
  const parts = lines.flatMap((text) => [...interfaceLine(text), "\n"]);
  code.replaceChildren(...parts.filter((part) => part !== "" && part !== undefined));
}

document$.subscribe(() => {
  document.querySelectorAll(INTERFACE_BLOCKS).forEach(highlightInterface);
});
