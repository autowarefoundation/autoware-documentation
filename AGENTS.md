# autoware-documentation

The documentation site of the Autoware Foundation. Zensical builds it from `zensical.toml`, and the `publish-docs` workflow publishes it to GitHub Pages:

- <https://autowarefoundation.github.io/autoware-documentation/main/>

## Build

1. Make a virtual environment with Python 3.11 or later.
2. Install `requirements.txt` into it.
3. Run `zensical serve`, then open <http://127.0.0.1:8000/autoware-documentation/>.

A clean tree builds with "No issues found". If the build shows a warning, your change caused it.

- To find macro errors, run `zensical build --clean`. An incremental build can skip a page and hide them.
- Jinja reads `{{ }}` in every page. Put GitHub Actions expressions such as `${{ github.ref }}` between `{% raw %}` and `{% endraw %}`.
- Zensical has its own versions of the plugins in `zensical.toml`. It does not run plugins from PyPI.

## Layout

- `docs/`: the content of the site.
- `.nav.yml` files: the navigation, starting at `docs/.nav.yml`. If the directory of a page has a `.nav.yml` file, add the page there. A page that this file leaves out is built but hidden. A directory without a `.nav.yml` file shows all its pages in alphabetical order.
- The `llmstxt` plugin writes `llms.txt` and a Markdown copy (`index.md`) of each page for AI agents. The copy page button uses that copy. Its `sections` in `zensical.toml` list each top-level directory of `docs/`. If you add a top-level directory, add it there too. `python3 tools/docs-site/llmstxt_sections.py` finds a page that no section lists, and the build fails on such a page.
- `zensical.toml`: the site configuration. `publish-docs` reads `site_url` from it. `site_url` must end with `/`.
- `main.py`: the macros, for example `diagram` and `resolve_msg_field`. Macros read `page` from the Jinja context (`@jinja2.pass_context`), not from `env`.
- `includes/abbreviations.md`: appended to every page. Define abbreviations there.
- `overrides/`: template overrides. `overrides/main.html` also defines the arrow heads and gradients of the diagrams.
- `docs/assets/css/theme.css`: the colors, the fonts, and the diagram styles.
- `tools/docs-site/`: the scripts that publish all versions as one site. Their README tells how.
- `tools/diagrams/`: the diagram style guide and `standalone.py`.
- `tools/autoware-interfaces/generate.py`: writes `yaml/autoware-interfaces.yaml`, the pages under `docs/design/autoware-architecture-v1/interfaces/ad-api/types/`, and `ad-api/list/index.md`. Do not edit these files by hand, because the next generation overwrites them. The pages under `ad-api/list/api/` are written by hand. See `tools/autoware-interfaces/README.md`.

Do not add `docs/index.md`. Zensical shows it as the first breadcrumb item on every page.

Each pull request from a branch in this repository gets a preview at `/pr-N/`, and a bot comments its address. A pull request from a fork gets a link to the built site as an artifact.

## Diagrams

All diagrams in this repository are hand-written SVG files in one style. The `diagram` macro puts each SVG into its page. `theme.css` gives the classes of the SVG the colors of the theme, so every diagram follows the light and the dark mode. `tools/diagrams/README.md` is the full reference, with examples. If you change a diagram style, update this section and that README.

Use a hand-written SVG file for each diagram. Use PNG or JPG only for screenshots and photos. Do not use Mermaid, PlantUML, or draw.io. `zensical.toml` still configures Mermaid and PlantUML, and `main.py` still has a `drawio` filter, but no page uses them.

### Add a diagram

1. Write the SVG file in the image folder that the page already uses. For a new page, use `images/`.
2. In the Markdown, call the `diagram` macro on its own line, with a blank line before and after it.
3. Run `python3 tools/diagrams/standalone.py <folder or file>`.
4. Run `pre-commit run prettier-svg --all-files`.

```jinja
{{ diagram("images/generator-selector.svg", "Generators make candidate trajectories, and the selector picks one.") }}
```

The first argument is the path relative to the page. The second argument is an optional caption. Do not use `![...](file.svg)`, because an image element cannot use the colors of the theme.

`standalone.py` adds a fallback `<style>` and `<defs>` with the attribute `data-aw-standalone`, so that GitHub shows the file correctly. The macro removes them from the page. Do not edit them by hand.

### The look

- Colors: indigo for node borders, groups, and the main node, grey for lines, cyan for AI nodes and the main flow, green for success, and crimson for faults. The classes set all colors.
- Text: the heading font of the site (Outfit). Code names use the monospace font.
- Shapes: use `rx="12"` for boxes and cards, and `rx="16"` for groups. A box has the page background and a 1.5 px indigo border. A group has a light indigo tint.
- Depth and motion: most diagrams use the illustration classes, for example a dot grid, glows, cards, and moving pulses on the lines. Map drawings use mostly the glass vehicles and the pulses on the routes.

### The SVG file

- Start with `<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 W H" role="img" aria-labelledby="ID-title">`. Then add a `<title id="ID-title">` that describes the diagram in one sentence. Use an id that is unique on the site, for example the file name.
- Use a width of 720 to 960 for the `viewBox`.
- Do not set colors, fonts, or stroke widths. Use the classes that follow. The only exception is `fill="none"` on a path.
- Do not add your own `<defs>`, markers, or gradients.
- Do not use `<foreignObject>`, embedded images, or text converted to paths.
- To center text in its box, use `text-anchor="middle"` and `dominant-baseline="central"`.
- Do not put a space between two elements, for example between two `<tspan>` elements. The macro removes it. Put the space inside the text.

### Shape classes

Use `<rect>` for boxes and groups.

- `box`: a node, for example a module, a package, or a component.
- `box primary`: the main node, with the indigo gradient. Use `on-primary` text on it.
- `box ai`: a learned or AI-based node, with a cyan tint.
- `box ok`: success, a safe state, or an output that passes a check, with a green tint.
- `box warn`: a fault, an emergency, or a rejected output, with a crimson tint.
- `box muted`: a secondary node, for example an external input or an optional part.
- `box ext`: a node outside Autoware, or a planned node. Dashed, with no fill.
- `group`: a container of nodes, for example a component or a layer. Put a `ttl` text in its corner.
- `group ai`: a container of learned nodes, with a cyan tint.
- `group dashed`: a container with a dashed border, for example an optional part.
- `icon`: a small shape, for example a vehicle or a sensor, in a muted color.

### Text classes

- No class: normal text, 14 px.
- `lbl`: the name of a node, 14 px, bold.
- `sub`: a second line under a name, 12 px, muted.
- `code`: a topic, a package, or a message name, 12 px, monospace.
- `ttl`: the title of a group, 12 px, bold, upper case.
- `on-primary`: text on a `box primary`. Add it next to `lbl` or `sub`.
- `edge-lbl`: a label on a line, 12 px. It has a halo, so it can sit on the line.

### Line classes

Use `<path>` or `<line>`. A line gets an arrow head at its end.

- `edge`: a data flow or a dependency.
- `edge accent`: the main flow of the diagram. Cyan and thicker.
- `edge dash`: an optional or planned flow. Add `dash` to the other classes.
- `edge both`: a flow in both directions. Add `both` to the other classes.
- `edge plain`: a line without an arrow head.

### Map drawing classes

Use these classes for drawings of roads, lanelets, and vehicles. The road colors are the same in both modes, because they show real surfaces.

- `road`, `shoulder`, `sidewalk`: asphalt, a road shoulder, and a sidewalk.
- `mark`: a white lane marking. Add `dash` for a dashed line, `yellow` for a yellow or orange line, and `thick` for a stop line or a crosswalk stripe.
- `lanelet`: a translucent indigo area. `lanelet alt` is cyan, for a second lanelet. `lanelet bad` is crimson, for a wrong example.
- `area`: a map area, for example a detection area. Green and dashed. `area warn` is crimson.
- `bound`: a linestring, for example a lanelet bound, in cyan. `bound alt` is light red. `bound point` is a circle on a linestring.
- `route`: the path of a vehicle, with a green arrow head. `route plain` has no arrow head.
- `sign`, `sign-text`: a round traffic sign with a crimson ring, and its text.
- `sign-panel`, `sign-fill`, `sign-line`, `sign-label`: a blue square traffic sign with a white border, and its white shapes, strokes, and text.
- `light-housing`, `light-red`, `light-yellow`, `light-green`, `light-off`: a traffic light and its lamps.
- `leader`: a thin line from a note to the thing that it describes, with no arrow head.
- `note`: a note on a drawing, 13 px. `note bad` is crimson, for "do not do this". `note good` is green. `note code` is monospace, for a tag name or a value.
- `mark-text`: white text on the road, for example a speed number.

The vehicles and road users of a map drawing use the glass style. A vehicle is 22 by 40 units and drives up. Move it with `translate`, and turn it with `rotate` around its center (11, 20):

```xml
<g transform="translate(X Y)">
  <rect class="car-shadow" x="0" y="1" width="22" height="40" rx="7"/>
  <rect class="car-body" x="0" y="0" width="22" height="40" rx="7"/>
  <path class="car-glass" d="M4 10 Q11 7 18 10 L16.5 15.5 Q11 14 5.5 15.5 Z"/>
  <rect class="car-glass" x="5.5" y="29" width="11" height="5" rx="2"/>
  <circle class="lidar" cx="11" cy="22" r="3.2"/>
  <rect class="headlight" x="3" y="1" width="4" height="2" rx="1"/>
  <rect class="headlight" x="15" y="1" width="4" height="2" rx="1"/>
</g>
```

For another vehicle, use `car-body other` (crimson) or `car-body npc` (grey), and leave out the LiDAR and the headlights. Pedestrians (`ped-body`, `ped-head`), traffic cones (`cone`, `cone-base`, `cone-stripe`), bicycles (`bike-wheel`, `bike-frame`), and trucks (`truck-box`, `truck-rib`) have the same style. `tools/diagrams/README.md` gives their sizes, and `docs/design/autoware-architecture-v1/components/planning/image/features-*.svg` shows them.

### Illustration classes

Most diagrams use these classes. They add depth and motion. For an example, see `docs/design/autoware-architecture-v2/images/generator-selector.svg`. Map drawings use mostly the glass vehicles and the pulses on the routes.

- `dots`: a faint dot grid. Put a `rect` of the full `viewBox` size first.
- `halo`, `halo cyan`: a soft indigo or cyan glow behind a group. Keep the circle inside the `viewBox`.
- `glass-panel`: a translucent panel for a group.
- `card`, `card ai`: a node with a soft shadow. `card ai` has a cyan tint.
- `icon-bg`, `icon-bg ai`, `icon-bg on`: a round icon background in indigo, in cyan, or in white on a `box primary`.
- `icon-line`, `icon-dot`: the white strokes and dots of an icon.
- `pill`, `pill-text`: a small step inside a `box primary`.
- `traj`: a trajectory, as a line with an indigo-to-cyan gradient. Do not make it a straight vertical or horizontal line, because a gradient needs a box with a size.
- `glow`: a glow on a line or a shape.
- `flow`, `flow flow-accent`, `flow flow-traj`, `flow flow-route`: two pulses that move along an `edge`, an `edge accent`, a `traj`, or a `route`. Draw the pulses as a `path` with the same `d` and `pathLength="100"` on top of the line. Then draw the line again with the extra class `head`, so that its arrow head stays on top. The pulses stop for readers who ask for reduced motion.
- `car-body`, `car-glass`, `car-shine`, `wheel`, `rim`, `hub`, `car-shadow`, `headlight`, `lidar`: the glass car. Copy its `g` element from `generator-selector.svg` to use it again.

## Files that you must not edit here

The `sync-files` workflow overwrites the files that `.github/sync-files.yaml` lists, for example `.pre-commit-config.yaml` and `.markdownlint.yaml`. Change them in the template repository:

- <https://github.com/autowarefoundation/sync-file-templates>

`autowarefoundation/open-ad-kit-docs` copies some files of this repository with its own `sync-files` workflow, for example `.github/workflows/deploy-docs.yaml`. Its `.github/sync-files.yaml` lists them. A change to such a file here also reaches that repository.

`mkdocs-base.yaml` is an old copy that nothing uses. Other Autoware repositories get their `mkdocs.yaml` from the template repository. Edit `zensical.toml` for this site.

## Writing

Follow `docs/contributing/documentation-guidelines/index.md`:

- Use American English.
- Use sentence case for page titles and headings.
- Use descriptive link text. Never write "click here".
- Write short and simple sentences. Many readers are not native speakers.

## Checks

- pre-commit.ci runs the pre-commit hooks on each pull request. To run them before you push, run `pre-commit run --all-files`.
- cspell checks the changed files with the shared Autoware dictionary. Add a word to `.cspell.json` only if this repository alone uses it. For a word in one file, add a `cspell:ignore` comment to that file.
- To test the scripts in `tools/`, run `pip3 install pytest -r tools/docs-site/requirements.txt`, then `pytest tools/ -v`.

## Commits and pull requests

- Use conventional commits, for example `docs(tutorials): update the launch tutorials`. Pull request titles use the same format.
- Each commit needs the DCO `Signed-off-by` line of its human author. An agent never signs in its own name. It adds the sign-off of its operator only when the operator tells it to.
- Read `docs/contributing/ai-contribution-policy.md` before you open a pull request that AI helped to write. In short:
  - If AI did more than autocompletion, grammar or spelling fixes, or translation, say so in the description: which tool, and how much.
  - Add a verification statement to the description: what AI generated, what the author reviewed, and how the change was tested.
  - Do not add AI tools as `Co-authored-by`, and do not add `Assisted-by` trailers.
  - A human reviews each pull request, issue, and comment before an agent posts it.

<!-- cspell:ignore endraw -->
