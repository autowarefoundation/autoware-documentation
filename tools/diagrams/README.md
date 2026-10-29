# Diagrams

The diagrams of this site are hand-written SVG files. The SVG uses a small set of classes, and `docs/assets/css/theme.css` gives these classes the colors of the theme. As a result, every diagram has the same look, and it follows the light and the dark mode.

## Add a diagram to a page

1. Write the SVG file next to the page, in the `images/` folder of the page, with the extension `.svg`.
2. In the Markdown, call the `diagram` macro on its own line, with a blank line before and after it:

   ```jinja
   {{ diagram("images/generator-selector.svg", "Generators make candidate trajectories, and the selector picks one.") }}
   ```

   The first argument is the path relative to the page. The second argument is an optional caption.

The macro puts the SVG into the HTML of the page. Do not use `![...](file.svg)` for these files. An image element cannot use the colors of the theme.

`zensical serve` does not rebuild a page when only its SVG changes. Save the Markdown file again to see the change.

## Viewers outside the site

GitHub and other viewers show an SVG file on its own, without the CSS of the site. So each SVG file carries a fallback style and the definitions that it uses, in a `<style>` and a `<defs>` element with the attribute `data-aw-standalone`. The `diagram` macro removes both when it puts the SVG into a page.

Do not edit these two elements by hand. After you add or change a diagram, or change the diagram styles in `docs/assets/css/theme.css` or the definitions in `overrides/main.html`, run:

```bash
python3 tools/diagrams/standalone.py docs/design
pre-commit run prettier-svg --all-files
```

## The SVG file

- Start with `<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 W H" role="img" aria-labelledby="ID-title">`, then a `<title id="ID-title">` that describes the diagram in one sentence. Use an id that is unique on the site, for example the file name.
- Use a width of 720 to 960 for the `viewBox`. One unit is about one pixel on a desktop screen.
- Do not set colors, fonts, or stroke widths in the file. Use the classes below. The only exception is `fill="none"` on a path.
- Do not add your own `<defs>`, markers, or gradients. The site defines them once (in `overrides/main.html`), and `standalone.py` copies the ones that a file uses into it.
- Do not use `<foreignObject>`, embedded images, or text converted to paths.
- Put text with `text-anchor="middle"` and `dominant-baseline="central"` in the center of its box.
- Do not put a space between two elements, for example between two `<tspan>` elements. The macro removes the white space between tags. Put the space inside the text.

## Classes

### Shapes

Use `<rect>` with `rx="10"` for boxes and `rx="16"` for groups.

| Class          | Use                                                                                       |
| :------------- | :---------------------------------------------------------------------------------------- |
| `box`          | A node, for example a module, a package, or a component.                                  |
| `box primary`  | The main node of the diagram. It has the indigo gradient. Use `on-primary` text on it.    |
| `box ai`       | A learned or AI-based node. It has a cyan tint.                                           |
| `box ok`       | A node that means success, a safe state, or an output that passes a check. Green tint.    |
| `box warn`     | A node that means a fault, an emergency, or a rejected output. Crimson tint.              |
| `box muted`    | A secondary node, for example an external input or an optional part.                      |
| `box ext`      | A node outside Autoware, or a planned node that does not exist yet. Dashed, no fill.      |
| `group`        | A container of nodes, for example a component or a layer. Put a `ttl` text in its corner. |
| `group ai`     | A container of learned nodes. Cyan tint.                                                  |
| `group dashed` | A container whose border is dashed, for example an optional part.                         |
| `icon`         | A small shape, for example a vehicle or a sensor. Muted color.                            |

### Text

| Class        | Use                                                                 |
| :----------- | :------------------------------------------------------------------ |
| (none)       | Normal text, 14 px.                                                 |
| `lbl`        | The name of a node, 14 px, bold.                                    |
| `sub`        | A second line under a name, 12 px, muted.                           |
| `code`       | A topic, a package, or a message name, 12 px, monospace.            |
| `ttl`        | The title of a group, 12 px, bold, upper case.                      |
| `on-primary` | Text on a `box primary`. Add it next to `lbl` or `sub`.             |
| `edge-lbl`   | A label on a line, 12 px. It has a halo, so it can sit on the line. |

### Lines

Use `<path>` or `<line>`. A line gets an arrow head at its end.

| Class         | Use                                                |
| :------------ | :------------------------------------------------- |
| `edge`        | A data flow or a dependency.                       |
| `edge accent` | The main flow of the diagram. Cyan, thicker.       |
| `edge dash`   | An optional or planned flow. Add it to the others. |
| `edge both`   | A flow in both directions. Add it to the others.   |
| `edge plain`  | A line without an arrow head.                      |

### Map illustrations

Use these classes for drawings of roads, lanelets, and vehicles. The road colors are the same in both modes, because they show real surfaces.

| Class                                                                    | Use                                                                                                                                                                               |
| :----------------------------------------------------------------------- | :-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `road`, `shoulder`, `sidewalk`                                           | Surfaces: asphalt, road shoulder, sidewalk.                                                                                                                                       |
| `mark`                                                                   | A white lane marking. Add `dash` for a dashed line, `yellow` for a yellow or orange line, `thick` for a stop line or crosswalk stripe.                                            |
| `lanelet`                                                                | A lanelet, as a translucent indigo area. `lanelet alt` is cyan, for a second lanelet. `lanelet bad` is crimson, for a wrong example.                                              |
| `area`                                                                   | A map area, for example a detection area. Green, dashed. `area warn` is crimson.                                                                                                  |
| `bound`                                                                  | A linestring, for example a lanelet bound. Cyan. `bound alt` is light red. `bound point` is a point of a linestring (a circle).                                                   |
| `route`                                                                  | The path of a vehicle, with a green arrow head. `route plain` has no arrow head.                                                                                                  |
| `car`                                                                    | The ego vehicle, indigo. `car other` is a crimson vehicle, `car npc` is a grey vehicle. Draw a rounded `rect` with `rx="6"` and put a `glass` rectangle on it for the windshield. |
| `sign`, `sign-text`                                                      | A round traffic sign with a crimson ring, and its text.                                                                                                                           |
| `light-housing`, `light-red`, `light-yellow`, `light-green`, `light-off` | A traffic light and its lamps.                                                                                                                                                    |
| `leader`                                                                 | A thin line from a note to the thing that it describes. No arrow head.                                                                                                            |
| `note`                                                                   | A note on an illustration, 13 px. `note bad` is crimson, for "do not do this". `note good` is green.                                                                              |
| `note code`                                                              | A note in monospace, for example a tag name or a value.                                                                                                                           |
| `mark-text`                                                              | White text painted on the road, for example a speed number.                                                                                                                       |

### Illustration layer

Use these classes only for a few overview diagrams, for example the generator-selector diagram of Architecture 2.0. They add depth and motion. Reference diagrams stay plain.

| Class                                                                                           | Use                                                                                                                                                                                                                                                                                                                                                                                                                          |
| :---------------------------------------------------------------------------------------------- | :--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `dots`                                                                                          | A faint dot grid. Put a `rect` of the full `viewBox` size first.                                                                                                                                                                                                                                                                                                                                                             |
| `halo`, `halo cyan`                                                                             | A soft indigo or cyan glow behind a group. It fades to nothing at its radius, so keep the circle inside the `viewBox`.                                                                                                                                                                                                                                                                                                       |
| `glass-panel`                                                                                   | A translucent panel for a group.                                                                                                                                                                                                                                                                                                                                                                                             |
| `card`, `card ai`                                                                               | A node with a soft shadow. `card ai` has a cyan tint.                                                                                                                                                                                                                                                                                                                                                                        |
| `icon-bg`, `icon-bg ai`, `icon-bg on`                                                           | A round icon background: indigo, cyan, or white on a `box primary`.                                                                                                                                                                                                                                                                                                                                                          |
| `icon-line`, `icon-dot`                                                                         | The white strokes and dots of an icon.                                                                                                                                                                                                                                                                                                                                                                                       |
| `pill`, `pill-text`                                                                             | A small step inside a `box primary`.                                                                                                                                                                                                                                                                                                                                                                                         |
| `traj`                                                                                          | A trajectory: a line with an indigo-to-cyan gradient. It must not be a straight vertical or horizontal line, because a gradient needs a box with a size.                                                                                                                                                                                                                                                                     |
| `glow`                                                                                          | Adds a glow to a line or a shape.                                                                                                                                                                                                                                                                                                                                                                                            |
| `flow`, `flow flow-accent`, `flow flow-traj`                                                    | Two pulses that move along a line, in a lighter tone of the line color. Use `flow` on an `edge`, `flow flow-accent` on an `edge accent`, and `flow flow-traj` on a `traj`. Draw it on top of the line as a `path` with the same `d` and `pathLength="100"`. Then draw the line again with the extra class `head`, so that its arrow head stays on top of the pulses. The pulses stop for readers who ask for reduced motion. |
| `car-body`, `car-glass`, `car-shine`, `wheel`, `rim`, `hub`, `car-shadow`, `headlight`, `lidar` | The glass car of the generator-selector diagram. Copy its `g` element to reuse it.                                                                                                                                                                                                                                                                                                                                           |

### The top-down glass car

Map drawings show vehicles from above. Use this group for a vehicle of 22 by 40 units that drives up. Move it with `translate`, and turn it with `rotate` around its center (11, 20). For another vehicle, use `car-body other` (crimson) or `car-body npc` (grey), and leave out the LiDAR and the headlights.

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

Other road users have the same glass style. Their sizes match the car:

- A pedestrian, with the head at (cx, cy): an `ellipse class="car-shadow"` under the feet, a `path class="ped-body"` for the body (14 wide, from cy+7 to cy+25), and a `circle class="ped-head"` with `r="5.5"`.
- An obstacle, as a traffic cone with its tip at (x, y): a `car-shadow` ellipse, a `cone-base` rect (26 by 5), a `cone` triangle (20 wide, 21 high), and a white `cone-stripe` across the middle.
- A bicycle of 40 by 16 that rides right: two `bike-wheel` rectangles, a `bike-frame` path for the frame and the handlebar, and a rider made of a `ped-body` ellipse and a `ped-head` circle.
- A truck of 34 by 110 that drives up: a `truck-box` trailer with `truck-rib` lines, a `car-body npc` cab of 32 by 28, and a `car-glass` windshield.

See `docs/design/autoware-architecture-v1/components/planning/image/` for examples: `features-run-out.svg` (pedestrian), `features-obstacle-stop.svg` (cone), `features-blind-spot.svg` (bicycle), and `features-intersection-occlusion.svg` (truck).

A route with pulses: draw the `route`, then a `flow flow-route` path with the same `d` and `pathLength="100"`, then the `route head` copy.

## Example

See `docs/design/autoware-architecture-v2/images/generator-selector.svg`.
