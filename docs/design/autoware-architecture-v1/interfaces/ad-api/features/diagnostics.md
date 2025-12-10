# Diagnostics

## Related API

- {{ link_ad_api('/api/system/diagnostics/struct') }}
- {{ link_ad_api('/api/system/diagnostics/status') }}
- {{ link_ad_api('/api/system/diagnostics/reset') }}

## Description

This API provides a diagnostic graph consisting of error levels for functional units of Autoware.
The system groups functions into arbitrary units depending on configuration and diagnoses their error levels.
Each functional unit has dependencies, so the whole looks like a fault tree analysis (FTA).
In practice, it becomes a directed acyclic graph (DAG) because multiple parents may share the same child.
Below is an example of the diagnostics provided by this API.
The `path` in the diagram is an arbitrary string that describes the functional unit, and the `level` is its error level.
For error level, the same value as `diagnostic_msgs/msg/DiagnosticStatus` is used.

![graph-tree](diagnostics/tree.drawio.svg)

The diagnostics data has static and dynamic parts, so the API provides these separately for efficiency.
Below is an example of a message that corresponds to the above diagram.
The static part of the diagnostic is published only once as the DiagGraphStruct that contains nodes, diags, and links.
The links specify dependencies between nodes by index into an array of them.
The dynamic part of the diagnostic is published periodically as DiagGraphStatus.
The status has nodes and diags of the same length as the struct, with the same index representing the same functional unit.

![graph-data](diagnostics/data.drawio.svg)

Some functional unit levels may be latched. If the abnormality continues for a certain period, the level value will not return to normal.
Use input_level to know the level before it was latched. Also, use latch_level to know if the latch is currently activated.
To restore the level after the latch has been activated, use the reset API.

![level](diagnostics/level.drawio.svg)
