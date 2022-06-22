# Change the operation mode

## Related API

- Operation mode

## Sequence

- Change to manual/autonomous/command mode with software.

  ```plantuml
  {% include 'design/autoware-interfaces/ad-api/use-cases/sequence/operation-mode-software.plantuml' %}
  ```

- Change to manual control mode with hardware.

  ```plantuml
  {% include 'design/autoware-interfaces/ad-api/use-cases/sequence/operation-mode-hardware-direct.plantuml' %}
  ```

- Change to autonomous/command control mode with hardware.

  ```plantuml
  {% include 'design/autoware-interfaces/ad-api/use-cases/sequence/operation-mode-hardware-command.plantuml' %}
  ```
