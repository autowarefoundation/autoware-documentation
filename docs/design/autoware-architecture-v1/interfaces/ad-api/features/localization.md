# Localization

## Related API

- {{ link_ad_api('/api/localization/initialization_state') }}
- {{ link_ad_api('/api/localization/initialize') }}

## Description

This API manages the initialization of localization. Autoware requires a global pose as the initial guess for localization.
The application can give this pose in the request, or send an empty pose array to let Autoware use the GNSS pose.
See [initialize the pose](../use-cases/initialize-pose.md) for the sequence.

## States

{{ diagram("localization/state.svg", "The localization initialization state changes with the initialize request and its result.") }}

| State         | Description                                                                      |
| ------------- | -------------------------------------------------------------------------------- |
| UNINITIALIZED | Localization is not initialized. Waiting for a global pose as the initial guess. |
| INITIALIZING  | Localization is initializing.                                                    |
| INITIALIZED   | Localization is initialized. Initialization can be requested again if necessary. |

If an initialization fails, the state changes to UNINITIALIZED, also when localization was initialized before the request.
