# Localization

## Related API

- {{ link_ad_api('/api/localization/initialization_state') }}
- {{ link_ad_api('/api/localization/initialize') }}

## Description

This API manages the initialization of localization. Autoware requires a global pose as the initial guess for localization.

## States

{{ diagram("localization/state.svg", "The localization initialization state changes with the initialize request and its result.") }}

| State         | Description                                                                      |
| ------------- | -------------------------------------------------------------------------------- |
| UNINITIALIZED | Localization is not initialized. Waiting for a global pose as the initial guess. |
| INITIALIZING  | Localization is initializing.                                                    |
| INITIALIZED   | Localization is initialized. Initialization can be requested again if necessary. |
