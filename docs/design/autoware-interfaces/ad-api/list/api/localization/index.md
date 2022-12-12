# Localization API

- {{ link_ad_api('/api/localization/initialization_state') }}
- {{ link_ad_api('/api/localization/initialize') }}

## Description

This API manages the initialization of localization. Autoware requires a global pose as the initial guess for localization.

## States

![localization-initialization-state](./state.drawio.svg)

| State         | Description                                                                      |
| ------------- | -------------------------------------------------------------------------------- |
| UNINITIALIZED | Localization is not initialized. Waiting for a global pose as the initial guess. |
| INITIALIZING  | Localization is initializing.                                                    |
| INITIALIZED   | Localization is initialized. Initialization can be requested again if necessary. |
