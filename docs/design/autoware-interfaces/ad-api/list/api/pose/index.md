# Pose API

- {{ link_ad_api('/api/pose/state') }}
- {{ link_ad_api('/api/pose/initialize') }}

## Description

This API manages the initialization of the vehicle pose. Autoware requires a global pose as the initial guess for localization.

## States

![pose-reset-state](./state.drawio.svg)

| State        | Description                                                            |
| ------------ | ---------------------------------------------------------------------- |
| UNAVAILABLE  | The vehicle pose is not available. Waiting for initialization request. |
| INITIALIZING | The vehicle pose is not available. Initialization is in progress.      |
| AVAILABLE    | The vehicle pose is available. Initialization can be requested again.  |
