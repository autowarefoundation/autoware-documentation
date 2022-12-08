# Resource API

- {{ link_ad_api('/api/resource/active_hashes') }}
- {{ link_ad_api('/api/resource/latest_hashes') }} (T.B.D.)
- {{ link_ad_api('/api/resource/get') }} (T.B.D.)
- {{ link_ad_api('/api/resource/put') }} (T.B.D.)

## Description

This API manages resources such as maps and parameters. Each resource is identified by a unique name.
Currently only hashes are provided as resources are dependent on the file system.

## Hashes

Applications can use hashes to check resource versions.
Some modules do not reflect resource updates during execution, so there are two types of hash: active and latest.
The active hash is related to resource already reflected and the latest hash is simply related to the latest resource.
If resources are updated while Autoware is running, these can become mismatched.
