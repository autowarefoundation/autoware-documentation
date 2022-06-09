# Interface feature

## Description

Considering the product life cycle, there may be multiple vehicles using different versions of the AD API due to changes in requirements or functional improvements.
For example, one vehicle uses `v1` for stability and another vehicle uses `v2` to enable more advanced functionality.

In that situation, the AD API users such as developers of a web service have to switch the application behavior based on the version that each vehicle uses.
The version of AD API follows [Semantic Versioning][semver] in order to provide an intuitive understanding of the changes between versions.

## Related API

- /api/interface/version

<!-- link -->

[semver]: https://semver.org/
