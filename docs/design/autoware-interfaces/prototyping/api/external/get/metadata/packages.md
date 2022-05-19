# /api/external/get/metadata/packages

## Classification

- Behavior: Service
- DataType: tier4_external_api_msgs/srv/GetMetadataPackages

## Description

Autoware の実装バージョンを特定するための各種情報を取得する。

## Requirement

Autoware を構成する各パッケージが本 API 向けの情報を提供している場合、パッケージ名をキーとする辞書形式で情報が取得できること。
