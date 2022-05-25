# Testing guidelines

## Unit tests

Unit Testing is a software testing method by which individual units of source code are tested to determine whether they are fit for use.

The tool used for unit testing in Autoware is `gtest`.

[Unit testing guidelines](unit-testing.md)

## Integration tests

Integration Testing combines and tests the individual software modules as a group, and is done after unit testing.

While performing integration testing, the following subtypes of tests are written:

1. Fault injection testing
1. Back-to-back comparison between a model and code
1. Requirements-based testing
1. Anomaly detection during integration testing
1. Random input testing

[Integration testing guidelines](integration-testing.md)
