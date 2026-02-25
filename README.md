# Autoware Documentation

<p align="center">
  <img src="./docs/assets/images/autoware-foundation.png" width="128" alt="Autoware Foundation Logo" />
  <br />
  <a href="https://github.com/autowarefoundation/autoware-documentation/actions"><img src="https://github.com/autowarefoundation/autoware-documentation/actions/workflows/deploy-docs.yaml/badge.svg" alt="Deploy Docs" /></a>
  <a href="LICENSE"><img src="https://img.shields.io/badge/license-Apache%202.0-blue.svg" alt="License" /></a>
</p>

<p align="center">
  <strong>The central documentation hub for the <a href="https://github.com/autowarefoundation/autoware">Autoware</a> open-source autonomous driving platform.</strong>
</p>

<p align="center">
  <a href="https://autowarefoundation.github.io/autoware-documentation/main/">📖 Read the Documentation</a>
</p>

---

## About

This repository contains the source files for Autoware's documentation, built and deployed with [MkDocs](https://www.mkdocs.org/). Autoware spans [multiple repositories](https://github.com/autowarefoundation/), and this site serves as a unified entry point to access information across all of them.

For more about the Autoware project and its ecosystem, see the [Autoware Foundation organization profile](https://github.com/autowarefoundation/.github/blob/main/profile/README.md).

## Getting started locally

```bash
# Clone the repository
git clone https://github.com/autowarefoundation/autoware-documentation.git
cd autoware-documentation

# Create and activate a virtual environment
python3 -m venv .venv
source .venv/bin/activate

# Install dependencies
pip install -r mkdocs-requirements.txt

# Serve locally
mkdocs serve
```

Then open [http://localhost:8000](http://localhost:8000) in your browser.

## Contributing

Contributions are welcome! Please review the [documentation guidelines](https://autowarefoundation.github.io/autoware-documentation/main/contributing/documentation-guidelines/) before submitting a pull request.

## License

This project is licensed under the [Apache License 2.0](LICENSE).
