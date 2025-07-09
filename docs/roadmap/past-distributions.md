# How is Autoware Core/Universe different from Autoware.AI and Autoware.Auto?

Autoware is the world's first "all-in-one" open-source software for self-driving vehicles.
Since it was first released in 2015, there have been multiple releases made with differing underlying concepts, each one aimed at improving the software.

## Autoware.AI

[Autoware.AI](https://github.com/Autoware-AI/autoware.ai) is the first distribution of Autoware that was released based on ROS 1. The repository contains a variety of packages covering different aspects of autonomous driving technologies - sensing, actuation, localization, mapping, perception and planning.

While it was successful in attracting many developers and contributions, it was difficult to improve Autoware.AI's capabilities for a number of reasons:

- A lack of concrete architecture design leading to a lot of built-up technical debt, such as tight coupling between modules and unclear module responsibility.
- Differing coding standards for each package, with very low test coverage.

Furthermore, there was no clear definition of the conditions under which an Autoware-enabled autonomous vehicle could operate, nor of the use cases or situations supported (eg: the ability to overtake a stationary vehicle).

From the lessons learned from Autoware.AI development, a different development process was taken for Autoware.Auto to develop a ROS 2 version of Autoware.

!!! warning

    Autoware.AI has reached end-of-life at the end of 2022.

## Autoware.Auto

[Autoware.Auto](https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto) is the second distribution of Autoware that was released based on ROS 2. As part of the transition to ROS 2, it was decided to avoid simply porting Autoware.AI from ROS 1 to ROS 2. Instead, the codebase was rewritten from scratch with proper engineering practices, including defining target use cases and ODDs (eg: Autonomous Valet Parking [AVP], Cargo Delivery, etc.), designing a proper architecture, writing design documents and test code.

Autoware.Auto development seemed to work fine initially, but after completing the AVP and and Cargo Delivery ODD projects, we started to see the following issues:

- The barrier to new engineers was too high.
  - A lot of work was required to merge new features into Autoware.Auto, and so it was difficult for researchers and students to contribute to development.
  - As a consequence, most Autoware.Auto developers were from companies in the Autoware Foundation and so there were very few people who were able to add state-of-the-art features from research papers.
- Making large-scale architecture changes was too difficult.
  - To try out experimental architecture, there was a very large overhead involved in keeping the main branch stable whilst also making sure that every change satisfied the continuous integration requirements.
