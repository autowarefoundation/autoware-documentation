# Autoware Architecture

## Executive Summary

This document presents a framework for the evolution of the Autoware Project from Autoware 1.0, a traditional robotics-based autonomous driving stack, to Autoware 2.0, a data-centric AI-based autonomous driving stack which can power safe and globally scalable End-to-End autonomous driving across multiple use-cases

## Overview

The Autonomous Driving industry has experienced rapid technological developments powered by advancements in Artificial Intelligence.

The emergence of neural networks have helped transform self-driving technologies from hand-coded software modules which follow a traditional robotics paradigm, to neural-network based components powered by machine learning.
By leveraging such data-driven approaches, self-driving technology developers have been able to rapidly scale and deploy autonomy around the world.

One of the latest such technological breakthroughs is the introduction of End-to-End autonomous driving, in which a single monolithic neural network can map sensing data to safe driving trajectories - essentially learning the entire driving task.

While this approach has great promise, there are important considerations around safety, explainability and model training which must be addressed.

Here, we present our framework for adopting End-to-End autonomous within the open-source Autoware project.

