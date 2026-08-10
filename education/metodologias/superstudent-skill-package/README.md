# SuperStudent: A Troubleshooting and Methodology Skill for Collaborative Robotics Education

## Overview
This package contains an AI-consumable methodology skill titled **SuperStudent**. It encapsulates the troubleshooting heuristics, integration rules, and robotics engineering experience derived from the Burger Delivery collaborative robotics experiment (Kinova Gen3 + TurtleBots + AprilTag + MoveIt 2).

Unlike theoretical manuals, this skill provides empirical, lab-tested heuristics for diagnosing TF tree conflicts, reducing joint jittering, handling network synchronization in ROS 2, and resolving perception-to-manipulation integration issues.

## Contents
- `SKILL.md`: The core knowledge base designed to be parsed by LLMs and autonomous agents. It contains both systemic design rules and a chronological learning log of robotics issues and solutions.
- `CITATION.cff`: Citation metadata for referencing this skill in academic literature.
- `LICENSE`: CC-BY-4.0 open access license.

## Usage for AI Agents
To equip an AI agent with this methodology:
1. Place `SKILL.md` in your agent's knowledge base or prompt context.
2. Instruct the agent to reference the "SuperStudent Methodology" when presented with ROS 2 debugging queries, specifically regarding TF trees, URDF localization, and manipulation timeouts.

## License
This work is published under the Creative Commons Attribution 4.0 International License (CC-BY-4.0).
