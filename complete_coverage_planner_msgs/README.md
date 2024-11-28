# Complete Coverage Planner Msgs <!-- omit in toc -->

<!-- Created using Markdown All In One vscode extension -->
- [Introduction](#introduction)
- [Messages](#messages)
- [Services](#services)
- [Actions](#actions)

## Introduction

This package contains services that are specifically used to interact with the Complete Coverage Planner node.

## Messages
| Message      | Purpose                                                      |
| ------------ | ------------------------------------------------------------ |
| Area         | Definition of an area, containing an outline and exclusions  |
| InfillPolicy | Definition of an infill policy for complete coverage planner |

## Services

| Service | Purpose                                                                                                                                      |
| ------- | -------------------------------------------------------------------------------------------------------------------------------------------- |
| SetArea | Set an area for the complete coverage planner. This can either be an area that needs to be covered, or an area that the planner cannot exit. |

## Actions
| Action      | Purpose                                                                                        |
| ----------- | ---------------------------------------------------------------------------------------------- |
| ComputePath | Action to compute a path, given a start and goal pose, allowed and coverage areas, and policy. |
