# Autonomy Msgs <!-- omit in toc -->

<!-- Created using Markdown All In One vscode extension -->

- [Introduction](#introduction)
- [Messages](#messages)
- [Services](#services)
- [Actions](#actions)

## Introduction

This package contains all message definitions for the main Autonomy Software Stack interface. This package should contain all message definitions required for external communication with the Autonomy Software Stack.

However, other messages might be defined by the components of the Autonomy Software Stack as well.

## Messages

| Message              | Purpose                                                                                                 |
| -------------------- | ------------------------------------------------------------------------------------------------------- |
| CoordinateSystemInfo | Contains information regarding the currently used coordinate system.                                    |
| Map2D                | Defines a `OccupancyGrid` map with database id and human readable name                                  |
| Path                 | Defines a `PoseWithTwist` path with database id and human readable name                                 |
| Polygon              | Defines a 2d polygon with database id and human readable name                                           |
| PoseWithTwist        | Custom definition of a pose with a twist. Only contains the data we need, in contrast to `Odometry` msg |
| StatusString         | Pass along status strings within the ros system.                                                        |

## Services

| Service             | Purpose                                                               |
| ------------------- | --------------------------------------------------------------------- |
| GetState            | Get the current (supervisor) state from a node                        |
| SetCoordinateSystem | Tells a node to use a target coordinate system                        |
| SetState            | Tells a node to set a new target (supervisor) state                   |
| Trigger             | Redefinition of std_srvs/Trigger to return success = false by default |

## Actions

No actions are defined
