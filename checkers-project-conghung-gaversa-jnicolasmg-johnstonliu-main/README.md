# CMSC 14200 Project - Checkers

## Group Members and Roles:

- Johnston Liu | Game Logic
- Hung Le Tran | GUI
- Giorgio Aversa | TUI
- Nico Marin Gamboa | Bot

## Setup

- Set up a virtual environment with necessary packages:

```
python3 -m venv venv
source venv/bin/activate
pip3 install -r requirements.txt
```

- To deactivate the virtual environment:

```
deactivate
```

## Instructions for running the TUI

- To run the TUI, run the following command from the root of the directory:

```
python3 src/tui.py
```

- CLI parameters:

| Parameter   | Type                                         | Default | Info                                                 |
| ----------- | -------------------------------------------- | ------- | ---------------------------------------------------- |
| --size      | int                                          | 8       | The size of the board                                |
| --player1   | human &#124; random-bot &#124; alphabeta-bot | human   | Type of player 1                                     |
| --player2   | human &#124; random-bot &#124; alphabeta-bot | human   | Type of player 2                                     |
| --bot-delay | float                                        | 0.1     | Delay time (in seconds) for bot if a player is a bot |

- Run the following command to get help

```
python3 src/tui.py --help
```

## Instructions for running the GUI

- To run the GUI, run the following command from the root of the directory:

```
python3 src/gui.py
```

- CLI parameters:

| Parameter   | Type                                         | Default | Info                                                 |
| ----------- | -------------------------------------------- | ------- | ---------------------------------------------------- |
| --size      | int                                          | 8       | The size of the board                                |
| --player1   | human &#124; random-bot &#124; alphabeta-bot | human   | Type of player 1                                     |
| --player2   | human &#124; random-bot &#124; alphabeta-bot | human   | Type of player 2                                     |
| --bot-delay | float                                        | 0.1     | Delay time (in seconds) for bot if a player is a bot |

- Run the following command to get help:

```
python3 src/gui.py --help
```

## Bots

- The `bot.py` includes 2 classes:

  - `RandomBot`: A bot that will just choose a move at random
  - `AlphaBetaBot`: A bot that uses a minimax algorithm with alpha-beta pruning

- These 2 classes are used in the TUI and GUI.
- To simulate a match between any 2 bots (random vs random, alphabeta vs alphabeta, random vs alphabeta), run the following command from the root of the directory (recommended number of games is 50 which is the default as well):

```
python3 src/benchmark.py
```

- CLI parameters:

| Parameter       | Type                    | Default   | Info                                                 |
| --------------- | ----------------------- | --------- | ---------------------------------------------------- |
| --size          | int                     | 8         | The size of the board                                |
| --n             | int                     | 50        | The number of games to be played                     |
| --bot1          | random &#124; alphabeta | random    | Type of bot 1                                        |
| --bot2          | random &#124; alphabeta | alphabeta | Type of bot 2                                        |
| --no-alt-colors | bool                    | False     | Whether the bots alternate colors after each game    |
| --bot-delay     | float                   | 0.1       | Delay time (in seconds) for bot if a player is a bot |

- Run the following command to get help:

```
python3 src/benchmark.py --help
```

## Changes in Checkers Design

- After Milestone 1: We modified from a multi-game design to a one-game design. Commands, both for moves and meta-game (offer draw, resign), are streamlined in one method.

- After Milestone 2: In response to feedback on the bloatedness of the Board class (as well as the need for a default Board that can be reused in other projects) the Board class was divided in to the `GeneralBoard` and `CheckersGame` classes.

## Changes in Checkers Implementation

- Modularized `GeneralBoard` class, with game-agnostic methods: initiating empty pieces, initiating each player's (empty) list of pieces, moving pieces, removing pieces, setting and getting piece attributes.
- Meanwhile, `CheckersGame` becomes solely concerned with game-specific functionalities.
- Added `Command` Enum for meta-game commands.
- Split the process_command() method into multiple helpers to improve readability.
- Added various in-line comments throughout to explain conditional statements.
- Added blank lines between dense code blocks to facilitate readability.

## Changes in TUI

- Appropriate refactoring for `GeneralBoard`, `CheckersGame`, `Command`

## Changes in GUI

- Appropriate refactoring for `GeneralBoard`, `CheckersGame`, `Command`

## Changes in Bot

- Appropriate refactoring for `CheckersGame`
- Changed early game evaluation function to match a different source: https://ieeexplore.ieee.org/document/7978579
- Endgame now allows some pawns (because RandomBot would never promote all pieces)
- Endgame now evaluates with sum of opening and endgame evaluations
- Tweaked weights of pieces
