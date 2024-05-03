"""
CMSC 14200, Winter 2023
Final Project - Checkers

Group Members:
    Hung Le Tran
    Giorgio Aversa
    Nico Marin Gamboa
    Johnston Liu

This file contains the implementation for the TUI of Checkers.
"""

import checkers as ch
import click
from checkers import Color, CheckersGame, GameStatus, Command
from typing import List, Optional
from colorama import Fore, Style
from bot import RandomBot, AlphaBetaBot, BotType


class TUIPlayer:
    """
    Class for interacting with a TUI player

    A TUI player is either a human user using a keyboard or a bot
    """

    def __init__(self, n: int, player_type: str, game: CheckersGame, 
                 color: Color, bot_delay: Optional[int] = None):
        """
        Constructor

        Creates a TUI player of specified player type, color, and number with
        the game that it plays on stored.
        If the player is a bot, stores the bot delay.

        Parameters:
            n : int : The player's number (1 or 2)
            player_type : str : "human", "random-bot", or "alphabeta-bot"
            game : CheckersGame : The game to play on
            color : Color : The player's color
            bot_delay : float : When playing as a bot, an artificial delay
                (in seconds) to wait before making a move.
        """
        if player_type == 'human':
            self.name = f"Player {n}"
            self.bot = None
        if player_type == 'random-bot':
            self.name = f"Random Bot {n}"
            self.bot = RandomBot(game, color)
        elif player_type == 'alphabeta-bot':
            self.name = f"AlphaBeta Bot {n}"
            self.bot = AlphaBetaBot(game, color, bot_delay)
        self.game = game
        self.color = color


    def validate_select_cmd(self, game: CheckersGame, cmd: str) -> bool:
        """
        Validates a command for selecting a piece on a game, where a valid
        command is a string made up of a valid row and column on the game
        mapping to a valid position that has an available move

        Parameters:
            game : CheckersGame
            cmd : str

        Returns: 
            bool : True if a cmd is in valid format
        """
        # Available column titles to compare to
        col_titles = "abcdefghijklmnopqrstuvwxyz"

        # Validates command can be split to a list of 2 or more values
        if "." not in cmd or len(cmd.split(".")) == 1:
            return False
        cmd = cmd.split(".")

        # Validates the second value in command correlates to a column
        if len(cmd[1]) != 1 or not cmd[1].isalpha():
            return False
        cmd[1] = col_titles.index(cmd[1])

        # Validates the first value in the command correlates to a row
        if not cmd[0].isnumeric():
            return False

        # Transforms the command into a position
        start = (int(cmd[0]), cmd[1])

        # Checks whether the position has any moves
        seqs = game.piece_seqs(start)
        if len(seqs) == 0:
            return False

        return True

    def get_turn(self, game: CheckersGame) -> None:
        """
        Gets a turn from the player

        If the player is a human player, prompt the player for a piece to select
        and move.
        If the player is a bot, receives a suggestion for.

        Parameters: 
            game : CheckersGame

        Returns: None
        """
        # Available column titles to compare to
        col_titles = "abcdefghijklmnopqrstuvwxyz"

        # Handles the interaction of a Human's turn
        if self.bot is None:

            # Handles if a draw has been requested
            if game.draw_offered_turn is not None:
                print("A Draw has been requested.")
                cmd = input("Respond to Draw Request (y/n) or (Resign): ")
                if cmd == 'y':
                    game.process_command((0, 0), (0, 0), Command.ACCEPT_DRAW)
                    return
                elif cmd == 'n':
                    game.process_command((0, 0), (0, 0), Command.DECLINE_DRAW)
                elif cmd == 'Resign':
                    game.process_command((0, 0), (0, 0), Command.RESIGN)
                    return
                else:
                    game.process_command((0, 0), (0, 0), Command.DECLINE_DRAW)
                    print("Input not recognized. Draw auto-declined.")
                    print()

            # Prompts the user to select a valid piece to move
            cmd = input("Select a Piece (row.col): ")
            while not self.validate_select_cmd(game, cmd):
                cmd = input("Invalid Piece, please select another Piece: ")
            cmd = cmd.split(".")
            cmd[1] = col_titles.index(cmd[1])
            start = (int(cmd[0]), cmd[1])

            # Generates a lst of sequences (jump or move) for the piece selected
            seqs = game.piece_seqs(start)
            print("Please choose a move:")
            for j, seq in enumerate(seqs):
                line = str(j+1) + ") "
                for k, pos in enumerate(seq):
                    if k == len(seq) - 1:
                        line += str(pos)
                    else:
                        line += str(pos) + " -> "
                print(line)

            # Validates the command for a selected sequence
            print()
            cmd = input("> ")
            while (not cmd.isnumeric()) or int(cmd) > len(seqs) or int(cmd) < 1:
                print()
                cmd = input("Invalid move, please select another move: ")

            # Transforms the command into a sequence to be processed
            decision = seqs[int(cmd) - 1]
            game.process_seq(decision)
            display_board(game)

            # Processes End of Turn actions
            while cmd != 'y':
                print()
                print("End Turn? (y/n) or (Offer Draw/Resign)")
                cmd = input("> ")
                if cmd == "Offer Draw" or cmd == 'Resign':
                    game.process_command((0, 0), (0, 0), Command(cmd))
                    game.process_command((0, 0), (0, 0), Command.END_TURN)
                    break
                elif cmd == 'y':
                    game.process_command((0, 0), (0, 0), Command.END_TURN)

        #Handles the interaction of a bot's turn
        else:
            
            #Handles if a draw has been requested
            if game.draw_offered_turn is not None:
                print("Draw declined.")
                game.process_command((0, 0), (0, 0), Command.DECLINE_DRAW)
            
            #Processes a bot's move
            bot_seq = self.bot.suggest_move()
            self.game.process_seq(bot_seq)
            self.game.process_command(None, None, Command.END_TURN)
            print()
            display_board(game)


def display_board(game: CheckersGame) -> None:
    """
    Prints the game to the screen

    Parameters:
        game : CheckersGame 

    Returns: None
    """
    grid = game.board.grid
    nrows = len(grid)
    ncols = len(grid[0])
    col_titles = "abcdefghijklmnopqrstuvwxyz"
    index = "   a "
    for n in range(1, ncols):
        index += col_titles[n % 26] + " "
    print(index)
    print(Fore.WHITE + "  ┌" + ("─┬" * (ncols-1)) + "─┐")

    for r in range(nrows):
        if r < 10:
            crow = " " + str(r) + "│"
        else:
            crow = str(r) + "│"
        for c in range(ncols):
            color = game.board.get_color((r, c))
            if color is Color.RED and game.board.is_king((r, c)):
                crow += Fore.RED + Style.BRIGHT + "◯"
            elif color is Color.RED:
                crow += Fore.RED + Style.BRIGHT + "●"
            elif color is Color.BLACK and game.board.is_king((r, c)):
                crow += Fore.BLACK + Style.BRIGHT + "◯"
            elif color is Color.BLACK:
                crow += Fore.BLACK + Style.BRIGHT + "●"
            elif color is Color.EMPTY:
                crow += " "
            crow += Fore.WHITE + Style.NORMAL + "│"
        print(crow)

        if r < nrows - 1:
            print(Fore.WHITE + "  ├" + ("─┼" * (ncols-1)) + "─┤")
        else:
            print(Fore.WHITE + "  └" + ("─┴" * (ncols-1)) + "─┘" + 
                Style.RESET_ALL)


def play_checkers(game: CheckersGame, players: List[TUIPlayer]) -> None:
    """
    Plays a game of Checkers on the terminal

    Paramters:
        game : CheckersGame
        players : lst[TUIPlayer]: A dictionary mapping ints representing
            turns to TUIPlayer objects

    Returns: None
    """
    # The starting player is black
    current = players[0]
    print(current.name)

    # Keep playing until there is a winner:
    while game.game_status() is GameStatus.ONGOING:
        print(f"{current.name} ({current.color.name.lower()}):")
        display_board(game)
        print()

        # Processes the turn of the current player
        current.get_turn(game)

        # Switches the turn to the other player
        current = players[game.turn]
        print()

    # Updates for the match outcome
    print()
    display_board(game)
    winner = game.game_status()
    if winner is not GameStatus.ONGOING:
        print(f"The winner is {winner.name.lower()}!")  # revise
    else:
        print("It's a tie!")


#
# Command-line interface
#

@click.command(name="checkers-tui")
@click.option('--size',
              type=click.INT,
              default=8,
              help="Size of the board, must be an even positive integer!")
@click.option('--player1',
              type=click.Choice(['human', 'random-bot', 'alphabeta-bot'],
              case_sensitive=False),
              default="human")
@click.option('--player2',
              type=click.Choice(['human', 'random-bot', 'alphabeta-bot'],
              case_sensitive=False),
              default="human")
@click.option('--bot-delay',
              type=click.FLOAT,
              default=0.1)
def cmd(size, player1, player2, bot_delay):
    """
    Takes in CLI inputs to run checkers game.
    """
    if not (isinstance(size, int) and size % 2 == 0):
        print("Invalid size, must be an even positive integer!")
        return
    game = CheckersGame(((size-2)//2))
    p1 = TUIPlayer(1, player1, game, Color.BLACK, bot_delay)
    p2 = TUIPlayer(2, player2, game, Color.RED, bot_delay)
    players = [p1, p2]
    play_checkers(game, players)


if __name__ == "__main__":
    cmd()
