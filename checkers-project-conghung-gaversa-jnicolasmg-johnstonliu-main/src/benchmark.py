"""
Bot benchmarking tool
"""
from typing import Optional, Union, Tuple, List
import sys
from datetime import datetime
from time import time
import click
from bot import RandomBot, AlphaBetaBot, BotType, MAX_DEPTH
from checkers import CheckersGame, Color, GameStatus, Command


class Logger(object):
    """
    Logger class to print both to terminal and log file.
    """

    def __init__(self):
        self.terminal = sys.stdout
        self.log = open("log.txt", "a")

    def write(self, message: str):
        """
        Prints to both terminal and log file.

        Parameters:
            message (str) : Message to be printed.
        """
        self.terminal.write(message)
        self.log.write(message)

    def flush(self):
        """
        Flush behavior.

        Parameters: None

        Returns: None
        """


sys.stdout = Logger()
DEFAULT_SIZE = 8
DEFAULT_DELAY = 0.1
DEFAULT_NGAMES = 50


class BotGame:
    """
    Play a bot vs bot game.
    """

    def __init__(self, size: int, black_red: List[str], bot_delay: float):
        """
        Constructor
        """
        assert size % 2 == 0
        self._size = size
        self._game = CheckersGame(size//2-1)
        p1 = (RandomBot(self._game, Color.BLACK)
              if black_red[0] == "random" else
              AlphaBetaBot(self._game, Color.BLACK, bot_delay))
        p2 = (RandomBot(self._game, Color.RED)
              if black_red[1] == "random" else
              AlphaBetaBot(self._game, Color.RED, bot_delay))
        self._players = [p1, p2]

    def play_game(self) -> GameStatus:
        """
        Plays out 1 game between 2 bots.

        Parameters: None

        Returns:
            GameStatus : The match result.
        """
        while self._game.game_status() is GameStatus.ONGOING:
            seq_output = self._players[self._game.turn].suggest_move()
            self._game.process_seq(seq_output)
            self._game.process_command(cmd=Command.END_TURN)
            # print(f"\tb.process_seq({seq_output})")
            # print("\tb.process_command(cmd=Command.END_TURN)")
        print("Game finished!")
        return self._game.game_status()


class BotMatch:
    """
    Play a bot vs bot match.
    """

    def __init__(self, size: int = DEFAULT_SIZE,
                 n_games: int = DEFAULT_NGAMES,
                 bot1: str = "random",
                 bot2: str = "alphabeta",
                 alternate_colors: bool = True,
                 bot_delay: float = DEFAULT_DELAY):
        """
        Constructor
        """
        self._size = size
        self._n_games = n_games
        self._bot1 = bot1
        self._bot2 = bot2
        # Names (type), Win, Draws, Losses
        self._names = [f"{bot1.capitalize()} Bot 1",
                       f"{bot2.capitalize()} Bot 2"]
        self._result = [[bot1, 0, 0, 0], [bot2, 0, 0, 0]]
        self._alternate_colors = alternate_colors
        self._bot_delay = bot_delay

    def play_match(self):
        """
        Plays the match.

        Parameters: None

        Returns: None
        """
        black_red = [self._bot1, self._bot2]
        print("Playing match...\nConfig")
        print(f"\tsize: {self._size}")
        print(f"\tn_games: {self._n_games}")
        print(f"\tbot1: {self._bot1}")
        print(f"\tbot2: {self._bot2}")
        print(f"\talternate_colors: {self._alternate_colors}")
        print(f"\tbot_delay: {self._bot_delay}")
        print(f"\tbot.MAX_DEPTH: {MAX_DEPTH}")
        start_match_time = time()
        for i in range(self._n_games):
            game = BotGame(self._size, black_red, self._bot_delay)
            print(f"Playing Game {i+1}...")
            start_game_time = time()
            game_result = game.play_game()
            if game_result is GameStatus.DRAW:
                print(f"Game {i+1}: Draw!")
                self._result[0][2] += 1
                self._result[1][2] += 1
            else:
                winner_bot = (game_result.value + i *
                              int(self._alternate_colors)) % 2
                print(f"Game {i+1}: {self._names[winner_bot]} won!")
                self._result[winner_bot][1] += 1
                self._result[1-winner_bot][3] += 1
                print(f"Duration: {(time()-start_game_time):.3f}s")
            if self._alternate_colors:
                black_red.reverse()
        print(f"Match Duration: {(time() - start_match_time):.3f}s")

    def print_stats(self):
        """
        Prints the head-to-head statistics.

        Parameters: None

        Returns: None
        """
        print("="*30 + "STATS" + "="*30)
        for i in range(2):
            print(f"{self._names[i]}: {self._result[i][1]} "
                  f"W ({self._result[i][1]/self._n_games*100:.2f}%),"
                  f" {self._result[i][2]} D "
                  f"({self._result[i][2]/self._n_games*100:.2f}%),"
                  f" {self._result[i][3]} L "
                  f"({self._result[i][3]/self._n_games*100:.2f}%)")
        print("="*28 + "END STATS" + "="*28)


@click.command(name="checkers-benchmark")
@click.option('-s', '--size',
              type=click.INT,
              default=DEFAULT_SIZE)
@click.option('-n', '--n',
              type=click.INT,
              default=DEFAULT_NGAMES)
@click.option('-b1', '--bot1',
              type=click.Choice(["random", "alphabeta"],
                                case_sensitive=False),
              default="random")
@click.option('-b2','--bot2',
              type=click.Choice(["random", "alphabeta"],
                                case_sensitive=False),
              default="alphabeta")
@click.option('--no-alt-colors', is_flag=True, default=False)
@click.option('-d', '--bot-delay',
              type=click.FLOAT,
              default=DEFAULT_DELAY)
def benchmark(size, n, bot1, bot2, no_alt_colors, bot_delay):
    """
    Runs the benchmarking process.
    """
    print("#"*100)
    print(f"Starting benchmark: {datetime.now()}")
    match = BotMatch(size, n, bot1, bot2, not no_alt_colors, bot_delay)
    match.play_match()
    match.print_stats()
    print("#"*100+"\n\n")


if __name__ == "__main__":
    benchmark()
