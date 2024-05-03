"""
CMSC 14200, Winter 2023
Final Project - Checkers

Group Members:
    Hung Le Tran
    Giorgio Aversa
    Nico Marin Gamboa
    Johnston Liu

This file contains the implementation of an AlphaBetaBot and
a RandomBot.
"""
import copy
import random
import sys
from time import time
from typing import Tuple, List, Dict, Union
from checkers import Color, GameStatus, Command, CheckersGame

sys.setrecursionlimit(2000000000)

MAX_DEPTH = 4
MAX_UTIL = 1000000000000000000000000000000000000
MAX_MOVE_TIME = 20


def is_endgame(game: CheckersGame) -> bool:
    """
    Returns whether or not the current board has a lot of kings.
    A lot of kings is decided by whether 3 * #kings is 
    greater than #pawns
    

    Parameters:
        game : CheckersGame

    Returns: bool
    """
    kings = 0
    pawns = 0
    pieces = game.board.piece_positions[0] + game.board.piece_positions[1]
    for piece in pieces:
        if game.board.is_king(piece):
            kings += 1
        else: 
            pawns += 1
    return 3 * kings >= pawns


def eval_function(game: CheckersGame) -> int:
    """
    Gives a heuristic evaluation of the current game position
    depending on whether the game is in endgame.

    Parameters:
        game : CheckersGame

    Returns: int: Evaluation of game position
    """

    if is_endgame(game):
        return game.n * opening_eval(game) + endgame_eval(game)
    return game.n * opening_eval(game)


def opening_eval(game: CheckersGame) -> int:
    """
    Produces an evalution of the position following https://ieeexplore.ieee.org/document/7978579
    but with a generalized version of the matrix and without considering the number of columns formed

    Parameters:
        game : CheckersGame

    Returns: int : evaluation of position
    """

    (p1_pieces, p2_pieces) = game.board.piece_positions
    total = 0
    def is_in_mid_board(size: int, pos: Tuple[int, int]) -> bool:
        '''
        Helper function for determining if piece is in the middle of the board

        Args:
            size : int : size of the board 
            pos : (int, int) : position of piece
        
        Returns: bool : whether the piece is in the middle of the board

        '''
        # essentially ceil(size/4)
        m = (size + 2) // 4
        return ((m <= pos[0] <= size - m - 1) and (m <= pos[1] <= size- m - 1))

    size = game.size
    total = 0
    multiplier = size + 2
    
    e_s = 0
    e_c = 0
    e_a = 0
    e_i = 0
    for piece_pos in p1_pieces:
        e_s += 2
        if game.board.is_king(piece_pos):
            e_s += 5
        e_a += size - piece_pos[0] - 1
        if is_in_mid_board(size, piece_pos):
            e_c += 1
        if piece_pos[0] + 1 >= size // 2:
            e_i += 1

    for piece_pos in p2_pieces:
        e_s -= 2
        if game.board.is_king(piece_pos):
            e_s -= 5
        e_a -= piece_pos[0]
        if is_in_mid_board(size, piece_pos):
            e_c -= 1
        if piece_pos[0] + 1 <= size // 2:
            e_i -= 1

    total = (4 * e_s + 10 * e_c + 10 * e_a + 2 * e_i) * multiplier
    if game.turn == 0:
        return total
    return -1 * total


def endgame_eval(game: CheckersGame) -> int:
    """
    Gives an evaluation of an endgame game depending on which side 
    has more pieces and the distance from one sides pieces to another.
    This follows the evaluation function in
    https://www.cs.huji.ac.il/w~ai/projects/old/English-Draughts.pdf
    but is adjusted for board size so that we still get positive evaluations
    for the winning side and negative for the losing side. 

    Parameters:
        game : CheckersGame

    Returns: int: an evaluation of position
    """
    player_number = game.turn
    op_number = 1 - player_number
    player_pieces = game.board.piece_positions[player_number]
    op_pieces = game.board.piece_positions[op_number]
    total_distance = 0
    for friendly in player_pieces:
        for enemy in op_pieces:
            total_distance += abs(friendly[0] - enemy[0]) + \
                abs(friendly[1] - enemy[1])
    if len(player_pieces) > len(op_pieces):
        return -1 * total_distance + game.size * game.size * game.size * (len(player_pieces) - len(op_pieces))
    if len(player_pieces) == len(op_pieces):
        return 0
    return total_distance + game.size * game.size * game.size * (len(player_pieces) - len(op_pieces))


def next_game_states(curr: CheckersGame) -> Dict[Tuple[Tuple[int, int]], CheckersGame]:
    """
    Returns a dictionary mapping action sequences to games.

    Parameters:
        curr : CheckersGame

    Returns: dict{(int, int) : CheckersGame} : the next possible game states
    """
    next_states = {}

    action_seqs = curr.player_action_seqs()

    for pos_list in action_seqs:
        temp = copy.deepcopy(curr)
        temp.process_seq(pos_list)
        next_states[tuple(pos_list)] = temp
    return next_states


class RandomBot:
    """
    Bot that makes moves at random
    """

    def __init__(self, game: CheckersGame, color: Color):
        """
        Constructor

        Parameters:
            game : CheckersGame

        Returns: None
        """

        self.game = game
        self.color = color
        self.number = color.value
        self.op_number = 1 - self.number

    def suggest_move(self) -> Union[Tuple[Tuple[int, int]], Command]:
        """
        Gives a random action sequence

        Parameters: None

        Returns: list[(int, int)] : suggested action sequence
        """
        if self.game.draw_offered_turn == self.op_number:
            return Command.DECLINE_DRAW 
        return random.choice(self.game.player_action_seqs())


class AlphaBetaBot:
    """
    Bot that uses alpha-beta pruning with a two-stage eval function
    to suggest moves.
    """

    def __init__(self, game: CheckersGame, color: Color, max_move_time: int = MAX_MOVE_TIME):
        """
        Constructor

        Parameters:
            game : CheckersGame
            color : Color: the color of the Bot
            max_move_time : int

        Returns: None
        """
        self.game = game
        self.color = color
        self.max_move_time = max_move_time
        self.number = color.value
        self.op_number = 1 - self.number

    def max_value(
        self, game: CheckersGame, alpha: int, beta: int, depth: int, start_time: int
    ) -> int:
        """
        Helper function to pick max value node from next level
        of game state tree.

        Parameters:
            game : CheckersGame
            alpha : int
            beta : int
            depth : int

        Returns : int : max value of next level of game state tree
        """
        value = -MAX_UTIL
        for succ_match in next_game_states(game).values():
            value = max(
                value,
                self.alpha_beta_search(
                    succ_match, alpha, beta, depth, start_time),
            )
            if value >= beta:
                return value
            alpha = max(alpha, value)
        return value

    def min_value(
        self, game: CheckersGame, alpha: int, beta: int, depth: int, start_time: int
    ) -> int:
        """
        Helper function to pick min value node from next level
        of game state tree.

        Parameters:
            game : CheckersGame
            alpha : int
            beta : int
            depth : int

        Returns : int : min value of next level of game state tree
        """
        value = MAX_UTIL
        for succ_match in next_game_states(game).values():
            value = min(
                value,
                self.alpha_beta_search(
                    succ_match, alpha, beta, depth - 1, start_time),
            )
            if value <= alpha:
                return value
            alpha = max(alpha, value)
        return value

    def alpha_beta_search(
        self, game: CheckersGame, alpha: int, beta: int, depth: int, start_time: int
    ) -> int:
        """
        Does alpha beta search on a game by searching the next move tree,
        evaluating positions, and pruning at every step.

        Parameters:
            game : CheckersGame
            alpha : int
            beta : int
            depth : int
            start_time : int

        Returns : int : best possible position's at given depth
        """
        if game.game_status() is not GameStatus.ONGOING:
            if self.color.value == game.game_status().value:
                return MAX_UTIL
            return -MAX_UTIL
        if depth <= 0 or time() - start_time > self.max_move_time:
            return eval_function(game)
        if self.number == game.turn:
            return self.max_value(game, alpha, beta, depth, start_time)
        return self.min_value(game, alpha, beta, depth, start_time)

    def suggest_move(self) -> Union[Tuple[Tuple[int, int]], str]:
        """
        Suggests a move by evaluating positions up to a certain time limit
        with alpha-beta pruning

        Parameters:
            None

        Returns : list[(int, int)] : the suggested move
        """
        if self.game.draw_offered_turn == self.op_number:
            return "Decline Draw"
        start = time()
        best_seq = None
        for depth in [1 + j for j in range(MAX_DEPTH)]:
            if time() - start > self.max_move_time:
                break
            value = -MAX_UTIL
            next_states = next_game_states(self.game)
            for ac_seq, state in next_states.items():
                best = self.alpha_beta_search(
                    state, -MAX_UTIL, MAX_UTIL, depth, start)
                if best > value:
                    value = best
                    best_seq = ac_seq
        if best_seq is None:
            best_seq = random.choice(self.game.player_action_seqs())
        return best_seq


BotType = Union[RandomBot, AlphaBetaBot]