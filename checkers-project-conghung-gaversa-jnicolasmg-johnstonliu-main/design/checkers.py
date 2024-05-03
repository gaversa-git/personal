"""
CMSC 14200, Winter 2023
Final Project - Checkers

Group Members:
    Hung Le Tran
    Giorgio Aversa
    Nico Marin Gamboa
    Johnston Liu

This file contains the design for the Game Logic of Checkers.

Examples:
1. Create a new Checkers board. We'll create a new Game:
    game = CheckersGame(3)

2. Check if move is feasible:
    action = game.validate_action(start, target)
    action == "Valid Jump" or action == "Valid Move"

3. Possible moves of piece:
    game.piece_moves((1,1))
    game.piece_jumps((1,1))
    
4. Possible moves of player:
    game.player_moves()
    game.player_jumps()
    
5. Check for winner:
    game.game_status()
"""


from enum import Enum
from typing import List, Set, Dict, Tuple, Optional


class Color(Enum):
    """
    Enum class for cell colors.

    Public attributes:
        BLACK = 0
        RED = 1
        EMPTY = -1
    """

    BLACK = 0
    RED = 1
    EMPTY = -1


class GameStatus(Enum):
    """
    Enum class for possible states of a game.

    Public attributes:
        BLACK = 0
            means BLACK won.
        RED = 1
            means RED won.
        DRAW = -1
            means game ended in a draw.
        ONGOING = -2
            means game is ongoing.
    """

    BLACK = 0
    RED = 1
    DRAW = -1
    ONGOING = -2


class Command(Enum):
    """
    Enum class for possible commands to pass into the process_command method.

    Public attributes:
        RESIGN = "Resign"
        OFFER_DRAW = "Offer Draw"
        ACCEPT_DRAW = "Accept Draw"
        DECLINE_DRAW = "Decline Draw"
        END_TURN = "End Turn"
    """

    RESIGN = "Resign"
    OFFER_DRAW = "Offer Draw"
    ACCEPT_DRAW = "Accept Draw"
    DECLINE_DRAW = "Decline Draw"
    END_TURN = "End Turn"


class Cell:
    """
    Represents a cell on the board.
    """

    #
    # PRIVATE ATTRIBUTES
    #

    # Color of the Cell.
    _color: Color

    # Position of the Cell on board.
    _pos: Tuple[int, int]

    # If Cell is king
    _is_king: bool

    #
    # PUBLIC METHODS
    #

    def __init__(self, color: Color, pos: Tuple[int, int],
                 is_king: bool = False):
        """
        Constructor

        Creates a Cell of specified color, position, and whether or not it
        is a king.

        Parameters:
            color : Color
            pos : (int, int)
            is_king : bool
        """
        self._color = color
        self._pos = pos
        self._is_king = is_king

    def set_color(self, color: Color) -> None:
        """
        Sets the Cell to a color.

        Parameters:
            color: Color

        Returns: None
        """
        raise NotImplementedError

    def get_color(self) -> Color:
        """
        Returns the color of a Cell.

        Parameters: None

        Returns: Color
        """
        raise NotImplementedError

    def get_pos(self) -> Tuple[int, int]:
        """
        Returns the position of a cell

        Parameters: None

        Returns: (int, int)
        """
        raise NotImplementedError

    def is_king(self) -> bool:
        """
        Returns _is_king.

        Parameters: None

        Returns: bool
        """
        raise NotImplementedError

    def set_king(self, is_king: bool) -> None:
        """
        Sets _is_king to is_king.

        Parameters:
            is_king : bool

        Returns: None
        """
        raise NotImplementedError


class GeneralBoard:
    """
    Represents a general board of size r x c, with common board operations.

    Public Attributes:
        grid : List[List[Cell]]
            A 2d list to represent a board
        piece_positions : List[List[(int, int)], List[(int, int)]]
            A list of fixed size 2 which stores the positions of all active
            pieces for both players.
    """

    #
    # PRIVATE ATTRIBUTES
    #

    # number of rows
    _r: int

    # number of columns
    _c: int

    def __init__(self, r: int, c: int):
        self._r = r
        self._c = c
        self.grid = self._init_grid()
        self.piece_positions = [[], []]

    def _init_grid(self):
        """
        Initializes an empty grid of size r x c.

        Parameters: None

        Returns: List[List[Cell]]
        """
        raise NotImplementedError

    def remove(self, pos) -> None:
        """
        Removes piece at specified position from board.

        Parameters:
            pos : (int, int)

        Returns: None
        """
        raise NotImplementedError

    def update(self, start: Tuple[int, int], target: Tuple[int, int]) -> None:
        """
        Moves piece from start position to target.

        Parameters:
            start : (int, int)
            target : (int, int)

        Returns: None
        """
        raise NotImplementedError

    def get_color(self, pos: Tuple[int, int]) -> Color:
        """
        Returns color of Cell at pos.

        Parameters:
            pos : (int, int)

        Returns: Color
        """
        raise NotImplementedError

    def is_king(self, pos: Tuple[int, int]) -> bool:
        """
        Returns whether a Cell is a king at pos.

        Parameters:
            pos : (int, int)

        Returns:
            bool : True if a cell is a king
        """
        raise NotImplementedError

    def get(self, pos: Tuple[int, int]) -> Cell:
        """
        Returns Cell at pos

        Parameters:
            pos : (int, int)

        Returns: Cell
        """
        raise NotImplementedError

    def set_color(self, pos: Tuple[int, int], color: Color) -> None:
        """
        Sets the color of the Cell at pos

        Parameters:
            pos : (int, int)

        Returns: None
        """
        raise NotImplementedError

    def set_king(self, pos: Tuple[int, int], is_king: bool) -> None:
        """
        Sets is_king attribute of Cell at pos

        Parameters:
            pos : (int, int)
            is_king : bool

        Returns: None
        """
        raise NotImplementedError

    def __str__(self) -> str:
        """
        Returns string representation of board.grid

        Parameters: None

        Returns: str
        """
        raise NotImplementedError

class CheckersGame:
    """
    Represents a game of Checkers. Handles game logic.

    Public attributes:
    n : int
        parameter for initilializing the board size
    size : int
        length/width of the board
    grid : size x size list[list[Cells]]
        Cells on board.
    piece_positions: list[list[(int, int)], list[(int, int)]]
        Each player's piece positions.
    turn : int
        Current player.
    draw_offered_turn : None or int
        The player that offered the draw (if any).
    winner : Color
        Winner if available, None otherwise
    state : GameStatus
        Current state of the game
    can_offer_draw : bool
        If a draw can be offered at the moment.
    jumped : (int, int) or None
        Position of the piece which jumped in a previous turn. If no piece
        exists then it is None.
    moves_without_capture : int
        Keeps track of how many moves have passed since the last capture of a
        piece in order to force draws and avoid infinite game durations.
    """

    #
    # PUBLIC METHODS
    #

    def __init__(self, n):
        """
        Constructor

        Initializes board of specified size with default configuration.

        Parameters:
            n : int
        """
        self.n = n
        self.size = (2 * n) + 2
        self.board = GeneralBoard(self.size, self.size)
        self._init_checkers_grid()
        self.turn = 0
        self.draw_offered_turn = None
        self.winner = None
        self.state = GameStatus.ONGOING
        self.can_offer_draw = False
        self.jumped = None
        self.moves_without_capture = 0

    def game_status(self) -> GameStatus:
        """
        Returns the game state.

        Parameters: None

        Returns: GameStatus
        """
        raise NotImplementedError

    def player_moves(self) -> Dict[Tuple[int, int], List[Tuple[int, int]]]:
        """
        Returns a dictionary containing start positions
        as keys and a list of possible target move positions as values.
        Of the current player only.

        Parameters: None

        Returns: dict{(int, int): List[(int, int)]}
        """
        raise NotImplementedError

    def player_jumps(self) -> Dict[Tuple[int, int], List[Tuple[int, int]]]:
        """
        Returns a dictionary containing start positions
        as keys and a list of possible target jump positions as values.
        Of the current player only.

        Parameters: None

        Returns: dict{(int, int): List[(int, int)]}
        """
        raise NotImplementedError

    def player_has_moves(self) -> bool:
        """
        Return whether or not the player has any available non-jump moves.
        Wrapper to check length of self._player_moves().
        Of the current player only.

        Parameters: None

        Returns: bool
        """
        raise NotImplementedError

    def player_has_jumps(self) -> bool:
        """
        Return whether or not a player has any available jumps.
        Wrapper to check length of self._player_jumps().
        Of the current player only.

        Parameters: None

        Returns: bool
        """
        raise NotImplementedError

    def piece_moves(self, start: Tuple[int, int]) -> List[Tuple[int, int]]:
        """
        Returns a list containing possible target positions
        obtainable from a move from the start position.

        Parameters:
            start : (int, int)

        Returns: list[(int, int)]
        """
        moves = self._get_moves(start)
        valid_moves = []
        for move in moves:
            if self._validate_move(start, move):
                valid_moves.append(move)
        return valid_moves

    def piece_jumps(self, start: Tuple[int, int]) -> List[Tuple[int, int]]:
        """
        Returns a list containing possible target positions
        obtainable from a jump from the start position.

        Parameters:
            start : (int, int)

        Returns: list[(int, int)]
        """
        raise NotImplementedError

    def piece_has_moves(self, pos: Tuple[int, int]) -> bool:
        """
        Returns whether or not a piece has any available non-jump moves.
        Wrapper to check length of self._piece_has_moves(pos).

        Parameters:
            pos : (int, int)

        Returns: bool
        """
        raise NotImplementedError

    def piece_has_jumps(self, pos: Tuple[int, int]) -> bool:
        """
        Returns whether or not a piece has any available jumps.
        Wrapper to check length of self._piece_jumps(pos).

        Parameters:
            pos : (int, int)

        Returns: bool
        """
        raise NotImplementedError

    def is_in_bounds(self, pos: Tuple[int, int]) -> bool:
        """
        Returns whether a position is in the board.

        Parameters:
            pos : (int, int)

        Returns: bool
        """
        raise NotImplementedError

    def is_jump(
        self, start: Tuple[int, int], target: Tuple[int, int]
    ) -> Optional[Tuple[int, int]]:
        """
        Returns the position of the cell to be removed from a jump move.

        Parameters:
            start: (int, int)
            target : (int, int)

        Returns: None or (int, int)
        """
        raise NotImplementedError

    def can_promote(self, pos: Tuple[int, int]) -> bool:
        """
        Checks whether a piece can be promoted.

        Parameters:
            pos : (int, int)

        Returns: bool
        """
        raise NotImplementedError

    def next(self) -> int:
        """
        Returns value of self.turn for the next turn.

        Parameters: None

        Returns: int
        """
        raise NotImplementedError

    def piece_seqs(self, pos: Tuple[int, int]) -> List[List[Tuple[int, int]]]:
        """
        Returns all moves and jumps of the piece

        Parameters:
            pos : (int, int)

        Returns: list[list[(int, int)]]
        """
        raise NotImplementedError

    def jump_paths(self, start: Tuple[int, int]
                   ) -> List[List[Tuple[int, int]]]:
        """
        Uses a DFS-Like algorithm to find all possible jump sequences
        for a cell.

        Parameters:
            start : (int ,int)
            removed : list[(int, int)]

        Returns: list[list[(int, int)]]
        """
        raise NotImplementedError

    def process_seq(self, seq: List[Tuple[int, int]]) -> None:
        """
        Processes a sequence of moves/jumps

        Parameters:
            seq : list[(int, int)]

        Returns: None
        """
        raise NotImplementedError

    def player_action_seqs(self):
        """
        Gives all valid action sequences for current player

        Parameters: None

        Returns : list[list[(int, int)]] : list of lists of positions the piece
        goes through in a move sequence
        """
        raise NotImplementedError

    def validate_action(self, start: Tuple[int, int],
                         target: Tuple[int, int]) -> str:
        """
        Returns whether or not a move/jump is absolutely valid.

        Parameters:
            start : (int, int)
            target : (int, int)

        Returns: str
        """
        raise NotImplementedError

    def process_command(
        self,
        start: Optional[Tuple[int, int]] = None,
        target: Optional[Tuple[int, int]] = None,
        cmd: Optional[Command] = None,
    ) -> Optional[str]:
        """
        Processes a command: move/jump or meta-game

        Parameters
            start : (int, int), optional
                The tuple representing the position of the piece to be moved.
            target : (int, int), optional
                The tuple representing the position of where the piece at start
                is to be moved to.
            cmd : Command, optional
                If a cmd is given, then the parameters for start and target are
                ignored and the method strictly evaluates the string.

        Returns: None or str
            If the command processed has no errors or produces no side-effects
            i.e. changes the GameStatus then None is returned. Otherwise a
            string containing the error or the side-effects i.e. who
            the winner is or if a draw occurred is returned.
        """
        raise NotImplementedError

    #
    # PRIVATE METHODS
    #

    def _validate_move(self, pos: Tuple[int, int], target: Tuple[int, int]
                      ) -> bool:
        """
        Returns whether or not a move/jump is "valid" by only checking
        if the positions given are within the bounds of the grid,
        if the cell at the start position belongs to the current player,
        if the cell at the target position is empty and a white square, and
        if the move inputted is a jump then it checks if there is a piece of
        the opposite color in between the two positions.

        Parameters:
            pos : (int, int)
            target : (int, int)

        Returns: bool
        """
        raise NotImplementedError

    def _init_checkers_grid(self) -> None:
        """
        Creates grid of self.size and adds pieces accordingly.

        Parameters: None

        Returns: None
        """
        raise NotImplementedError

    def _move(self, start, target) -> None:
        """
        Performs a non-jump from the start position to target.

        Parameters:
            start : (int, int)
            target : (int, int)

        Returns: None
        """
        raise NotImplementedError

    def _jump(self, start, target) -> None:
        """
        Performs a jump from the start position to target.

        Parameters:
            start : (int, int)
            target : (int, int)

        Returns: None
        """
        raise NotImplementedError

    def _get_winner(self) -> Optional[Color]:
        """
        Returns the winner of the game if there is one.
        Otherwise, returns None.

        Parameters: None

        Returns: Color or None
        """
        raise NotImplementedError

    def _get_jumps(self, start: Tuple[int, int]) -> List[Tuple[int, int]]:
        """
        Returns all possible jumps from a position regardless of
        validity.

        Parameters:
            start : (int, int)

        Returns: list[(int, int)]
        """
        raise NotImplementedError

    def _get_moves(self, start: Tuple[int, int]) -> List[Tuple[int, int]]:
        """
        Returns all possible non-jumps from a position regardless of
        validity.

        Parameters:
            start : (int, int)

        Returns: list[(int, int)]
        """
        raise NotImplementedError

    def _gen_diag(self, start: Tuple[int, int], n: int
                  ) -> List[Tuple[int, int]]:
        """
        Helper function to generate a list of diagonal indices at a distance
        n from the start position.

        Parameters:
            start : (int, int)
            n : int

        Returns: list[(int, int)]
        """
        raise NotImplementedError

    def _is_black(self, pos: Tuple[int, int]) -> bool:
        """
        Returns whether or not a cell is a black square.

        Parameters:
            pos : (int, int)

        Returns: bool
        """
        raise NotImplementedError

    def _jump_paths(
        self,
        start: Tuple[int, int],
        removed: Set[Tuple[int, int]],
        color: Color,
        king: bool,
    ) -> List[List[Tuple[int, int]]]:
        """
        Helper function for jump_paths().

        Parameters:
            start : (int, int)
            removed : list[(int, int)]
            color : Color
            king : bool

        Returns: list[list[(int, int)]]
        """
        raise NotImplementedError

    def _get_children(
        self,
        start: Tuple[int, int],
        removed: Set[Tuple[int, int]],
        color: Color,
        king: bool,
    ) -> List[Tuple[int, int]]:
        """
        Helper function for _jump_paths(). Obtains all valid jumps from a
        position on the board with the specified parameters.

        Parameters:
            start : (int, int)
            removed : set((int, int))
            color : Color
            king : bool

        Returns: list[(int, int)]
        """
        raise NotImplementedError

    def _process_meta(self, cmd: Command) -> Optional[str]:
        """
        Helper function for process_command to handle meta commands and
        modularize code.

        Parameters:
            cmd : Command

        Returns: str or None
        """
        raise NotImplementedError
