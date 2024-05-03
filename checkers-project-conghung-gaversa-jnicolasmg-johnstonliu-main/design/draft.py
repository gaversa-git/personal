"""
CMSC 14200, Winter 2023
Final Project Draft Design - Checkers

Group Members:
    Hung Le Tran
    Giorgio Aversa
    Nico Marin Gamboa
    Johnston Liu

"""
import copy
from enum import Enum
from typing import List, Union, Dict, Tuple, Optional


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


class MatchState(Enum):
    """
    Enum class for possible states of a match.

    Public attributes:
        BLACK = 0
            means BLACK won.
        RED = 1
            means RED won.
        DRAW = -1
            means match ended in a draw.
        ONGOING = -2
            means match is ongoing.
    """

    BLACK = 0
    RED = 1
    DRAW = -1
    ONGOING = -2


class Cell:
    """
    Represents a cell on the board.

    Public attributes:
        color : Color
            color of the Cell.
        pos : (int, int)
            position of the Cell on board.
        is_king : bool
            if Cell is king.
    """

    def __init__(self, color: Color, pos: Tuple[int, int], is_king: bool = False):
        """
        Constructor

        Creates a Cell of specified color, position,
        and whether or not it is a king.

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
        self._color = color

    def get_color(self) -> Color:
        """
        Returns the color of a Cell.

        Parameters: None

        Returns: Color
        """
        return self._color

    def get_pos(self) -> Tuple[int, int]:
        """
        Returns the position of a cell

        Parameters: None

        Returns: (int, int)
        """
        return self._pos

    def is_king(self) -> bool:
        """
        Returns _is_king.

        Parameters: None

        Returns: bool
        """
        return self._is_king

    def set_king(self, is_king: bool) -> None:
        """
        Sets _is_king to is_king.

        Parameters:
            is_king : bool

        Returns: None
        """
        self._is_king = is_king


class Board:
    """
    Represents a board of Cells.
    Handles board operations and game logic.

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
    state : MatchState
        Current state of the game
    """

    def __init__(self, n):
        """
        Constructor

        Initializes board of specified size with default configuration.

        Parameters:
            n : int
        """
        self.n = n
        self.size = (2 * n) + 2
        init = self._init_grid()
        self.grid = init[0]
        self.piece_positions = init[1]
        self.turn = 0
        self.draw_offered_turn = None
        self.winner = None
        self.state = MatchState.ONGOING
        self.can_offer_draw = False
        self.jumped = None

    def _init_grid(self) -> Tuple[List[List[Cell]], List[List[Tuple[int, int]]]]:
        """
        Creates grid of self.size and adds pieces accordingly.

        Parameters: None

        Returns: size x size list[list[Cells]], list[list[Cells], list[Cells]]
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

    def get_color(self, pos: Tuple[int, int]) -> Color:
        """
        Returns color of Cell at pos.

        Parameters:
            pos : (int, int)

        Returns: Color
        """
        return self.get(pos).get_color()

    def is_king(self, pos: Tuple[int, int]) -> bool:
        """
        Returns whether a Cell is a king at pos.

        Parameters:
            pos : (int, int)

        Returns:
            bool : True if a cell is a king
        """
        return self.get(pos).is_king()

    def get(self, pos: Tuple[int, int]) -> Cell:
        """
        Returns Cell at pos

        Parameters:
            pos : (int, int)

        Returns: Cell
        """
        return self.grid[pos[0]][pos[1]]

    def set_color(self, pos: Tuple[int, int], color: Color) -> None:
        """
        Sets the color of the Cell at pos

        Parameters:
            pos : (int, int)

        Returns: None
        """
        self.grid[pos[0]][pos[1]].set_color(color)

    def set_king(self, pos: Tuple[int, int], is_king: bool) -> None:
        """
        Sets is_king attribute of Cell at pos

        Parameters:
            pos : (int, int)
            is_king : bool

        Returns: None
        """
        self.grid[pos[0]][pos[1]].set_king(is_king)

    def __str__(self) -> str:
        """
        Returns string representation of board.grid

        Parameters: None

        Returns: str
        """
        raise NotImplementedError

    def _get_winner(self) -> Optional[Color]:
        """
        Returns the winner of the match if there is one.
        Otherwise, returns None.

        Parameters: None

        Returns: Color or None
        """
        raise NotImplementedError

    def match_state(self) -> MatchState:
        """
        Returns the match state.

        Parameters: None

        Returns: MatchState
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
        return len(self.player_moves()) > 0

    def player_has_jumps(self) -> bool:
        """
        Return whether or not a player has any available jumps.
        Wrapper to check length of self._player_jumps().
        Of the current player only.

        Parameters: None

        Returns: bool
        """
        return len(self.player_jumps()) > 0

    def piece_moves(self, start: Tuple[int, int]) -> List[Tuple[int, int]]:
        """
        Returns a list containing possible target positions
        obtainable from a move from the start position.

        Parameters:
            start : (int, int)

        Returns: list[(int, int)]
        """
        raise NotImplementedError

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
        return len(self.piece_moves(pos)) != 0

    def piece_has_jumps(self, pos: Tuple[int, int]) -> bool:
        """
        Returns whether or not a piece has any available jumps.
        Wrapper to check length of self._piece_jumps(pos).

        Parameters:
            pos : (int, int)

        Returns: bool
        """
        return len(self.piece_jumps(pos)) != 0

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

    def _gen_diag(self, start: Tuple[int, int], n: int) -> List[Tuple[int, int]]:
        """
        Helper function to generate a list of diagonal indices at a distance
        n from the start position.

        Parameters:
            start : (int, int)
            n : int

        Returns: list[(int, int)]
        """
        raise NotImplementedError

    def is_in_bounds(self, pos: Tuple[int, int]) -> bool:
        """
        Returns whether a position is in the board.

        Parameters:
            pos : (int, int)

        Returns: bool
        """
        return pos[0] >= 0 and pos[0] < self.size and pos[1] >= 0 and pos[1] < self.size

    def validate_move(self, pos: Tuple[int, int], target: Tuple[int, int]) -> bool:
        """
        Returns whether or not a move is valid only checking if the positions given are
        within the bounds of the grid, if the cell at the start position belongs to the current player,
        if the cell at the target position is empty and a white square, and if the move inputted is a jump
        if there is a piece of the opposite color in between the two positions.

        Parameters:
            pos : (int, int)
            target : (int, int)

        Returns: bool
        """
        raise NotImplementedError

    def is_black(self, pos: Tuple[int, int]) -> bool:
        """
        Returns whether or not a cell is a black square.

        Parameters:
            pos : (int, int)

        Returns: bool
        """
        return (pos[0] + pos[1]) % 2 == 1

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
        return 1 - self.turn

    def piece_seqs(self, pos: Tuple[int, int]) -> List[List[Tuple[int, int]]]:
        """
        Returns all moves and jumps of the piece 

        Parameters:
            pos : (int, int)

        Returns: list[list[(int, int)]]
        """
        if self.piece_has_jumps(pos):
            return self.jump_paths(pos)
        if self.player_has_jumps():
            return []
        return [[pos, target] for target in self.piece_moves(pos)]

    def jump_paths(self, start: Tuple[int, int]) -> List[List[Tuple[int, int]]]:
        """
        Implements DFS to find all possible jump sequences for a cell.

        Parameters:
            start : (int ,int)
            removed : list[(int, int)]

        Returns: list[list[(int, int)]]
        """
        return self._jump_paths(start, set(), self.get_color(start), self.is_king(start))

    def _jump_paths(self, start, removed, color, king):
        """
        Helper function for jump_pahts().

        Parameters:
            start : (int, int)
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
        for i in range(len(seq) - 1):
            self.process_command(seq[i], seq[i+1])

    def process_command(
        self,
        start: Optional[Tuple[int, int]],
        target: Optional[Tuple[int, int]],
        cmd: Optional[str] = None,
    ) -> Optional[str]:
        """
        Processes a command: move/jump or meta-match

        Parameters
            start : (int, int)
                The tuple representing the position of the piece to be moved.
            target : (int, int)
                The tuple representing the position of where the piece at start
                is to be moved to.
            cmd : str, optional
                If a cmd is given, then the parameters for start and target are ignored
                and the method strictly evaluates the string.

        Returns: None or str
            If the command processed has no errors or produces no side-effects i.e. changes the MatchState
            then None is returned. Otherwise a string containing the error or the side-effects i.e. who
            the winner is or if a draw occurred is returned.
        """
        raise NotImplementedError
