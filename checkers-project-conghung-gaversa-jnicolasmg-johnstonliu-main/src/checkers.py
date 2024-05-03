"""
CMSC 14200, Winter 2023
Final Project - Checkers

Group Members:
    Hung Le Tran
    Giorgio Aversa
    Nico Marin Gamboa
    Johnston Liu

This file contains the implementation for the Game Logic of Checkers.
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
        grid = []
        for i in range(self._r):
            lst = []
            for j in range(self._c):
                lst.append(Cell(color=Color.EMPTY, pos=(i, j)))
            grid.append(lst)
        return grid

    def remove(self, pos) -> None:
        """
        Removes piece at specified position from board.

        Parameters:
            pos : (int, int)

        Returns: None
        """
        i = pos[0]
        j = pos[1]
        self.grid[i][j].set_color(Color.EMPTY)

    def update(self, start: Tuple[int, int], target: Tuple[int, int]) -> None:
        """
        Moves piece from start position to target.

        Parameters:
            start : (int, int)
            target : (int, int)

        Returns: None
        """
        team = self.get_color(start)
        self.set_color(target, team)
        self.piece_positions[team.value].remove(start)
        self.piece_positions[team.value].append(target)
        self.set_color(start, Color.EMPTY)
        self.set_king(target, self.is_king(start))

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
        lines = ""
        for i in range(self._r):
            for j in range(self._c):
                pos = i, j
                if self.get_color(pos) is not Color.EMPTY:
                    if self.is_king(pos):
                        if self.get_color(pos) is Color.RED:
                            lines += "R"
                        elif self.get_color(pos) is Color.BLACK:
                            lines += "B"
                    else:
                        lines += str(self.get_color(pos).value)
                else:
                    lines += "E"
            lines += "\n"
        return lines


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
        if self.state.value != GameStatus.ONGOING.value:
            return self.state
        winner = self._get_winner()
        if winner is None:
            return GameStatus.ONGOING
        return GameStatus(winner.value)

    def player_moves(self) -> Dict[Tuple[int, int], List[Tuple[int, int]]]:
        """
        Returns a dictionary containing start positions
        as keys and a list of possible target move positions as values.
        Of the current player only.

        Parameters: None

        Returns: dict{(int, int): List[(int, int)]}
        """
        starts = self.board.piece_positions[self.turn]
        dct = {}
        for start in starts:
            value = self.piece_moves(start)
            if len(value) > 0:
                dct[start] = value
        return dct

    def player_jumps(self) -> Dict[Tuple[int, int], List[Tuple[int, int]]]:
        """
        Returns a dictionary containing start positions
        as keys and a list of possible target jump positions as values.
        Of the current player only.

        Parameters: None

        Returns: dict{(int, int): List[(int, int)]}
        """
        starts = self.board.piece_positions[self.turn]
        dct = {}
        for start in starts:
            value = self.piece_jumps(start)
            if len(value) > 0:
                dct[start] = value
        return dct

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
        jumps = self._get_jumps(start)
        valid_jumps = []
        for jump in jumps:
            if self._validate_move(start, jump):
                valid_jumps.append(jump)
        return valid_jumps

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

    def is_in_bounds(self, pos: Tuple[int, int]) -> bool:
        """
        Returns whether a position is in the board.

        Parameters:
            pos : (int, int)

        Returns: bool
        """
        return (pos[0] >= 0 and pos[0] < self.size
                and pos[1] >= 0 and pos[1] < self.size)

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
        if target[0] == start[0] + 2 and target[1] == start[1] + 2:
            return start[0] + 1, start[1] + 1
        if target[0] == start[0] - 2 and target[1] == start[1] - 2:
            return start[0] - 1, start[1] - 1
        if target[0] == start[0] + 2 and target[1] == start[1] - 2:
            return start[0] + 1, start[1] - 1
        if target[0] == start[0] - 2 and target[1] == start[1] + 2:
            return start[0] - 1, start[1] + 1
        return None

    def can_promote(self, pos: Tuple[int, int]) -> bool:
        """
        Checks whether a piece can be promoted.

        Parameters:
            pos : (int, int)

        Returns: bool
        """
        if self.board.get_color(pos) is Color.BLACK:
            return pos[0] == 0 and not self.board.is_king(pos)
        if self.board.get_color(pos) is Color.RED:
            return pos[0] == self.size - 1 and not self.board.is_king(pos)
        return False

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
        return self._jump_paths(
            start, set(),
            self.board.get_color(start),
            self.board.is_king(start)
        )

    def process_seq(self, seq: List[Tuple[int, int]]) -> None:
        """
        Processes a sequence of moves/jumps

        Parameters:
            seq : list[(int, int)]

        Returns: None
        """
        for i in range(len(seq) - 1):
            self.process_command(seq[i], seq[i + 1])

    def player_action_seqs(self):
        """
        Gives all valid action sequences for current player

        Parameters: None

        Returns : list[list[(int, int)]] : list of lists of positions the piece
        goes through in a move sequence
        """
        player_actions = []

        jump_dict = self.player_jumps()
        if self.player_has_jumps():
            for piece in jump_dict:
                player_actions += self.jump_paths(piece)
            return player_actions

        move_dict = self.player_moves()
        for key, val in move_dict.items():
            for v in val:
                player_actions.append([key, v])
        return player_actions

    def validate_action(self, start: Tuple[int, int],
                         target: Tuple[int, int]) -> str:
        """
        Returns whether or not a move/jump is absolutely valid.

        Parameters:
            start : (int, int)
            target : (int, int)

        Returns: str
        """
        if self._validate_move(start, target):
            # Checks if the player is currently in a jump sequence.
            if (self.jumped is not None and
                    self.piece_has_jumps(self.jumped)):
                if (start == self.jumped and
                        target in self.piece_jumps(start)):
                    return "Valid Jump"
                return f"You must choose a valid jump from {start}."
            elif self.player_has_jumps():
                if target in self.piece_jumps(start):
                    return "Valid Jump"
                return "Invalid Jump"
            elif target in self.piece_moves(start):
                return "Valid Move"
            return "Not a possible move for this piece"
        return "Invalid Move"

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
        # Checks if a meta-command was passed
        if cmd is not None:
            return self._process_meta(cmd)

        if self.can_offer_draw:
            return "You must offer a draw or end your turn."

        # Checks if command is for an action i.e. move/jump
        if not (start is None or target is None):
            # Checks if a draw was offered by the other player
            if self.draw_offered_turn == self.next():
                return "You must either Accept or Decline the Draw"
            action = self.validate_action(start, target)
            if action == "Valid Jump":
                self._jump(start, target)
            elif action == "Valid Move":
                self._move(start, target)
            else:
                return action

        if self.moves_without_capture == 40:
            self.state = GameStatus.DRAW

        if self.game_status() is not GameStatus.ONGOING:
            return self.game_status().name
        return None

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
        if not (self.is_in_bounds(pos) and self.is_in_bounds(target)):
            return False
        if not (self.board.get_color(target) is Color.EMPTY
                and self._is_black(target)):
            return False
        if self.board.get_color(pos).value != self.turn:
            return False

        jump_target = self.is_jump(pos, target)
        if jump_target is not None:
            if self.board.get_color(jump_target) is not Color(self.next()):
                return False
        return True

    def _init_checkers_grid(self) -> None:
        """
        Creates grid of self.size and adds pieces accordingly.

        Parameters: None

        Returns: None
        """
        for i in range(self.n):
            for j in range((i + 1) % 2, self.size, 2):
                pos = (i, j)
                self.board.grid[i][j] = Cell(color=Color.RED, pos=pos)
                self.board.piece_positions[Color.RED.value].append(pos)
                pos = (self.size - i - 1, self.size - j - 1)
                self.board.grid[self.size - i - 1][self.size - j - 1] = Cell(
                    color=Color.BLACK, pos=pos
                )
                self.board.piece_positions[Color.BLACK.value].append(pos)

    def _move(self, start, target) -> None:
        """
        Performs a non-jump from the start position to target.

        Parameters:
            start : (int, int)
            target : (int, int)

        Returns: None
        """
        self.board.update(start, target)
        if self.can_promote(target):
            self.board.set_king(target, True)
        self.can_offer_draw = True
        self.moves_without_capture += 1

    def _jump(self, start, target) -> None:
        """
        Performs a jump from the start position to target.

        Parameters:
            start : (int, int)
            target : (int, int)

        Returns: None
        """
        self.board.update(start, target)
        jump_target = self.is_jump(start, target)
        if jump_target is not None:
            self.board.remove(jump_target)
            self.board.piece_positions[self.next()].remove(jump_target)

        self.jumped = target
        if self.can_promote(target):
            self.board.set_king(target, True)
            self.jumped = None
            self.can_offer_draw = True
        elif not self.piece_has_jumps(target):
            self.jumped = None
            self.can_offer_draw = True
        self.moves_without_capture = 0

    def _get_winner(self) -> Optional[Color]:
        """
        Returns the winner of the game if there is one.
        Otherwise, returns None.

        Parameters: None

        Returns: Color or None
        """
        if self.winner is not None:
            return self.winner
        if not (self.player_has_moves() or self.player_has_jumps()):
            return Color(self.next())
        return None

    def _get_jumps(self, start: Tuple[int, int]) -> List[Tuple[int, int]]:
        """
        Returns all possible jumps from a position regardless of
        validity.

        Parameters:
            start : (int, int)

        Returns: list[(int, int)]
        """
        jumps = self._gen_diag(start, 2)
        if self.board.is_king(start):
            return jumps
        if self.board.get_color(start) is Color.RED:
            return jumps[0:2]
        if self.board.get_color(start) is Color.BLACK:
            return jumps[2:]
        return []

    def _get_moves(self, start: Tuple[int, int]) -> List[Tuple[int, int]]:
        """
        Returns all possible non-jumps from a position regardless of
        validity.

        Parameters:
            start : (int, int)

        Returns: list[(int, int)]
        """
        moves = self._gen_diag(start, 1)
        if self.board.is_king(start):
            return moves
        if self.board.get_color(start) is Color.RED:
            return moves[0:2]
        if self.board.get_color(start) is Color.BLACK:
            return moves[2:]
        return []

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
        r = start[0]
        c = start[1]
        moves = [(r + n, c + n), (r + n, c - n),
                 (r - n, c + n), (r - n, c - n)]
        return moves

    def _is_black(self, pos: Tuple[int, int]) -> bool:
        """
        Returns whether or not a cell is a black square.

        Parameters:
            pos : (int, int)

        Returns: bool
        """
        return (pos[0] + pos[1]) % 2 == 1

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
        jumps = self._get_children(start, removed, color, king)
        if len(jumps) == 0:
            return [[start]]
        paths = []
        for target in jumps:
            to_remove = self.is_jump(start, target)
            if to_remove is None:
                continue
            # If the original piece was not a king, then it can be promoted.
            # When a piece gets promoted, during a jump sequence its turn
            # immediately ends. Hence, we check for this.
            if not king:
                promoted = color is Color.BLACK and target[0] == self.size - 1
                promoted = promoted or (color is Color.RED and target[0] == 0)
                if promoted:
                    paths.append([start, target])
                    continue
            # Note: the removed set differs between every path unlike a
            # traditional depth first search algorithm with a visited array.
            for path in self._jump_paths(
                target, removed.union({to_remove}), color, king
            ):
                paths.append([start] + path)
        return paths

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
        jumps = self._gen_diag(start, 2)
        if not king:
            if color is Color.RED:
                jumps = jumps[0:2]
            else:
                jumps = jumps[2:]
        valid = []
        for jump in jumps:
            if (self.is_in_bounds(jump) and
                    self.board.get_color(jump) is Color.EMPTY):
                to_remove = self.is_jump(start, jump)
                if to_remove is None or to_remove in removed:
                    continue
                if self.board.get_color(to_remove) is not Color(self.next()):
                    continue
                valid.append(jump)
        return valid

    def _process_meta(self, cmd: Command) -> Optional[str]:
        """
        Helper function for process_command to handle meta commands and
        modularize code.

        Parameters:
            cmd : Command

        Returns: str or None
        """
        if cmd is Command.RESIGN:
            self.winner = Color(self.next())
            self.state = self.game_status()

        elif cmd is Command.OFFER_DRAW:
            if self.can_offer_draw:
                self.draw_offered_turn = self.turn
                self.turn = self.next()
                self.can_offer_draw = False
                return "Offered Draw!"
            return "You must complete your turn before offering a draw."

        elif cmd is Command.END_TURN:
            if self.can_offer_draw:
                self.turn = self.next()
                self.can_offer_draw = False
                return f"Turn Ended. It is now {Color(self.turn).name}'s turn."
            return "You must complete your turn before ending your turn."

        elif cmd is Command.ACCEPT_DRAW:
            if (self.draw_offered_turn is None or
                    self.draw_offered_turn == self.turn):
                return "Can't Accept Draw, No Draw Offered"
            self.state = GameStatus.DRAW
            return "Accepted Draw!"

        elif cmd is Command.DECLINE_DRAW:
            if (self.draw_offered_turn is None or
                    self.draw_offered_turn == self.turn):
                return "Can't Decline Draw, No Draw Offered"
            self.draw_offered_turn = None
            return "Declined Draw!"
        return None
