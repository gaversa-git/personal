"""
CMSC 14200, Winter 2023
Final Project - Checkers

Group Members:
    Hung Le Tran
    Giorgio Aversa
    Nico Marin Gamboa
    Johnston Liu

This file contains the implementation for the GUI of Checkers.
"""

from typing import Optional, Tuple
import sys
import click
import pygame
from bot import RandomBot, AlphaBetaBot, BotType
from const import (
    SCREEN_WIDTH, SCREEN_HEIGHT, BOARD_LENGTH,
    TLWH_END, TLWH_OFFER, TLWH_RESIGN, TLWH_ACCEPT, TLWH_DECLINE,
    CT_TURN, CT_END, CT_OFFER, CT_RESIGN, CT_ACCEPT, CT_DECLINE,
    FPS,
    RADIUS_RATIO, RING_RATIO, MIN_RING_WIDTH,
    FONT_RATIO, MIN_FONT_SIZE,
    FONT_H1, FONT_H2, FONT_PATH,
    RGB_CODES, PLAYER_TYPES)
from checkers import CheckersGame, Color, GameStatus, Command


class GUIPlayer:
    """
    Class to store information about a GUI player.

    A GUI player can either be a human player using the keyboard, or a bot.

    Public attributes:
        name (str) : Name of the player.
        bot (BotType) : Bot object.
        player_type (str) : Type of player (human | random-bot | alphabeta-bot).
    """

    name: str
    bot: Optional[BotType]
    player_type: str
    def __init__(self, n: int, player_type: str,
                 game: CheckersGame, color: Color,
                 bot_delay: Optional[int] = None):
        """
        Constructor
        """
        if player_type == "human":
            self.name = f"Player {n}"
            self.bot = None
        elif player_type == "random-bot":
            self.name = f"Random Bot (Player {n})"
            self.bot = RandomBot(game, color)
        else:
            self.name = f"AlphaBeta Bot (Player {n})"
            self.bot = (AlphaBetaBot(game, color)
                        if bot_delay is None else
                        AlphaBetaBot(game, color, bot_delay))
        self._game = game
        self.player_type = player_type


class GUICheckers:
    """
    Wrapper class to draw and play_checkers a game of Checkers.
    
    Public attributes: None
    """

    def __init__(self, size: int,
                 player1_type: str, player2_type: str,
                 bot_delay: float) -> None:
        """
        Constructor
        """
        # Meta-game attributes
        self._size = size
        self._game = CheckersGame(size//2 - 1)
        p1 = GUIPlayer(1, player1_type, self._game,
                       Color.BLACK, bot_delay)
        p2 = GUIPlayer(2, player2_type, self._game, Color.RED, bot_delay)
        self._players = [p1, p2]

        # Initialize pygame and display attributes
        pygame.init()
        pygame.display.set_caption("Checkers")
        self._screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
        self._clock = pygame.time.Clock()
        self._cl, self._rd, self._rw, self._fs = self._init_display_params()
        self._font_king = pygame.font.Font(FONT_PATH, self._fs)
        self._font_h1 = pygame.font.Font(FONT_PATH, FONT_H1)
        self._font_h2 = pygame.font.Font(FONT_PATH, FONT_H2)

        # Helper logic attributes
        self._selected_piece = None

    def _init_display_params(self) -> Tuple[int, int, int, int]:
        """
        Calculates appropriate pixel widths to render.

        Parameters: None

        Returns:
            Tuple[int, int, int, int]:
                (cell length, piece radius, ring width, font size).
        """
        cl = BOARD_LENGTH // self._size
        rd = (BOARD_LENGTH//self._size * RADIUS_RATIO) // 2
        rw = max(int(cl * RING_RATIO), MIN_RING_WIDTH)
        fs = max(int(cl * FONT_RATIO), MIN_FONT_SIZE)
        return (cl, rd, rw, fs)

    def _sq_ct(self, pos: Tuple[int, int]) -> Tuple[int, int]:
        """
        Returns center of square at position pos of grid in pixels.

        Parameters:
            pos (Tuple[int, int]) : Input grid position.

        Returns:
            Tuple[int, int]: Center of square at grid position in pixels.
        """
        return (pos[1] * self._cl + self._cl//2,
                pos[0] * self._cl + self._cl//2)

    def _mouse_pos_to_grid(self, pos: Tuple[int, int]) -> Tuple[int, int]:
        """
        Converts (top, left) mouse position to cell on grid.

        Parameters:
            pos (Tuple[int, int]) : Input mouse position.

        Returns:
            Tuple[int, int] : Corresponding grid coordinates.
        """
        return (pos[1]//self._cl, pos[0]//self._cl)

    def _center_text(self, surface: pygame.surface.Surface,
                     font: pygame.font.Font, text: str,
                     color: Tuple[int, int, int],
                     sq_ct: Tuple[int, int]) -> None:
        """
        Renders text horizontally and vertically centered at given position.

        Parameters:
            surface (pygame.surface.Surface) : surface to be rendered on
            font (pygame.font.Font) : rendered font
            color (Tuple[int, int, int]) : rendered color
            sq_ct (Tuple[int, int]) : (left, top) to be rendered at

        Returns: None
        """
        txt = pygame.font.Font.render(font, text, True, color)
        txt_pos = sq_ct[0]-txt.get_width()//2, sq_ct[1] - txt.get_height()//2
        pygame.surface.Surface.blit(surface, txt, txt_pos)

    def _draw(self) -> None:
        """
        Draws current state of the game in the window.
        
        Parameters: None
        
        Returns: None
        """
        self._screen.fill(RGB_CODES['BG'])
        # Draw board
        for row in range(self._size):
            for col in range(self._size):
                rect = pygame.Rect(col * self._cl, row *
                                   self._cl, self._cl, self._cl)
                square_color = RGB_CODES['DARK'] if (
                    (row + col) % 2 == 1) else RGB_CODES['LIGHT']
                pygame.draw.rect(self._screen,
                                 color=square_color,
                                 rect=rect)

        # Draw pieces
        for piece_poss in self._game.board.piece_positions:
            for piece_pos in piece_poss:
                piece_color = RGB_CODES['BLACK'] if self._game.board.get_color(
                    piece_pos) is Color.BLACK else RGB_CODES['RED']
                pygame.draw.circle(self._screen,
                                   color=piece_color,
                                   center=self._sq_ct(piece_pos),
                                   radius=self._rd)
                if self._game.board.is_king(piece_pos):
                    self._center_text(self._screen, self._font_king,
                                      'K', RGB_CODES['WHITE'],
                                      self._sq_ct(piece_pos))

        # Draw ring selection
        if (not self._game.can_offer_draw
                and self._game.draw_offered_turn != self._game.next()
                and self._players[self._game.turn].player_type == "human"):
            if self._selected_piece is None:
                # no piece was selected, board shows possible jumps
                # if there isn't any, then display possible moves
                if self._game.player_has_jumps():
                    if self._game.jumped is None:
                        for start_pos in self._game.player_jumps():
                            pygame.draw.circle(self._screen,
                                               color=RGB_CODES['YELLOW'],
                                               center=self._sq_ct(start_pos),
                                               radius=self._rd,
                                               width=self._rw)
                    else:
                        pygame.draw.circle(self._screen,
                                           color=RGB_CODES['YELLOW'],
                                           center=self._sq_ct(
                                               self._game.jumped),
                                           radius=self._rd,
                                           width=self._rw)
                else:
                    for start_pos in self._game.player_moves():
                        pygame.draw.circle(self._screen,
                                           color=RGB_CODES['YELLOW'],
                                           center=self._sq_ct(start_pos),
                                           radius=self._rd,
                                           width=self._rw)
            else:
                # A piece was selected
                # If player has jumps, check if selected piece has a jump
                # If player doesn't have jumps, check if there are moves
                if self._game.player_has_jumps():
                    if self._selected_piece in self._game.player_jumps():
                        pygame.draw.circle(self._screen,
                                           color=RGB_CODES['GREEN'],
                                           center=self._sq_ct(
                                               self._selected_piece),
                                           radius=self._rd,
                                           width=self._rw)
                    for target_pos in self._game.piece_jumps(
                            self._selected_piece):
                        pygame.draw.circle(self._screen,
                                           color=RGB_CODES['WHITE'],
                                           center=self._sq_ct(target_pos),
                                           radius=self._rd,
                                           width=self._rw)
                elif self._game.piece_has_moves(self._selected_piece):
                    pygame.draw.circle(self._screen,
                                       color=RGB_CODES['GREEN'],
                                       center=self._sq_ct(self._selected_piece),
                                       radius=self._rd,
                                       width=self._rw)
                    for target_pos in self._game.piece_moves(
                            self._selected_piece):
                        pygame.draw.circle(self._screen,
                                           color=RGB_CODES['WHITE'],
                                           center=self._sq_ct(target_pos),
                                           radius=self._rd,
                                           width=self._rw)

        # Turn counter, resign, request draw, accept, reject draw buttons
        self._center_text(self._screen, self._font_h1,
                          f"Turn: {Color(self._game.turn).name}",
                          RGB_CODES['BLACK'], CT_TURN)
        pygame.draw.rect(self._screen, color=RGB_CODES['SUCCESS']
                         if self._game.can_offer_draw
                         else RGB_CODES['DISABLED'], rect=TLWH_END)
        pygame.draw.rect(self._screen, color=RGB_CODES['WARNING']
                         if self._game.can_offer_draw
                         else RGB_CODES['DISABLED'], rect=TLWH_OFFER)
        pygame.draw.rect(self._screen,
                         color=RGB_CODES['DANGER'], rect=TLWH_RESIGN)
        pygame.draw.rect(self._screen, color=RGB_CODES['SUCCESS']
                         if self._game.draw_offered_turn is self._game.next()
                         else RGB_CODES['DISABLED'], rect=TLWH_ACCEPT)
        pygame.draw.rect(self._screen, color=RGB_CODES['SUCCESS']
                         if self._game.draw_offered_turn is self._game.next()
                         else RGB_CODES['DISABLED'], rect=TLWH_DECLINE)
        self._center_text(self._screen, self._font_h2,
                          "End Turn", RGB_CODES['WHITE'], CT_END)
        self._center_text(self._screen, self._font_h2,
                          "Offer Draw", RGB_CODES['WHITE'], CT_OFFER)
        self._center_text(self._screen, self._font_h2,
                          "Resign", RGB_CODES['WHITE'], CT_RESIGN)
        self._center_text(self._screen, self._font_h2,
                          "Accept Draw", RGB_CODES['WHITE'], CT_ACCEPT)
        self._center_text(self._screen, self._font_h2,
                          "Decline Draw", RGB_CODES['WHITE'], CT_DECLINE)

    def play_checkers(self) -> None:
        """
        Play checkers game.

        Parameters: None

        Returns: None
        """
        def is_in_button(tl_pos: Tuple[int, int, int, int],
                         e_pos: Tuple[int, int]) -> bool:
            """
            Checks if a mouse click hit a button given its top-left position.

            Parameters:
                tl_pos (Tuple[int, int, int, int]) : Button's t-l-w-h.
                e_pos (Tuple[int, int]) : Mouse click event's position.

            Returns:
                bool: If mouse click was in button.
            """
            return (e_pos[0] > tl_pos[0]
                    and e_pos[0] < tl_pos[0] + tl_pos[2]
                    and e_pos[1] > tl_pos[1]
                    and e_pos[1] < tl_pos[1] + tl_pos[3])

        def hit_cmd(e_pos: Tuple[int, int]) -> Command:
            """
            Returns the command that the mouse clicked on.

            Parameters:
                e_pos (Tuple[int, int]) : Mouse click event's position.

            Returns:
                Command: Command that was clicked on.
            """
            if is_in_button(TLWH_OFFER, e_pos):
                return Command.OFFER_DRAW
            if is_in_button(TLWH_RESIGN, e_pos):
                return Command.RESIGN
            if is_in_button(TLWH_ACCEPT, e_pos):
                return Command.ACCEPT_DRAW
            if is_in_button(TLWH_DECLINE, e_pos):
                return Command.DECLINE_DRAW
            if is_in_button(TLWH_END, e_pos):
                return Command.END_TURN
            return None

        while self._game.game_status() is GameStatus.ONGOING:
            self._draw()
            pygame.display.update()
            self._clock.tick(FPS)
            if self._players[self._game.turn].player_type == "human":
                # Human input
                events = pygame.event.get()
                for event in events:
                    if event.type == pygame.QUIT:
                        pygame.quit()
                        print("Game was quit!")
                        sys.exit()
                    if event.type == pygame.MOUSEBUTTONDOWN:
                        m_pos = self._mouse_pos_to_grid(event.pos)
                        if (self._game.is_in_bounds(m_pos)
                                and not self._game.can_offer_draw):
                            # user clicked on board
                            if ((self._game.jumped is None
                                and m_pos in self._game.player_jumps())
                                or (not self._game.player_has_jumps()
                                    and self._game.piece_has_moves(m_pos))
                                or (self._game.jumped is not None
                                    and m_pos == self._game.jumped)):
                                # check if a piece can be selected
                                self._selected_piece = m_pos
                            else:
                                if self._selected_piece is not None:
                                    self._game.process_command(
                                        self._selected_piece, m_pos)
                                    self._selected_piece = None
                        else:
                            # user clicked on screen chin
                            self._selected_piece = None
                            meta_cmd = hit_cmd(event.pos)
                            if meta_cmd is not None:
                                self._game.process_command(
                                    None, None, meta_cmd)
                    elif event.type == pygame.KEYDOWN:
                        # provide some shortcuts for the user
                        if event.key == pygame.K_SPACE:
                            self._game.process_command(None, None,
                                                       Command.END_TURN)
            else:
                # Bot input
                bot_seq = self._players[self._game.turn].bot.suggest_move()
                self._game.process_seq(bot_seq)
                self._game.process_command(None, None, Command.END_TURN)
        final_state = self._game.game_status()

        # Print results
        print("="*30+"CHECKERS"+"="*30)
        if final_state is GameStatus.DRAW:
            print("The match has ended in a draw!")
        else:
            print("The winner is "
                  f"{self._players[final_state.value].name} of "
                  f"color {final_state.name}!")
        print("="*30+"CHECKERS"+"="*30)


# CLI


@click.command(name="checkers-gui")
@click.option('--size',
              type=click.INT,
              default=8,
              help="Size of the board, must be an even positive integer!")
@click.option('--player1',
              type=click.Choice(PLAYER_TYPES,
                                case_sensitive=False),
              default='human')
@click.option('--player2',
              type=click.Choice(PLAYER_TYPES,
                                case_sensitive=False),
              default='human')
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
    game = GUICheckers(size, player1, player2, bot_delay)
    game.play_checkers()


if __name__ == "__main__":
    cmd()
