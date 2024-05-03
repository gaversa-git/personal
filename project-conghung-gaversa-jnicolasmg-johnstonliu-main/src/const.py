"""
Necessary constants for project, mostly for GUI.
"""
# Match dimensions
SCREEN_WIDTH = 600
SCREEN_HEIGHT = 700
BOARD_LENGTH = SCREEN_WIDTH
SCREEN_CHIN = SCREEN_HEIGHT - BOARD_LENGTH

# Text and buttons positioning constants
CT_TURN = (SCREEN_WIDTH // 7, BOARD_LENGTH + SCREEN_CHIN * 3 // 11)
TLWH_END = (BOARD_LENGTH * 7 // 18, BOARD_LENGTH + SCREEN_CHIN * 6 // 11,
            BOARD_LENGTH // 6, SCREEN_CHIN * 4 // 11)
TLWH_OFFER = (BOARD_LENGTH * 7 // 18, BOARD_LENGTH + SCREEN_CHIN//11,
              BOARD_LENGTH // 6, SCREEN_CHIN * 4 // 11)
TLWH_RESIGN = (BOARD_LENGTH // 18, BOARD_LENGTH + SCREEN_CHIN * 6 // 11,
               BOARD_LENGTH // 6, SCREEN_CHIN * 4 // 11)
TLWH_ACCEPT = (BOARD_LENGTH * 13 // 18, BOARD_LENGTH + SCREEN_CHIN//11,
               BOARD_LENGTH // 6, SCREEN_CHIN * 4 // 11)
TLWH_DECLINE = (BOARD_LENGTH * 13 // 18, BOARD_LENGTH + SCREEN_CHIN * 6 // 11,
                BOARD_LENGTH // 6, SCREEN_CHIN * 4 // 11)
(CT_END, CT_OFFER, CT_RESIGN, CT_ACCEPT, CT_DECLINE) = map(
    lambda tl: (tl[0] + tl[2]//2, tl[1] + tl[3] // 2),
    (TLWH_END, TLWH_OFFER, TLWH_RESIGN, TLWH_ACCEPT, TLWH_DECLINE))

# Rendering ratios
FPS = 60
RADIUS_RATIO = 0.7
RING_RATIO = 0.035
MIN_RING_WIDTH = 1
FONT_RATIO = 0.3
MIN_FONT_SIZE = 10
FONT_H1 = 20
FONT_H2 = 14

# Miscellaneous
RGB_CODES = {
    'LIGHT': (232, 204, 177),
    'DARK': (149, 99, 43),
    'BLACK': (51, 51, 51),
    'RED': (221, 0, 0),
    'YELLOW': (255, 255, 0),
    'GREEN': (0, 204, 0),
    'WHITE': (255, 255, 255),
    'BG': (255, 255, 255),
    'SUCCESS': (40, 167, 69),
    'DANGER': (220, 53, 69),
    'WARNING': (240, 173, 78),
    'DISABLED': (108, 117, 125),
}
PLAYER_TYPES = ["human", "random-bot", "alphabeta-bot"]
FONT_PATH = "assets/OpenSans-SemiBold.ttf"
