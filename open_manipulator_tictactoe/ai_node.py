# ai_node.py
# Fonction : best_move(board, ai='O', human='X')
# Entrée : board = liste de 9 cases, ex: ['X','O',' ',' ','O',' ','X',' ',' ']
# Sortie : index (0-8) du meilleur coup

import random
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

EMPTY = ' '
HUMAN = 'X'
AI = 'O'
CORNERS = [0, 2, 6, 8]
SIDES = [1, 3, 5, 7]
CENTER = 4
OPPOSITE = {0:8, 8:0, 2:6, 6:2}

WIN_LINES = [
    (0,1,2), (3,4,5), (6,7,8),  # lignes
    (0,3,6), (1,4,7), (2,5,8),  # colonnes
    (0,4,8), (2,4,6)            # diagonales
]

def winner(b):
    for a, c, d in WIN_LINES:
        if b[a] != EMPTY and b[a] == b[c] == b[d]:
            return b[a]
    if EMPTY not in b:
        return 'draw'
    return None

def available_moves(b):
    return [i for i, v in enumerate(b) if v == EMPTY]

def play_move(b, idx, mark):
    nb = b[:]
    nb[idx] = mark
    return nb

def find_winning_move(b, mark):
    for m in available_moves(b):
        if winner(play_move(b, m, mark)) == mark:
            return m
    return None

def count_immediate_wins(b, mark):
    cnt = 0
    for m in available_moves(b):
        if winner(play_move(b, m, mark)) == mark:
            cnt += 1
    return cnt

def find_fork_move(b, mark):
    for m in available_moves(b):
        nb = play_move(b, m, mark)
        if count_immediate_wins(nb, mark) >= 2:
            return m
    return None

def find_block_fork(b, me, opp):
    # Empêcher l’adversaire de forker
    forks = []
    for m in available_moves(b):
        nb = play_move(b, m, opp)
        if count_immediate_wins(nb, opp) >= 2:
            forks.append(m)
    if len(forks) == 1:
        return forks[0]
    # sinon, créer une menace obligeant l’adversaire à défendre
    for m in available_moves(b):
        nb = play_move(b, m, me)
        if find_winning_move(nb, me) is not None:
            return m
    return None

def best_move(b, ai='O', human='X'):
    # 1) Gagner immédiatement
    m = find_winning_move(b, ai)
    if m is not None: return m

    # 2) Bloquer une victoire adverse
    m = find_winning_move(b, human)
    if m is not None: return m

    # 3) Créer un fork
    m = find_fork_move(b, ai)
    if m is not None: return m

    # 4) Bloquer un fork adverse
    m = find_block_fork(b, ai, human)
    if m is not None: return m

    # 5) Prendre le centre
    if b[CENTER] == EMPTY:
        return CENTER

    # 6) Coin opposé si possible
    opposite_choices = [OPPOSITE[c] for c in CORNERS if b[c] == human and b[OPPOSITE[c]] == EMPTY]
    if opposite_choices:
        return random.choice(opposite_choices)

    # 7) Coin libre
    corners_free = [c for c in CORNERS if b[c] == EMPTY]
    if corners_free:
        return random.choice(corners_free)

    # 8) Côté libre
    sides_free = [s for s in SIDES if b[s] == EMPTY]
    if sides_free:
        return random.choice(sides_free)

    return None  # aucune case libre (match nul)


class TicTacToeAI(Node):
    def __init__(self):
        super().__init__('tictactoe_ai')
        self.publisher_ = self.create_publisher(String, 'ai_move', 10)
        self.subscription = self.create_subscription(
            String,
            'board_state',
            self.run_move,
            10)
        self.board = [' '] * 9
        self.get_logger().info("🤖 Nœud IA prêt : écoute /board_state, publie sur /ai_move")


    def run_move(self,msg):
        # Le message arrive sous forme "X,O, , ,O, , , ,X"
        self.board = msg.data.split(',')
        self.get_logger().info('Received board state: "%s"' % msg.data)
        
        nextmove = best_move(self.board, ai=AI, human=HUMAN)
        if nextmove is not None:
            msg_out = String()
            msg_out.data = str(nextmove)
            self.publisher_.publish(msg_out)
            self.get_logger().info('Publishing move: "%s"' % msg_out.data)
        else:
            self.get_logger().info('No moves available, game over.')

        
    
def main():
    rclpy.init()
    game_ai = TicTacToeAI()
    try:
        rclpy.spin(game_ai)
    except KeyboardInterrupt:
        pass
    game_ai.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
