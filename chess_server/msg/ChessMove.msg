### Fully qualified chess move message.

int16         move_num        # move number
ChessColor    player          # player/piece color
string        SAN             # move in FIDE Standard Algebraic Notation
ChessPos      src             # move's source (from) square
ChessPos      dst             # move's destination (to) square
ChessPiece    piece_moved     # moved piece
ChessPiece    piece_captured  # captured piece, if any
ChessPiece    piece_promoted  # pawn promoted to this piece, if any
bool          en_passant      # [not] an en passant move modifier
ChessCastling castling        # [not] a castling move modifier
ChessCheckMod check_mod       # check modifier
ChessResult   result          # result of move

ChessPos      aux_src         # aux. source square (en passant captured pawn
                              #   or castling rook)
ChessPos      aux_dst         # aux. destination square (castling rook)
ChessColor    winner          # winner, if any
