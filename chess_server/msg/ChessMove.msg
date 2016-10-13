### Fully qualified chess move message.

int16         move_num        # move number
ChessColor    player          # player/piece color
string        AN              # move in Algebraic Notation
ChessPos      src             # from square
ChessPos      dst             # to square
ChessPiece    piece_moved     # moved piece
ChessPiece    piece_captured  # captured piece, if any
ChessPiece    piece_promoted  # pawn promoted to this piece, if any
bool          en_passant      # [not] an en passant move modifier
ChessCastling castling        # [not] a castling move modifier
bool          check           # opponent [not] placed in check modifier
ChessPos      aux_src         # aux. piece location (en passant pawn, rook)
ChessPos      aux_dst         # aux. destination location (castling rook)
ChessColor    winner          # winner, if any
ChessResult   result          # result of move
