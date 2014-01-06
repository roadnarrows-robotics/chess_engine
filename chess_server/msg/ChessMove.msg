### Chess move message.

int16         move_num    # move number
ChessColor    player      # player/piece color
string        AN          # move in Algebraic Notation
ChessPiece    moved       # moved piece
ChessSquare   src         # from square
ChessSquare   dst         # to square
ChessPiece    captured    # captured piece, if any
bool          en_passant  # [not] an en passant move
ChessCastling castle      # [not] a castling move
ChessSquare   aux_src     # auxilliary piece location (en passant pawn, castle)
ChessSquare   aux_dst     # auxilliary destination location (castling castle)
ChessPiece    promotion   # pawn promotion, if any
bool          check       # opponent [not] placed in check
ChessColor    winner      # winner, if any
ChessResult   result      # result of move, if any
