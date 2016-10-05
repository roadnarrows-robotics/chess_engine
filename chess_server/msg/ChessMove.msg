### Chess move message.

int16         move_num    # move number
ChessColor    player      # player/piece color
string        AN          # move in Algebraic Notation
ChessPiece    moved       # moved piece
ChessPos      src         # from square
ChessPos      dst         # to square
ChessPiece    captured    # captured piece, if any
bool          en_passant  # [not] an en passant move
ChessCastling castle      # [not] a castling move
ChessPos      aux_src     # aux. piece location (en passant pawn, castle)
ChessPos      aux_dst     # aux. destination location (castling castle)
ChessPiece    promotion   # pawn promotion, if any
bool          check       # opponent [not] placed in check
ChessColor    winner      # winner, if any
ChessResult   result      # result of move, if any
