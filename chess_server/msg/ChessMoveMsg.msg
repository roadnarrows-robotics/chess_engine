### Chess move message.

int16             move_num    # move number
ChessColorMsg     player      # player/piece color
string            AN          # move in Algebraic Notation
ChessPieceMsg     moved       # moved piece
ChessSquareMsg    src         # from square
ChessSquareMsg    dst         # to square
ChessPieceMsg     captured    # captured piece, if any
bool              en_passant  # [not] an en passant move
ChessCastlingMsg  castle      # [not] a castling move
ChessSquareMsg    aux_src     # aux. piece location (en passant pawn, castle)
ChessSquareMsg    aux_dst     # aux. destination location (castling castle)
ChessPieceMsg     promotion   # pawn promotion, if any
bool              check       # opponent [not] placed in check
ChessColorMsg     winner      # winner, if any
ChessResultMsg    result      # result of move, if any
