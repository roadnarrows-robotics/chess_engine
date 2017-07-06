### Fully qualified chess move message.

int16         move_num      # move number
ChessColor    player        # player/piece color
string        AN            # move in Algebraic Notation
ChessPos      src           # move's source (from) square
ChessPos      dst           # move's destination (to) square
ChessPiece    moved         # moved piece
ChessPiece    captured      # captured piece, if any
ChessPiece    promoted      # pawn promoted to this piece, if any
bool          en_passant    # [not] an en passant move modifier
ChessCastling castling      # [not] a castling move modifier
ChessCheckMod check         # check modifier
ChessResult   result        # result of move
