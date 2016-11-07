### Move/game results code message.

### code enumeration constants
uint8 NoResult      =  48 # '0' (ascii zero)
uint8 Ok            = 121 # 'y' good move
uint8 BadMove       = 110 # 'n' tried an illegal move
uint8 OutOfTurn     =  63 # '?' tried to move out-of-turn
uint8 Checkmate     =  35 # '#' checkmate
uint8 Draw          = 100 # 'd' draw
uint8 Resign        = 114 # 'r' resign
uint8 Disqualified  = 107 # 'k' player disqualified
uint8 NoGame        =  36 # '$' no game
uint8 GameFatal     =  33 # '!' game fatal error

# the arguments
uint8 code
