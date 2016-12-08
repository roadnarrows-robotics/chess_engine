### Move/game results code message.

### code enumeration constants
uint8 NoResult      =  48 # '0' (ascii zero) unknown game or action result 
uint8 Ok            = 121 # 'y' good move
uint8 BadMove       = 110 # 'n' tried an illegal move
uint8 OutOfTurn     =  63 # '?' tried to move out-of-turn
uint8 Busy          =  98 # 'b' busy already making a move
uint8 InPlay        =  62 # '>' game in play
uint8 Checkmate     =  35 # '#' checkmate move / end of game state
uint8 Draw          = 100 # 'd' draw declaration / end of game state
uint8 Resign        = 114 # 'r' resign declaraion / end of game state
uint8 Disqualified  = 107 # 'k' disqualified declaraion / end of game state
uint8 NoGame        =  36 # '$' no game
uint8 GameFatal     =  33 # '!' game fatal error

# the code
uint8 code
