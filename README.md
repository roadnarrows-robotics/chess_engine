chess_engine
============

A ROS package to enable the easy integration of the GNU chess engine into your
robotic applications. 

## Status
Nearing completion. (About time!) I need to add:
* <em>rqt_chess</em> visualizer
* a secondary chess engine backend
* refine the ROS messages and services.

# Summary
## What It Is
The chess_engine meta-package provides a set of libraries, headers,
and ROS nodes to facilitate the developemnt of robotic chess playing algorithms.
The end goal? Make a robotic manipulator that autonomously competes against a 
human or robot opponent, playing on a physical board with physical pieces. 

To that end, the developers of **the** perfect robotic chess playing machine
must solve the following problems:
* Locate and orient the chess board.
* Locate and recognize the chess pieces.
* Apply kinodynamic algorithms to move, grasp, and release any chess piece.
* If playing on the same board with a human/robot competitor, behave with
good chess etiquete.
(Don't try to move when someone's hand is hovering over the board,
equivocating over a move!)
* Define victory moves to be a very bad winner.
* Reset the physical game from the current board state and off-board captured
pieces.
* Add synthesized voice to distract your opponent's concentration and
entertain your judges.

### The Game
The <em>chess_server</em> ROS node supports the **full** game of chess:
* Basic moves
* Castling
* Pawn promotion
* En passant
* End of game (check/checkmate/draw/resign) 


## What It Is Not
If you are looking to develop a Deeper Blue, this repo will not aid you in
obtaining Chess Nirvana. Good luck.

## License
MIT

# Requirements
* Ubuntu 16.04 + ROS kinetic _or_ Ubuntu 14.04 + ROS indigo
* GNU Chess 6.x
* Qt 5.x

# System Reference Diagram

# Game Scenarios
Game | Player 0 | Player 1
---- | -------- | --------
Auto-Play | CE0 | CE1 black/white
Robot vs. Computer | CE0 | CE1
Robot vs. Human | CE0 | human
Robot vs. Robot | CE0 | CE1
Computer vs. Human | CE0 | human
