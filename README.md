# pyDstarLite

This project was created firstly for the purpose of getting a better understanding for the workings of D*lite algorithm and secondly because of the lack of for this algorithm in python.
The code in python is a direct derivation, plus or minus some language implementation constraints, of the code in C++ in https://github.com/ArekSredzki/dstar-lite

The Dstar class takes start and end positions in a 2d grid and considers everything that has not been marked as obstable to be traversed.

Obstacles can be updated at any time and the replanning phase calculates the new path given the previous one.

TODO:
check priority queue

