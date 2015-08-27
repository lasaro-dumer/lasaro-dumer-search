/**********************************************************************
 *  Hyrule's maze readme.txt template
 **********************************************************************/


Name: Lásaro Curtinaz Dumer
Student ID: 11112375-8


Hours to complete assignment (optional): 12


/**********************************************************************
 *  Explain briefly how you implemented the datatype for states.
 **********************************************************************/
It was implemented using tuples, containing the state's 'x' and 'y'

/**********************************************************************
 *  Explain briefly how you represented a search node
 *  (state + number of moves + previous search node).
 **********************************************************************/
A search node is a extension of the state tuple. The state tuple is (x,y).
The search node tuple extend it and is like the following:
search_node = (x,y,'x_y',cost,previous)
where:
search_node[0] = the 'x' value of the state
search_node[1] = the 'y' value of the state
search_node[2] = a name for the search node, used for indexation of the known nodes
search_node[3] = the cost to reach this node
search_node[4] = the node that reached this at the cost of [3]

/**********************************************************************
 *  Explain briefly how you detected unsolvable problems.
 **********************************************************************/
The basic detection was implemented looking the number of successors that the Player's start position and goal's position have
The more generic one is based if the algorithm was already executed, if it was and haven't found a path, then it's not solvable
The more little more 'off the box' is explained in the next question about solvable maps



/**********************************************************************
 *  If you wanted to solve random $10^6$ problem, which would you
 * prefer:  more time (say, 2x as much), more memory (say 2x as much),
 * a better priority queue (say, 2x as fast), or a better priority
 * function (say, one on the order of improvement from Hamming to
 * Manhattan)? Why?
 **********************************************************************/
More time, because the reconstructions of the path is a little time consuming, for example,
a 500x500 grid it takes around 5 seconds to find a path, to reconstruct it takes 15 seconds





/**********************************************************************
 *  If you did the extra credit, describe your algorithm briefly and
 *  state the order of growth of the running time (in the worst case)
 *  for isSolvable().
 **********************************************************************/
 So, a more 'off the box' approach that I used was simple:
 if the map has less tiles to move that what the heuristic tells, then it's unsolvable
 Of course that it didn't covers all the possibilities, but will suffice a bunch



/**********************************************************************
 *  Known bugs / limitations.
 **********************************************************************/



/**********************************************************************
 *  Describe whatever help (if any) that you received.
 *  Don't include readings, lectures, and precepts, but do
 *  include any help from people (including staff, classmates, and
 *  friends) and attribute them by name.
 **********************************************************************/
I and some classmates shared map files and compared our times, and talked about general stuff, like data structures
That classmates are João Pedro Chagas and Marcelo Lazer


/**********************************************************************
 *  Describe any serious problems you encountered.
 **********************************************************************/
Have deleted, unwilling, the previous semester's code



/**********************************************************************
 *  List any other comments here. Feel free to provide any feedback
 *  on how much you learned from doing the assignment, and whether
 *  you enjoyed doing it.
 **********************************************************************/
Regardless of the incident with previous semester's code, this one seems pretty much better,
I think that the final result is actually much faster than the previous semester
