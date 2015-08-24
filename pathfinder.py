#!/usr/bin/env python
# Four spaces as indentation [no tabs]
import sys
import Queue
from common import *

# ==========================================
# PathFinder A Star
# ==========================================

class PathFinder_A_Star:

    def __init__(self):
        self.path = None
        pass

    # ------------------------------------------
    # Cost
    # ------------------------------------------

    def get_cost(self, x1, y1, x2, y2):
        return 1

    # ------------------------------------------
    # Heuristic
    # ------------------------------------------

    def heuristic(self, x1, y1, x2, y2):
       return abs(x2-x1) + abs(y2-y1);

    # ------------------------------------------
    # Solve
    # ------------------------------------------

    def solve(self, sx, sy, gx, gy, map_data, map_width, map_height):
        if DEBUG:
            import time
            start_time = time.time()

        closedset = {}
        start = (sx,sy,str(sx)+'_'+str(sy),0)
        goal = (gx, gy,str(gx)+'_'+str(gy),float("inf"))
        openset = {start[2]:start}

        knows = {}
        knows[start[2]] = start

        scoredQueue = Queue.PriorityQueue()
        scoredQueue.put((start[3] + self.heuristic(start[0],start[1], goal[0],goal[1]),start))

        while len(openset) > 0:
            scored = scoredQueue.get()
            current = scored[1]
            if current[0] == goal[0] and current[1] == goal[1]:
                goal = current
                break

            del openset[current[2]]
            closedset[current[2]]=current

            succs = successors(current[0],current[1], map_data, map_width, map_height)

            for succ in succs:
                succ = succ + ( str(succ[0]) + '_' + str(succ[1]), )
                if succ[2] in closedset:
                    continue
                if succ[2] in knows:
                    succ = knows[succ[2]]
                else:
                    succ = succ + (float("inf"),);
                guess = current[3] + self.get_cost(current[0],current[1],succ[0],succ[1])

                if succ[2] not in openset or guess < succ[3]:
                    knows[succ[2]] = (succ[0],succ[1],succ[2],guess,current)#,direction(succ[0],succ[1],current[0], current[1]))
                    scoredQueue.put((succ[3] + self.heuristic(succ[0],succ[1], goal[0],goal[1]),knows[succ[2]]))
                    if succ[2] not in openset:
                        openset[succ[2]] = succ

        self.path = []
        if DEBUG:
            AStar_time = time.time() - start_time
            print 'AStar_time  =',AStar_time
        present = goal
        while len(present) > 4:
            last = present[4]
            if self.path == None or len(self.path)==0:
                #self.path = [present[5]]
                self.path = [direction(last[0],last[1],present[0], present[1])]
            else:
                #self.path.insert(0,present[5])
                self.path.insert(0,direction(last[0],last[1],present[0], present[1]))
                #self.path = [direction(last[0],last[1],present[0], present[1])] + self.path
            present=last

        if DEBUG:
            elapsed_time = time.time() - start_time
            print 'elapsed_time=',elapsed_time
        return self.path

    # ------------------------------------------
    # Get solvable
    # ------------------------------------------

    def get_solvable(self, sx = None, sy = None, gx = None, gy = None, map_data = None, map_width = None, map_height = None):
        beforeGoal = successors(sx,sy, map_data, map_width, map_height)
        if len(beforeGoal) == 0:
            return False

        afterStart = successors(gx,gy, map_data, map_width, map_height)
        if len(afterStart) == 0:
            return False

        if self.path != None or len(self.path) > 0:
            return True
        else:
            return False

    # ------------------------------------------
    # Get max tree height
    # ------------------------------------------

    def get_max_tree_height(self, sx = None, sy = None, gx = None, gy = None, map_data = None, map_width = None, map_height = None):
        # TODO return max tree height if plan found, otherwise None
        return self.get_min_moves(sx,sy,gx,gy,map_data,map_width,map_height) #is that?

    # ------------------------------------------
    # Get min moves
    # ------------------------------------------

    def get_min_moves(self, sx = None, sy = None, gx = None, gy = None, map_data = None, map_width = None, map_height = None):
        if self.path != None:
            return len(self.path)
        return None

# ------------------------------------------
# Main
# ------------------------------------------

if __name__ == '__main__':
    if len(sys.argv) == 2:
        map_name = sys.argv[1]
    else:
        map_name = DEFAULT_MAP
    print "Loading map: " + map_name
    sx, sy, gx, gy, map_data, map_width, map_height = read_map(map_name)
    solver = PathFinder_A_Star();
    plan = solver.solve(sx, sy, gx, gy, map_data, map_width, map_height)
    if DEBUG:
        solvable = solver.get_solvable(sx, sy, gx, gy, map_data, map_width, map_height)
        maxTreeH = solver.get_max_tree_height(sx, sy, gx, gy, map_data, map_width, map_height)
        minMoves = solver.get_min_moves(sx, sy, gx, gy, map_data, map_width, map_height)
        print 'Solvable=',solvable
        print 'MaxTreeH=',maxTreeH
        print 'MinMoves=',minMoves

    if plan == None:
        print "No plan was found"
    else:
        print "Plan found:"
        if DEBUG:
            print 'Plan length=',len(plan)
        else:
            for i, move in enumerate(plan):
                if move == MOVE_UP:
                    print i, ": Move Up"
                elif move == MOVE_DOWN:
                    print i, ": Move Down"
                elif move == MOVE_LEFT:
                    print i, ": Move Left"
                elif move == MOVE_RIGHT:
                    print i, ": Move Right"
                else:
                    print i, ": Movement unknown = ", move
