import math
tau = 6.28318
def reachedPoint(x1, y1, x2, y2):
  tolerance = 100;
  return (  getDist(x1, y1, x2, y2) < tolerance)


def getDist(x1, y1, x2, y2):
  return math.sqrt( float((x1-x2)**2) + float((y1-y2)**2) )

def findAverageAngle(a1, a2):
  addAngle = findAngleDiff(a2, a1)/2.0 #angle between the 2; need to consider 0 = tau
  targetAngle = (a1 + addAngle) % tau
  return targetAngle

def findAngleDiff(goal, curr): #from 0 to 2pi
  angleDiff = (goal - curr) #direction robot needs to turn
  if(abs(angleDiff) > tau/2):
      if(angleDiff < 0):
        angleDiff += tau
      else:
        angleDiff -= tau
      #angleDiff = (angleDiff- tau)#%tau
  return angleDiff #positive means goal > curr, negative means goal < curr

class Node:
    def __init__(self, info, parent):
        self.infoList = info #info is (coord, accumulatedDistance) to go around
        self.parentNode = parent

    def getCoord(self):
        return self.infoList[0]

    def getTravelledDist(self):
      return self.infoList[1]
      '''
        if(self.parentNode != -1):
            return self.infoList[2] + self.parentNode.getCost()
        else:
            return 0
      '''
    def getParent(self):
        return self.parentNode #-1 means no parent
      
class PriorityQueue():
  def __init__(self):
    self.queue = []
    
  def push(self, node, cost):
    self.queue.append((cost, node))

  def sortAndPop(self): #gets lowest cost, removes from list and returns
    self.queue.sort()
    node = self.queue.pop(0)
    return node[1] #ignore cost, return only item

  def __len__(self):
      return (len(self.queue))

  def display(self):
      for x in range(len(self.queue)):
          print self.queue[x][0], self.queue[x][1]

class Counter(dict):
    """
    A counter keeps track of counts for a set of keys.

    The counter class is an extension of the standard python
    dictionary type.  It is specialized to have number values
    (integers or floats), and includes a handful of additional
    functions to ease the task of counting data.  In particular,
    all keys are defaulted to have value 0.  Using a dictionary:

    a = {}
    print a['test']

    would give an error, while the Counter class analogue:

    >>> a = Counter()
    >>> print a['test']
    0

    returns the default 0 value. Note that to reference a key
    that you know is contained in the counter,
    you can still use the dictionary syntax:

    >>> a = Counter()
    >>> a['test'] = 2
    >>> print a['test']
    2

    This is very useful for counting things without initializing their counts,
    see for example:

    >>> a['blah'] += 1
    >>> print a['blah']
    1

    The counter also includes additional functionality useful in implementing
    the classifiers for this assignment.  Two counters can be added,
    subtracted or multiplied together.  See below for details.  They can
    also be normalized and their total count and arg max can be extracted.
    """
    def __getitem__(self, idx):
        self.setdefault(idx, 0)
        return dict.__getitem__(self, idx)
