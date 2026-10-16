"""
Azura's dependencie free math toolbox! 
"""

import umath as math
from collections.abc import Iterable
import os

# ===================[ Pretty visual stuff ]===================
grayscale_map = " .,-~+>=ixH#$▒▓█"

class PrettyPrint:
    def __init__(self, box = True, label = None):
        self.box = box
        self.label = label
        self.lines = []
        pass

    def line(self, s):
        self.lines.extend(s.split("\n"))

    def print(self):
        width = max([len(line) for line in self.lines if isinstance(line, str)])
        if self.box:
            print("┌" + "─" * width + "┐")
            for line in self.lines:
                if isinstance(line, str):
                    print(f"│{line:<{width}}│")
                else:
                    if line[1] == "lb": print(f"│{width * "-"}│")

            print("└" + "─" * width + "┘")

    def __add__(self, other):
        if isinstance(other, list):
            self.lines.extend(other)
        else:
            self.line(other)

        return self

    def line_break(self):
        self.lines.append(("EXTRA", "lb"))

def grayscale_char(v, min = 0):
    return grayscale_map[int(min + int(len(grayscale_map) - 1 - min) * v)]

def print_grid(grid: list, do_outline = True, print = True, square = True, min = 0):
    pp = PrettyPrint(box=True)
    g = grid.copy()
    g.reverse()
    for row in g:
        pp + "".join([str(v) for v in row])

    if print: pp.print()

def print_probabilitie_grid(grid: list, do_outline = True, print = True, square = True, min = 0):
    m = grid_max(grid)
    pp = PrettyPrint(box=True)
    g = grid.copy()
    g.reverse()
    for row in g:
        pp + "".join([grayscale_char(v / m, min = min) * (2 if square else 1) for v in row])

    if print: pp.print()

def clear_terminal():
    # 'nt' means Windows, otherwise it assumes Linux/macOS
    os.system('cls' if os.name == 'nt' else 'clear')
# ===================[ Grid direction stuff ]===================

STEP_TUPLE_FROM_DIR = [(0, 1), (1, 0), (0, -1), (-1, 0)]

def straight_move_vec_from_4_state_dir(dir):
    if dir % 4 == 0: return  0,  1
    if dir % 4 == 1: return  1,  0
    if dir % 4 == 2: return  0, -1
    if dir % 4 == 3: return -1,  0

# ===================[ Sets, arrays and grid stuff ]===================

class Counter(dict):
    '''
    Stripped down Counter from Collections. Mewo!
    '''
    # References:
    #   http://en.wikipedia.org/wiki/Multiset
    #   http://www.gnu.org/software/smalltalk/manual-base/html_node/Bag.html
    #   http://www.demo2s.com/Tutorial/Cpp/0380__set-multiset/Catalog0380__set-multiset.htm
    #   http://code.activestate.com/recipes/259174/
    #   Knuth, TAOCP Vol. II section 4.6.3

    def __init__(self, iterable=None, /, **kwds):
        super().__init__()
        if iterable is not None:
            self.update(iterable)
        if kwds:
            self.update(kwds)

    def __missing__(self, key):
        'The count of elements not in the Counter is zero.'
        # Needed so that self[missing_item] does not raise KeyError
        return 0

    def highest(self):
        m = max(self.values())
        for v in self.keys():
            if self[v] == m:
                return v

    def highest_not(self, no):
        m = 0
        r = None
        for v in self.keys():
            if self[v] > m and v not in no:
                m = self[v]
                r = v
        return r

    def least(self):
        m = min(self.values())
        for v in self.keys():
            if self[v] == m:
                return v

    def least_not(self, no):
        m = 999999999999
        r = None
        for v in self.keys():
            if self[v] < m and v not in no:
                m = self[v]
                r = v
        return r

    def copy(self):
        'Return a shallow copy.'
        return self.__class__(self)

    def __reduce__(self):
        return self.__class__, (dict(self),)

    def __delitem__(self, elem):
        'Like dict.__delitem__() but does not raise KeyError for missing values.'
        if elem in self:
            super().__delitem__(elem)



def grid_max(g):
    if not isinstance(g, Iterable) or isinstance(g, (str, bytes)):
        return g

    return max(grid_max(x) for x in g)

def grid_sum(g):
    if not isinstance(g, Iterable) or isinstance(g, (str, bytes)):
        return g

    return sum(grid_sum(x) for x in g)

def grid_mul(g, v):
    if not isinstance(g, Iterable) or isinstance(g, (str, bytes)):
        return g * v

    return [grid_mul(x, v) for x in g]

def grid_pow(g, v):
    if not isinstance(g, Iterable) or isinstance(g, (str, bytes)):
        return pow(g, v)

    return [grid_pow(x, v) for x in g]

def for_all_grid(g, f, co = None, depth = None):
    if not isinstance(g, Iterable) or isinstance(g, (str, bytes)) or depth == 0:
            if co is None: return f(g)
            else: return f(g, co)
    
    if co is None: return [for_all_grid(x, f,    depth= None if depth is None else (depth - 1)) for x in g]
    else         : return [for_all_grid(x, f, y, depth= None if depth is None else (depth - 1)) for x, y in zip(g, co)]

def grid_neighbors(grid, coord, neighboorhood = STEP_TUPLE_FROM_DIR, default = 0):
    out = []
    for step in neighboorhood:
        ax = grid
        for dim, v in enumerate(step):
            d_pos = coord[dim] + v
            if d_pos < 0 or d_pos >= len(ax):
                ax = default
                break
            
            ax = ax[d_pos]

        out.append(ax)

    return out


# ===================[ Combinatories and information theory stuff ]===================

def combinations(iterable, r):
    # combinations('ABCD', 2) --> AB AC AD BC BD CD
    # From itertools recipes
    pool = tuple(iterable)
    n = len(pool)
    if r > n:
        return
    indices = list(range(r))
    yield tuple(pool[i] for i in indices)
    while True:
        for i in reversed(range(r)):
            if indices[i] != i + n - r:
                break
        else:
            return
        indices[i] += 1
        for j in range(i + 1, r):
            indices[j] = indices[j - 1] + 1
        yield tuple(pool[i] for i in indices)

def possibilities(options, n):
    if n == 1:
        return [[v] for v in options]
    else:
        p = possibilities(options, n - 1)
        new = []
        for v in options:
            for pp in p:
                pp = pp.copy()
                pp.append(v)
                new.append(pp)
        return new

def entropy(counts):
    n = sum(counts)
    return -sum(c / n * log2(c / n) for c in counts if c)

# ===================[ Math wrappers ]===================
e = math.e
def pow(a, b):
    return math.pow(a, b)

def exp(p):
    return pow(e, p)

def log2(x):
    return math.log2(x)

def sigmoid(v, decay = 1):
    return 1 / (1 + exp(-v * decay))

def saturate(x, scale):
    x = max(x, 0)
    return 1 - exp(-x / scale)