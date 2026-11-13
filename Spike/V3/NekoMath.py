import umath


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

# ===================[ Grid direction stuff ]===================

STEP_TUPLE_FROM_DIR = [(0, 1), (1, 0), (0, -1), (-1, 0)]

def straight_move_vec_from_4_state_dir(dir):
    if dir % 4 == 0: return  0,  1
    if dir % 4 == 1: return  1,  0
    if dir % 4 == 2: return  0, -1
    if dir % 4 == 3: return -1,  0

# ===================[ Sets, arrays and grid stuff ]===================


def for_all_grid(g, f, co = None, depth = None):
    if not isinstance(g, list) or isinstance(g, (str, bytes)) or depth == 0:
            if co is None: return f(g)
            else: return f(g, co)
    
    if co is None: return [for_all_grid(x, f,    depth= None if depth is None else (depth - 1)) for x in g]
    else         : return [for_all_grid(x, f, y, depth= None if depth is None else (depth - 1)) for x, y in zip(g, co)]

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

# ===================[ Math wrappers ]===================
e = umath.e
def pow(a, b):
    return umath.pow(a, b)

def exp(p):
    return pow(e, p)

def sigmoid(v, decay = 1):
    return 1 / (1 + exp(-v * decay))

def saturate(x, scale):
    x = max(x, 0)
    return 1 - exp(-x / scale)

def floor(v):
    return umath.floor(v)

def sign(v):
    return 1 if v > 0 else (-1 if v < 0 else 0)