class Vec:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __add__(self, other):
        return Vec(self.x + other.x, self.y + other.y)

    def __sub__(self, other):
        return Vec(self.x - other.x, self.y - other.y)

    def __mul__(self, scalar):
        return Vec(self.x * scalar, self.y * scalar)

    def __rmul__(self, scalar):
        return self * scalar

    def __neg__(self):
        return Vec(-self.x, -self.y)

    def __iter__(self):
        yield self.x
        yield self.y

    def __repr__(self):
        return f"Vec({self.x}, {self.y})"

    def __getitem__(self, index):
        if index == 0:
            return self.x
        elif index == 1:
            return self.y
        else:
            raise IndexError("Vec only has two components: x and y.")