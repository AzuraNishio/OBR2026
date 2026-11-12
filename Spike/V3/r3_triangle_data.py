"""Triangle configuration kept separately from room geometry."""

_triangle_entries = []


def set_triangles(room, triangles):
    """Associate triangles with a room without external runtime dependencies."""
    for index, entry in enumerate(_triangle_entries):
        if entry[0] is room:
            _triangle_entries[index] = (room, tuple(triangles))
            return
    _triangle_entries.append((room, tuple(triangles)))


def triangles_for(room):
    for candidate, triangles in _triangle_entries:
        if candidate is room:
            return triangles
    return ()


def is_triangle(room, x, y):
    for corner in triangles_for(room):
        if room.corner_cell(corner) == (x, y):
            return True
    return False
