def define_line(x0, y0, x1, y1):
    if abs(y1 - y0) < abs(x1 - x0):
        if x0 > x1:
            line = define_line_low(x1, y1, x0, y0)
            line.reverse()
            return line
        else:
            return define_line_low(x0, y0, x1, y1)
    else:
        if y0 > y1:
            line = define_line_high(x1, y1, x0, y0)
            line.reverse()
            return line
        else:
            return define_line_high(x0, y0, x1, y1)


def define_line_low(x0, y0, x1, y1):
    points = []
    dx = x1 - x0
    dy = y1 - y0
    yi = 1
    if dy < 0:
        yi = -1
        dy = -dy

    D = (2 * dy) - dx
    y = y0

    for x in range(x0, x1 + 1):
        points.append((x, y))

        if D > 0:
            y = y + yi
            D = D + (2 * (dy - dx))
        else:
            D = D + 2 * dy

    return points


def define_line_high(x0, y0, x1, y1):
    points = []

    dx = x1 - x0
    dy = y1 - y0
    xi = 1

    if dx < 0:
        xi = -1
        dx = -dx

    D = (2 * dx) - dy
    x = x0

    for y in range(y0, y1 + 1):
        points.append((x, y))

        if D > 0:
            x = x + xi
            D = D + (2 * (dx - dy))
        else:
            D = D + 2 * dx

    return points
