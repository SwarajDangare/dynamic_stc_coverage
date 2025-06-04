GRID_SIZE = 60
CELL_SIZE = 0.5

def cell_to_center(cell):
    i, j = cell
    x = (i + 0.5) * CELL_SIZE
    y = (j + 0.5) * CELL_SIZE
    return x, y

order = []
for j in range(60):
    row = list(range(60))
    if j % 2 == 1:
        row.reverse()
    for i in row:
        order.append(cell_to_center((i, j)))
print(order)
