def sign(x):
    if x == 0:
        return 0
    else:
        return x / abs(x)


# Returns 1, 0 or -1
def cmp(a, b):
    return (a > b) - (a < b)
