import numpy as np
import matplotlib.pyplot as plt


def triangular(a, b, c):
    """
    Triangular membership function.
    """

    def membership(x):
        x = np.asarray(x, dtype=float)

        # Rising edge
        if a != b:
            left_slope = (x - a) / (b - a)
        else:
            left_slope = (x >= a).astype(float)  # step up immediately at a

        # Falling edge
        if b != c:
            right_slope = (c - x) / (c - b)
        else:
            right_slope = (x <= b).astype(float)  # step down immediately at b

        return np.maximum(0, np.minimum(left_slope, right_slope))

    return membership


def trapezoid(a, b, c, d):
    """
    Trapezoidal membership function.
    """

    def membership(x):
        x = np.asarray(x, dtype=float)

        # Rising edge
        if a != b:
            left_slope = (x - a) / (b - a)
        else:
            left_slope = (x >= a).astype(float)  # step up immediately at a

        # Falling edge
        if c != d:
            right_slope = (d - x) / (d - c)
        else:
            right_slope = (x <= d).astype(float)  # step down immediately at d

        mu = np.maximum(0, np.minimum(np.minimum(left_slope, 1), right_slope))
        return mu

    return membership


def gaussian(a, theta):
    """
    Gaussian membership function.
    """

    def membership(x):
        x = np.asarray(x)
        return np.exp(-0.5 * ((x - a) / theta) ** 2)

    return membership


def plot(
    membership_func, x_range=(-10, 10), num_points=200, title="Membership Function"
):
    """
    Plot any fuzzy membership function.
    """
    xs = np.linspace(*x_range, num_points)
    ys = membership_func(xs)

    plt.figure(figsize=(7, 4))
    plt.plot(xs, ys, linewidth=2)
    plt.title(title)
    plt.xlabel("x")
    plt.ylabel("Membership degree")
    plt.grid(True, linestyle="--", alpha=0.6)
    plt.ylim(-0.05, 1.05)
    plt.show()
