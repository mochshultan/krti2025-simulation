import binding

if __name__ == "__main__":
    # Sample data for your call
    x = 6
    y = 2.3

    answer = binding.cppmult(x, y)
    print(f"In Python: int: {x} float {y:.1f} return val {answer:.1f}")
