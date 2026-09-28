"""ID-range parameters: a list of [first, last] pairs flattened, e.g.
[100, 199] or [101, 104, 251, 251]."""


def declare_ranges(node, name, default):
    values = list(node.declare_parameter(name, default).value)
    if len(values) % 2:
        raise ValueError(f"parameter {name} needs [first, last] pairs")
    return [(values[i], values[i + 1]) for i in range(0, len(values), 2)]


def in_ranges(ranges, can_id):
    return any(first <= can_id <= last for first, last in ranges)
