ROUTES = [
    {"id": 0, "start": 0, "end": 9},
    {"id": 1, "start": 0, "end": 13},
    {"id": 2, "start": 0, "end": 42},
    {"id": 3, "start": 0, "end": 109},
    {"id": 4, "start": 0, "end": 57},
    {"id": 5, "start": 0, "end": 42, "parked_obstacle": 50},
    {"id": 6, "start": 0, "end": 42, "traffic": {"vehicles": 20, "seed": 1234}},
    # blocker plus a stream in the neighbouring same-direction lane, released from our
    # own start so they catch us up while we wait behind it. Exists to make the
    # lane-entry gates actually refuse: routes 5 and 6 never do (5 has an empty
    # adjacent lane, 6 has no blocker), so until now they were only ever validated
    # synthetically. Left of the blocker is oncoming, so the right lane is the only
    # candidate and one stream is enough.
    {"id": 7, "start": 0, "end": 42, "parked_obstacle": 50,
     "adjacent_traffic": {"interval_s": [1.5, 3.0], "growth": 1.35, "seed": 1234, "active": 4,
                          "behind_m": 60.0}},
]