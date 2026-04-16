import random

TRANSITIONS = {
    's0': {'i': 's1', 'safe': 's0', 'unsafe': 's0', 'e': 's2'},
    's1': {'i': 's1', 'safe': 's3', 'unsafe': 's2', 'e': 's2'},
    's2': {'i': 's2', 'safe': 's2', 'unsafe': 's2', 'e': 's2'},
    's3': {'i': 's3', 'safe': 's3', 'unsafe': 's2', 'e': 's0'},
}

ACCEPT = {'s0', 's3'}


def run_dfa(trace):
    state = 's0'
    for symbol in trace:
        state = TRANSITIONS[state][symbol]
    return state


def rand_safe_unsafe():
    return random.choice(['safe', 'unsafe'])


# -------- GOOD TRACE --------
def generate_good_trace():
    trace = []

    # (safe/unsafe){0..3}
    for _ in range(random.randint(0, 3)):
        trace.append(rand_safe_unsafe())

    # i{1..3} → ensure reaching s1
    for _ in range(random.randint(1, 3)):
        trace.append('i')

    # must hit s3 → at least one safe
    trace.append('safe')

    # (safe/unsafe){0..3}
    for _ in range(random.randint(0, 3)):
        trace.append(rand_safe_unsafe())

    # e{0..1}
    if random.choice([True, False]):
        trace.append('e')  # safe only if coming from s3

    return trace


# -------- BAD TRACE --------
def generate_bad_trace():
    trace = []

    # (safe/unsafe){0..3}
    for _ in range(random.randint(0, 3)):
        trace.append(rand_safe_unsafe())

    choice = random.choice(['no_i', 'break'])

    if choice == 'no_i':
        # no i → stays in s0, but e will break it
        pass
    else:
        # go to s1 but avoid reaching s3
        for _ in range(random.randint(1, 3)):
            trace.append('i')
        trace.append('unsafe')  # forces s2

    # (safe/unsafe){0..3}
    for _ in range(random.randint(0, 3)):
        trace.append(rand_safe_unsafe())

    # e{0..1} → ensure BAD
    if random.choice([True, False]):
        trace.append('e')

    return trace


# -------- MAIN --------
if random.choice([True, False]):  # 50% GOOD
    trace = generate_good_trace()
else:
    trace = generate_bad_trace()

final_state = run_dfa(trace)

for sym in trace:
    print(sym)

print()
print("GOOD" if final_state in ACCEPT else "BAD")