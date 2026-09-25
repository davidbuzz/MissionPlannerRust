#!/usr/bin/env python3
"""Keeps each GUI script's expected run time, its `budget` line, in step with what the suite measures.

    tools/gui-budgets.py record <logdir> [<logdir> ...]
    tools/gui-budgets.py bump <logdir> [<logdir> ...]
    tools/gui-budgets.py retime <logdir> [<logdir> ...]

`record`: for every script that passed in the given suite log directories (tools/gui-suite.sh
-o <logdir>), the time it took from its window to its last line - the runner's "passed in N s" -
becomes its expected run time, rounded up to whole seconds with one to spare, written as
`budget N` after its `window` line (replacing an earlier budget). Scripts with no passing run
keep what they had.

`bump`: every script the given runs failed for running past its budget ("took N s; the budget is
B s", or a hard stop) gets one second more - the owner's rule of 2026-09-25: "increase budget by 1 sec for all
the ones that missed".

`retime`: every script the given runs failed for its time alone - every line of it passed and
only "took N s; the budget is B s" is the failure - gets `record`'s budget from that run, the
time rounded up plus one: the run's time is its expected time as much as a passing run's is.
For when the runner itself got slower for every script (the click tool's wider still window,
2026-09-26), where `bump`'s second at a time would take a pass per second.

The runner fails a run past its budget and stops it three seconds after that.
"""
import glob, math, os, re, sys

root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def script_path(name):
    return os.path.join(root, 'tests', 'gui', name + '.gui')


def current_budget(lines):
    for l in lines:
        m = re.match(r'^budget\s+([0-9.]+)', l)
        if m:
            return float(m.group(1))
    return 5.0


def write_budget(name, budget):
    path = script_path(name)
    if not os.path.exists(path):
        return False
    lines = open(path).read().split('\n')
    lines = [l for l in lines if not re.match(r'^budget\s', l)]
    for i, l in enumerate(lines):
        if l.startswith('window '):
            lines.insert(i + 1, f'budget {budget}')
            break
    else:
        lines.insert(0, f'budget {budget}')
    open(path, 'w').write('\n'.join(lines))
    return True


def logs(dirs):
    for logdir in dirs:
        for path in glob.glob(os.path.join(logdir, 'gui-*.log')):
            yield os.path.basename(path)[4:-4], open(path, errors='replace').read()


def record(dirs):
    times = {}
    for name, text in logs(dirs):
        m = re.search(r'passed in ([0-9.]+) s', text)
        if m:
            times[name] = max(times.get(name, 0.0), float(m.group(1)))
    if not times:
        sys.exit('no passing runs found in ' + ' '.join(dirs))
    changed = sum(write_budget(name, int(math.ceil(took)) + 1) for name, took in sorted(times.items()))
    print(f'{changed} scripts given a budget from {len(times)} passing runs')


def bump(dirs):
    missed = set()
    for name, text in logs(dirs):
        # A run over its budget says so; one the hard stop ended says only that, and missed by
        # at least the margin.
        if re.search(r'took [0-9.]+ s; the budget is [0-9.]+ s|^FAIL: hard stop', text, re.M):
            missed.add(name)
    changed = 0
    for name in sorted(missed):
        path = script_path(name)
        if not os.path.exists(path):
            continue
        budget = current_budget(open(path).read().split('\n'))
        if write_budget(name, int(math.ceil(budget)) + 1):
            changed += 1
            print(f'{name}: budget {int(math.ceil(budget))} -> {int(math.ceil(budget)) + 1}')
    print(f'{changed} scripts bumped')


def retime(dirs):
    changed = 0
    for name, text in logs(dirs):
        fails = [l for l in text.split('\n') if l.startswith('FAIL')]
        timing = [l for l in fails if re.search(r'took [0-9.]+ s; the budget is|hard stop -', l)]
        if not fails or len(timing) != len(fails):
            continue
        m = re.search(r'took ([0-9.]+) s; the budget is [0-9.]+ s', text)
        path = script_path(name)
        if not os.path.exists(path):
            continue
        if m:
            budget = int(math.ceil(float(m.group(1)))) + 1
            why = f'from its {m.group(1)} s'
        else:
            # A hard stop alone: the run ended at its budget plus the margin with no time of its
            # own, so it gets the margin and two more, for the next pass to record over.
            budget = int(math.ceil(current_budget(open(path).read().split('\n')))) + 5
            why = 'after a hard stop'
        if write_budget(name, budget):
            changed += 1
            print(f'{name}: budget {budget} {why}')
    print(f'{changed} scripts retimed')


if __name__ == '__main__':
    modes = {'record': record, 'bump': bump, 'retime': retime}
    if len(sys.argv) < 3 or sys.argv[1] not in modes:
        sys.exit(__doc__)
    modes[sys.argv[1]](sys.argv[2:])
