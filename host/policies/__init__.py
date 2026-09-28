"""policies — control strategies for the headless runner (host/apps/run_policy.py).

A Policy.step(state) returns an Action (per-motor MIT commands and/or control
requests, or nothing). See base.py for the interface; listen_policy.py for the
do-nothing policy.
"""
