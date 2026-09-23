## Known issues / TODO

### Correctness
- [ ] `move_to_target` ([src/orchestrator.py](../src/orchestrator.py)) doesn't raise or signal failure once `failure_retries` is exhausted — it just returns silently instead of surfacing that the arm never reached the target.

### Tooling
- [ ] Two separate config systems (`utils.config` from `src/config.yaml`, and `scripts/config/params.yaml` loaded ad hoc in `scripts/main.py`) with no single source of truth.
- [ ] Planner/controller default hyperparameters are duplicated across function signatures, `__main__` blocks, and `scripts/config/params.yaml`, and can drift out of sync.
- [ ] No automated test suite. validation is print/CSV/plot scripts that need eyeballing, not asserted pass/fail; nothing catches a silent regression.

### Code hygiene
- [ ] Debug leftovers to clean up: stray `print(q_goal)` in `orchestrator.py`, commented-out plotting block in `motion_planner.py`'s `__main__`, dead trailing blank lines/comments at the end of `rrt_star.py`.
- [ ] Status/debug output is all bare `print()` calls rather than `logging`, across `src/` and `scripts/`.
- [ ] Inconsistent type hints and docstrings across modules.
