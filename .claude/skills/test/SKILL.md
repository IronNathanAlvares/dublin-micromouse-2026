---
name: test
description: Run every offline check before committing: firmware build for the ESP32-C6, solver tests, and the virtual robot running the real firmware through a full competition. Use before any commit or after changing firmware, sim or tools.
---

# Test everything (no robot needed)

1. `python tools/mouse.py build`: must say Build OK with no warnings.
2. `python tools/mouse.py test`: must end with ALL TESTS PASSED.
3. If motion code changed (`Motion.h`, `Hardware.h`, `config.h`), use the
   sim-tester agent for the multi-seed, multi-maze virtual-robot run.
4. Only then commit, with a message in the team's style (`fix: ...`,
   `feat: ...`, `calibrate: ...`, `test: ...`), and push.
