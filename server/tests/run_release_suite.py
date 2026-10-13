"""Run isolated release tests; no real hardware or robot commands."""
from pathlib import Path
import sys
import unittest
root=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(root),str(root/'tests')]
patterns=['test_rf_real_acquisition.py', 'test_platform_management.py', 'test_camera_ready_candidates.py', 'test_modular_capture.py', 'test_device_management.py', 'test_segments.py', 'test_quality_metrics.py', 'test_automatic_capture.py', 'test_review_workflow.py', 'test_camera_inventory.py']
suite=unittest.TestSuite()
for pattern in patterns:suite.addTests(unittest.defaultTestLoader.discover(str(root/'tests'),pattern=pattern))
result=unittest.TextTestRunner(verbosity=2).run(suite)
raise SystemExit(not result.wasSuccessful())
