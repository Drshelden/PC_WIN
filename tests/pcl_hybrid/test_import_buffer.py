import pytest

def test_import_points_from_buffer():
    """Smoke test for PCWin_PointCloud.importPointsFromBuffer using a synthetic point cloud.

    This test will be skipped if the compiled extension `pcl_hybrid_py` is not importable
    in the current Python environment (common when the native module hasn't been built).
    """
    try:
        import numpy as np
    except Exception:
        pytest.skip("numpy not available in this environment")

    try:
        import pcl_hybrid_py as ph
    except Exception as e:
        pytest.skip(f"pcl_hybrid_py not available: {e}")

    # Small but non-trivial synthetic cloud
    n_points = 1024
    pts = np.random.RandomState(0).rand(n_points, 3).astype(np.float32)

    pc = ph.PCWin_PointCloud()

    # Pass the NumPy array directly as a buffer to the binding.
    rc = pc.importPointsFromBuffer(pts)
    assert rc == 0, f"importPointsFromBuffer returned error code {rc}"

    # Check that normals were computed and exposed to Python (if not exposed, skip)
    normals = getattr(pc, 'normals', None)
    if normals is None:
        pytest.skip('PCWin_PointCloud.normals not exposed to Python bindings')

    # normals should have same number of entries as input points
    assert len(normals) == n_points, f"expected {n_points} normals, got {len(normals)}"
