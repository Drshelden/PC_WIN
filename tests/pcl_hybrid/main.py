#!/usr/bin/env python3
"""
Python wrapper that mirrors main.cpp behavior using the pybind11 extension.
Usage: main.py input.(pcd|xyz|e57) [export.json]
"""
import sys
import os
import json

# make sure the extension and PCL DLLs can be found when importing
bd = os.path.abspath(os.path.join(os.path.dirname(__file__), 'build', 'Debug'))
if os.path.isdir(bd):
    # Prepend to PATH so Windows loader can find dependent DLLs (PCL etc.)
    os.environ['PATH'] = bd + os.pathsep + os.environ.get('PATH', '')
    # Prepend to sys.path so Python can find the pyd
    sys.path.insert(0, bd)

try:
    import pcl_hybrid_py as ph
except Exception as e:
    print('Failed to import pcl_hybrid_py:', e)
    print('Make sure you built the extension and that build/Debug is on PATH/sys.path')
    sys.exit(-10)

# Return codes should match the C++ program's constants when possible
LOAD_OK = 0
LOAD_NOT_FOUND = -1
LOAD_UNSUPPORTED_FORMAT = -2
LOAD_PARSE_ERROR = -3



def extract_shapes_from_points(points):
    """
    Given a list of [x, y, z] points, run ShapeFinder and return a dict:
    {
        'planes': [
            {
                'coefficients': [...],
                'critical_points': [...],
                'shape_points': [...],
                'cluster_id': int
            }, ...
        ],
        'cylinders': [
            {
                'coefficients': [...],
                'critical_points': [...],
                'shape_points': [...],
                'cluster_id': int
            }, ...
        ]
    }
    """
    import numpy as np
    pc = ph.PCWin_PointCloud()
    arr = np.array(points, dtype=np.float32)
    # create a temporary Nx3 .xyz file in memory
    import tempfile
    with tempfile.NamedTemporaryFile(mode='w+', suffix='.xyz', delete=False) as tf:
        for pt in arr:
            tf.write(f"{pt[0]} {pt[1]} {pt[2]}\n")
        tf.flush()
        rc = pc.importPoints(tf.name)
    # run shape finding
    sf = ph.ShapeFinder()
    sf.findShapes(pc)
    # walk the root shape tree and collect planes/cylinders
    def collect_shapes(shape, cluster_id=None):
        out = {'planes': [], 'cylinders': []}
        def walk(sh, cid=None):
            t = sh.get_type()
            if t == 'plane':
                entry = {
                    'coefficients': sh.get_coefficients(),
                    'critical_points': np.asarray(sh.get_critical_points_array()).tolist(),
                    'shape_points': np.asarray(sh.get_points_array()).tolist(),
                    'cluster_id': cid
                }
                out['planes'].append(entry)
            elif t == 'cylinder':
                entry = {
                    'coefficients': sh.get_coefficients(),
                    'critical_points': np.asarray(sh.get_critical_points_array()).tolist(),
                    'shape_points': np.asarray(sh.get_points_array()).tolist(),
                    'cluster_id': cid
                }
                out['cylinders'].append(entry)
            # recurse
            for c in sh.get_children():
                walk(c, cid)
        walk(shape, cluster_id)
        return out
    root = sf.get_root_shape()
    if root is not None:
        return collect_shapes(root)
    return {'planes': [], 'cylinders': []}

def traverse_shape(shape, depth=0):
    if shape is None:
        return
    indent = '  ' * depth
    t = shape.get_type()
    print(f"{indent}Shape type: {t}")
    # try to get concise metadata
    try:
        coeff = shape.get_coefficients()
        if coeff is not None:
            print(f"{indent} coefficients: {coeff}")
    except Exception:
        pass
    try:
        cps = shape.get_critical_points_array()
        if cps is not None:
            import numpy as _np
            _arr = _np.asarray(cps)
            print(f"{indent} critical points: shape={_arr.shape}")
    except Exception:
        pass
    try:
        children = shape.get_children()
        if children:
            print(f"{indent} children: {len(children)}")
            for c in children:
                traverse_shape(c, depth+1)
    except Exception:
        pass


def main(argv):
    if len(argv) < 2:
        print(f"Usage: {argv[0]} input.(pcd|xyz|e57) [export.json]")
        return -1

    infile = argv[1]
    pc = ph.PCWin_PointCloud()
    rc = pc.importPoints(infile)
    if rc == LOAD_OK:
        # Try to access number of points (watch for attribute names)
        try:
            n = pc.cloud.size() if hasattr(pc.cloud, 'size') else len(pc.cloud)
        except Exception:
            # best-effort
            n = '<unknown>'
        print(f'Loaded {n} points from {infile}')
    elif rc == LOAD_NOT_FOUND:
        print('File not found:', infile, file=sys.stderr)
        return -1
    elif rc == LOAD_UNSUPPORTED_FORMAT:
        print('Unsupported file format:', infile, file=sys.stderr)
        return -2
    else:
        print('Failed to parse or load file:', infile, file=sys.stderr)
        return -3

    sf = ph.ShapeFinder()
    rc2 = sf.findShapes(pc)
    print('Found', sf.clusters_size(), 'clusters in total')

    # Demonstrate structured dtype cluster arrays (x,y,z,nx,ny,nz)
    try:
        import numpy as np
        for i in range(sf.clusters_size()):
            sarr = sf.get_cluster_structured(pc, i)
            np_sarr = np.asarray(sarr)
            print(f'Cluster {i}: structured array shape {np_sarr.shape}, dtype={np_sarr.dtype}')
            if np_sarr.size:
                print('  first row sample:', np_sarr[0])
    except Exception as e:
        print('Structured array demo failed:', e)

    # Retrieve root shape and traverse it for a concise tree dump
    try:
        root = sf.get_root_shape()
        if root is not None:
            print('\nTraversing root shape:')
            traverse_shape(root)
        else:
            print('No root shape available')
    except Exception as e:
        print('Error retrieving/traversing root shape:', e)

    if len(argv) > 2:
        export_file = argv[2]
        try:
            root_json = sf.get_root_json()
            # parse then pretty-print to match main.cpp behavior
            try:
                parsed = json.loads(root_json)
                with open(export_file, 'w', encoding='utf-8') as f:
                    json.dump(parsed, f, indent=2)
                print('Wrote root JSON to', export_file)
            except Exception as e:
                # fallback: write raw string
                with open(export_file, 'w', encoding='utf-8') as f:
                    f.write(root_json)
                print('Warning: JSON parse/dump failed:', e, 'wrote raw string')
        except Exception as e:
            print('No root shape available to export or error:', e, file=sys.stderr)

    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
