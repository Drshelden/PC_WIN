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
