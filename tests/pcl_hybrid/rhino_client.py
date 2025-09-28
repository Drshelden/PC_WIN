#!/usr/bin/env python3
# r: numpy
"""
Rhino 3D Point Cloud Processing Client
Integrates with PCL+CGAL headless processor via WSL

This script runs inside Rhinoceros 3D and allows users to:
1. Select point cloud objects from the Rhino scene
2. Process them using competitive shape detection (planes, cylinders, cones, torus)
3. Display results back in Rhino as geometry and annotations

Requirements:
- Rhinoceros 3D with Python scripting
- WSL Ubuntu with PCL headless processor built
- Point cloud objects in Rhino scene
"""

import rhinoscriptsyntax as rs
import Rhino
import Rhino.Geometry as rg
from Rhino.Geometry import Point3d, Vector3d, PointCloud, Plane, Circle, LineCurve, PlaneSurface, Cylinder, Cone, NurbsSurface, PolylineCurve, BoundingBox
import System
import json
import os
import subprocess
import time
import tempfile
import shutil
import random
from System.Drawing import Color
import scriptcontext
try:
    import numpy as np
except Exception:
    np = None

min_points_for_shape = 100  # Minimum points required to visualize a shape

# Ensure Release build folder and vcpkg bin are on PATH and sys.path at import time
try:
    import sys
    _br = os.path.abspath(os.path.join(os.path.dirname(__file__), 'build_py39', 'Release'))
    _vcpkg_bin = r'C:\vcpkg\installed\x64-windows\bin'
    if os.path.isdir(_br):
        os.environ['PATH'] = _br + os.pathsep + _vcpkg_bin + os.pathsep + os.environ.get('PATH', '')
        if _br not in sys.path:
            sys.path.insert(0, _br)
    else:
        # try older build location
        _alt = os.path.abspath(os.path.join(os.path.dirname(__file__), 'build', 'Debug'))
        if os.path.isdir(_alt) and _alt not in sys.path:
            sys.path.insert(0, _alt)
except Exception:
    # Do not prevent Rhino from importing this module if PATH tweak fails
    pass

class RhinoPointCloudProcessor:

    def __init__(self, wsl_script_path="/mnt/c/_LOCAL/GitHub/PC_WIN/tests/pcl_hybrid/process_wsl.sh"):
        self.wsl_script_path = wsl_script_path
        # Use Windows temp directory for communication with WSL
        self.windows_input_dir = r"C:\temp\pcl_processing"
        # Expect the WSL processor to write ./data/ws_output.json under the WSL project folder
        # Map that to the Windows-accessible path via the WSL UNC share
        self.windows_output_dir = "\\\\wsl.localhost\\Ubuntu\\home\\sheldd\\pcl\\pcl_win\\data"
        self.input_file = os.path.join(self.windows_input_dir, "ws_input.xyz")  # Changed to XYZ
        self.output_file = os.path.join(self.windows_output_dir, "ws_output.json")
        
        # Create input directory if it doesn't exist
        if not os.path.exists(self.windows_input_dir):
            os.makedirs(self.windows_input_dir)
    
    def select_point_cloud(self):
        """Allow user to select a point cloud object in Rhino"""
        print("Please select a point cloud object...")
        
        # Filter for point cloud objects
        filter = Rhino.DocObjects.ObjectType.PointSet
        rc, objref = Rhino.Input.RhinoGet.GetOneObject("Select point cloud", False, filter)
        
        if rc != Rhino.Commands.Result.Success:
            print("No point cloud selected or operation cancelled.")
            return None
            
        # Get the point cloud geometry
        pointcloud = objref.Geometry()
        if not isinstance(pointcloud, rg.PointCloud):
            print("Selected object is not a point cloud.")
            return None

        print("Selected point cloud with " + str(pointcloud.Count) + " points")
        return pointcloud, objref.ObjectId
    
    def pointcloud_to_list(self, pointcloud):
        """Convert Rhino PointCloud to list of [x,y,z] coordinates"""
        points = []
        for i in range(pointcloud.Count):
            pt = pointcloud[i].Location
            points.append([float(pt.X), float(pt.Y), float(pt.Z)])
        return points
    
    def process_pointcloud(self, points, solver_type="BEST"):
        """
        Process the point cloud using the built Windows Release Python bindings directly
        (pcl_hybrid_py). This avoids calling a WSL script and runs ShapeFinder in-process.
        Returns the same dict format as `extract_shapes_from_points` used elsewhere.
        """
        print("Processing " + str(len(points)) + " points via local bindings...")
        # Ensure Release build and vcpkg bins are on PATH and sys.path so imports and DLL loading succeed
        try:
            import sys
            import glob
            import importlib
            # Guarded imports: some embedded Pythons (Rhino) lack importlib.util or importlib.machinery
            try:
                import importlib.util as _importlib_util
            except Exception:
                _importlib_util = None
            try:
                import importlib.machinery as _importlib_machinery
            except Exception:
                _importlib_machinery = None

            br = os.path.abspath(os.path.join(os.path.dirname(__file__), 'build_py39', 'Release'))
            vcpkg_bin = r'C:\vcpkg\installed\x64-windows\bin'

            # Candidate directories to probe for the compiled extension
            candidate_dirs = [
                br,
                os.path.abspath(os.path.join(os.path.dirname(__file__), 'build_py39', 'Debug')),
                os.path.abspath(os.path.join(os.path.dirname(__file__), 'build', 'Debug')),
                os.path.abspath(os.path.join(os.path.dirname(__file__), 'build', 'Release')),
            ]

            for d in candidate_dirs:
                if os.path.isdir(d):
                    os.environ['PATH'] = d + os.pathsep + vcpkg_bin + os.pathsep + os.environ.get('PATH', '')
                    if d not in sys.path:
                        sys.path.insert(0, d)

            # Try normal import first
            try:
                import pcl_hybrid_py as ph
            except Exception:
                # If normal import failed, try to find a matching .pyd and load it directly
                pyd_paths = []
                for d in candidate_dirs:
                    try:
                        pyd_paths.extend(glob.glob(os.path.join(d, 'pcl_hybrid_py*.pyd')))
                    except Exception:
                        pass

                # fallback: recursive search inside the module folder
                if not pyd_paths:
                    pyd_paths = glob.glob(os.path.join(os.path.dirname(__file__), '**', 'pcl_hybrid_py*.pyd'), recursive=True)

                if pyd_paths:
                    pyd = pyd_paths[0]
                    try:
                        # Try using importlib.machinery.ExtensionFileLoader where available
                        loader = None
                        if _importlib_machinery is not None:
                            loader = _importlib_machinery.ExtensionFileLoader('pcl_hybrid_py', pyd)
                        # Some embedded Python runtimes (older Rhino/Python) may not expose importlib.util
                        if loader is not None and _importlib_util is not None:
                            spec = _importlib_util.spec_from_loader(loader.name, loader)
                            ph = _importlib_util.module_from_spec(spec)
                            loader.exec_module(ph)
                            sys.modules['pcl_hybrid_py'] = ph
                        else:
                            # Fallback for older runtimes: try imp.load_dynamic
                            try:
                                import imp
                                ph = imp.load_dynamic('pcl_hybrid_py', pyd)
                                sys.modules['pcl_hybrid_py'] = ph
                            except Exception:
                                # Last resort: insert directory to sys.path and use normal import
                                ddir = os.path.dirname(pyd)
                                if ddir not in sys.path:
                                    sys.path.insert(0, ddir)
                                ph = __import__('pcl_hybrid_py')
                    except Exception as e:
                        raise ImportError('Failed to load pcl_hybrid_py from "' + pyd + '": ' + str(e))
                else:
                    raise ImportError('pcl_hybrid_py compiled extension not found in candidate build directories')
        except Exception as e:
            error_msg = 'Failed to import Release bindings (pcl_hybrid_py): ' + str(e)
            print(error_msg)
            return {"error": error_msg}

        # create a PCWin_PointCloud and import points from an in-memory float32 buffer (x,y,z...)
        try:
            pc = ph.PCWin_PointCloud()

            # Try to use numpy for efficient buffer construction; fall back to array
            buf = None
            try:
                import numpy as np
                arr = np.asarray(points, dtype=np.float32)
                if arr.ndim == 2 and arr.shape[1] >= 3:
                    flat = arr[:, :3].reshape(-1)
                else:
                    flat = arr.reshape(-1)
                buf = flat
            except Exception:
                # fallback: use array('f')
                from array import array
                a = array('f')
                for p in points:
                    a.append(float(p[0])); a.append(float(p[1])); a.append(float(p[2]))
                buf = memoryview(a)

            # pybind11 binding expects a single buffer argument; the C++ side determines point count
            rc = pc.importPointsFromBuffer(buf)
            if rc != 0:
                return {"error": "importPointsFromBuffer failed with code " + str(rc)}
        except Exception as e:
            return {"error": "Failed to import points into PCWin_PointCloud: " + str(e)}

        # run shape finding
        try:
            sf = ph.ShapeFinder()
            print("Calling ShapeFinder.findShapes(pc)...")
            sf.findShapes(pc)
        except Exception as e:
            error_msg = "ShapeFinder failed: " + str(e)
            print(error_msg)
            return {"error": error_msg}

        # First try: traverse the Shape tree directly via the ShapeFinder bindings
        try:
            get_root_shape = getattr(sf, 'get_root_shape', None)
            if callable(get_root_shape):
                root = get_root_shape()
                if root is not None:
                    results_from_shapes = {'planes': [], 'cylinders': []}

                    def walk_shape(node):
                        try:
                            ntype = node.get_type().lower()
                        except Exception:
                            ntype = ''

                        # coefficients (if available)
                        coeffs = None
                        try:
                            if hasattr(node, 'get_coefficients'):
                                coeffs = node.get_coefficients()
                        except Exception:
                            coeffs = None

                        # critical points (numpy array or list)
                        cps = []
                        try:
                            if hasattr(node, 'get_critical_points_array'):
                                cp = node.get_critical_points_array()
                                if np is not None and hasattr(cp, 'tolist'):
                                    cps = cp.tolist()
                                else:
                                    # fallback: attempt list conversion
                                    try:
                                        cps = list(map(list, cp))
                                    except Exception:
                                        cps = []
                        except Exception:
                            cps = []

                        # shape points (prefer numpy array when available)
                        pts = []
                        try:
                            if hasattr(node, 'get_points_array'):
                                pa = node.get_points_array()
                                if np is not None and hasattr(pa, 'tolist'):
                                    # keep numpy array available for downstream use, but also
                                    # produce a Python list for Rhino geometry creation
                                    pts = pa.tolist()
                                else:
                                    try:
                                        pts = list(map(list, pa))
                                    except Exception:
                                        pts = []
                        except Exception:
                            pts = []

                        entry = {
                            'coefficients': coeffs,
                            'critical_points': cps,
                            'shape_points': pts,
                            'cluster_id': None
                        }

                        if 'plane' in ntype:
                            results_from_shapes['planes'].append(entry)
                        elif 'cylinder' in ntype:
                            results_from_shapes['cylinders'].append(entry)

                        # recurse children
                        try:
                            children = node.get_children() or []
                            for c in children:
                                walk_shape(c)
                        except Exception:
                            pass

                    walk_shape(root)
                    return results_from_shapes
        except Exception:
            # If anything fails, fall back to the JSON path below
            pass

        # Fallback: use get_root_json() as the single source of truth for shapes
        try:
            get_root_json = getattr(sf, 'get_root_json', None)
            if not callable(get_root_json):
                return {'planes': [], 'cylinders': []}

            raw = get_root_json()
            if isinstance(raw, bytes):
                raw = raw.decode('utf-8', errors='replace')
            parsed = json.loads(raw)

            # convert parsed GenericShape tree into flat results dict
            results_from_json = {'planes': [], 'cylinders': []}

            def walk_json(node):
                if not isinstance(node, dict):
                    return
                typ = node.get('type', '').lower()
                coeffs = node.get('coefficients') or node.get('coeff') or node.get('coefs') or None
                cps = node.get('critical_points') or node.get('criticalPoints') or node.get('critical') or []
                pts = node.get('points') or node.get('shape_points') or node.get('shapePoints') or []
                entry = {
                    'coefficients': coeffs,
                    'critical_points': cps,
                    'shape_points': pts,
                    'cluster_id': node.get('cluster_id') or node.get('cluster') or None
                }
                if 'plane' in typ:
                    results_from_json['planes'].append(entry)
                elif 'cylinder' in typ:
                    results_from_json['cylinders'].append(entry)
                for c in node.get('children', []) or []:
                    walk_json(c)

            if isinstance(parsed, dict) and 'children' in parsed:
                for c in parsed.get('children', []) or []:
                    walk_json(c)
            else:
                walk_json(parsed)

            return results_from_json
        except Exception:
            return {'planes': [], 'cylinders': []}
    
    def visualize_plane(self, coefficients, critical_points, shape_points, cluster_id, i):
        if len(shape_points) < min_points_for_shape:
            print("  Skipping plane visualization due to insufficient points (" + str(len(shape_points)) + " < " + str(min_points_for_shape) + ")")
            return
        shape_type="plane"
        created = []
        if len(shape_points) > 0:
            # color will be assigned by caller based on plane_label
            point_cloud_guid = self.add_pointcloud_to_rhino(shape_points, "plane", cluster_id, i)
            if point_cloud_guid:
                created.append(point_cloud_guid)
                #print("  Added " + str(shape_type) + " point cloud to Rhino (" + str(len(shape_points)) + " points)")

        """Create transparent plane surface and yellow boundary polygon from critical points"""
        if not critical_points or len(critical_points) < 3:
            print("Error: Not enough critical points for plane")
            return 
            
        # Handle both old array format and new named dictionary format for coefficients
        if isinstance(coefficients, dict):
            if not all(key in coefficients for key in ["nx", "ny", "nz", "d"]):
                print("Error: Missing required plane coefficients (nx, ny, nz, d)")
                return 
            nx, ny, nz, d = coefficients["nx"], coefficients["ny"], coefficients["nz"], coefficients["d"]
        elif isinstance(coefficients, (list, tuple)) and len(coefficients) >= 4:
            nx, ny, nz, d = coefficients[:4]
        else:
            print("Error: Invalid plane coefficients format")
            return 
            
        # Create points from critical points
        rhino_points = []
        for pt in critical_points:
            rhino_points.append(rg.Point3d(pt[0], pt[1], pt[2]))
        
        # Create boundary polyline (convex hull boundary)
        if len(rhino_points) >= 3:
            # Close the polygon by adding the first point at the end
            closed_points = rhino_points + [rhino_points[0]]
            boundary_polyline = rg.Polyline(closed_points)
            boundary_curve = boundary_polyline.ToNurbsCurve()
            
            # Create plane surface by extruding the boundary slightly
            normal = rg.Vector3d(nx, ny, nz)
            normal.Unitize()
            
            # Create planar surface from the boundary curve
            breps = rg.Brep.CreatePlanarBreps([boundary_curve], 0.001)
            if breps and len(breps) > 0:
                plane_surface = breps[0]
                if plane_surface:
                    # Add transparent white plane surface
                    surface_guid = scriptcontext.doc.Objects.AddBrep(plane_surface)
                    if surface_guid:
                        created.append(surface_guid)
                        rs.ObjectColor(surface_guid, Color.White)
                        rs.ObjectMaterialIndex(surface_guid, -1)  # Use default material
                        # Make transparent (this might need to be done differently in Rhino)
                        cluster_str = str(cluster_id)
                        count_str = str(i)
                        rs.ObjectName(surface_guid, "Plane_Cluster" + cluster_str + "_Shape" + count_str + "_Surface")
                        #print("  Added plane surface")
            
                if boundary_curve:
                    # Add boundary curve colored according to plane label (caller will recolor the point cloud)
                    boundary_guid = scriptcontext.doc.Objects.AddCurve(boundary_curve)
                    if boundary_guid:
                        created.append(boundary_guid)
                        # default yellow if caller doesn't change it
                        rs.ObjectColor(boundary_guid, Color.Yellow)
                        rs.ObjectPrintWidth(boundary_guid, 0.01)  # 1px width
                        cluster_str = str(cluster_id)
                        count_str = str(i)
                        rs.ObjectName(boundary_guid, "Plane_Cluster" + cluster_str + "_Shape" + count_str + "_Boundary")
                        #print("  Added plane boundary")

        return created
    
    def visualize_cylinder(self, coefficients, critical_points, shape_points, cluster_id, i):
        if len(shape_points) < min_points_for_shape:
            print("  Skipping cylinder visualization due to insufficient points (" + str(len(shape_points)) + " < " + str(min_points_for_shape) + ")")
            return
        shape_type="cylider"
        created = []
        if len(shape_points) > 0:
            # color will be assigned by caller based on cylinder_label
            point_cloud_guid = self.add_pointcloud_to_rhino(shape_points, "cylinder", cluster_id, i)
            if point_cloud_guid:
                created.append(point_cloud_guid)
                #print("  Added " + str(shape_type) + " point cloud to Rhino (" + str(len(shape_points)) + " points)")

        """Create transparent cylinder surface and red axis line from critical points"""
        if not critical_points or len(critical_points) != 2:
            print("Error: Need exactly 2 critical points for cylinder")
            return None, None
            
        # Handle both old array format and new named dictionary format for coefficients
        if isinstance(coefficients, dict):
            required_keys = ["px", "py", "pz", "dx", "dy", "dz", "radius"]
            if not all(key in coefficients for key in required_keys):
                print("Error: Missing required cylinder coefficients")
                return None, None
            radius = coefficients["radius"]
        elif isinstance(coefficients, (list, tuple)) and len(coefficients) >= 7:
            radius = coefficients[6]
        else:
            print("Error: Invalid cylinder coefficients format")
            return None, None
            
        # Critical points are P1 and P2 (cylinder ends)
        p1 = rg.Point3d(critical_points[0][0], critical_points[0][1], critical_points[0][2])
        p2 = rg.Point3d(critical_points[1][0], critical_points[1][1], critical_points[1][2])
        
        # Create axis line
        axis_line = rg.Line(p1, p2)
        axis_curve = axis_line.ToNurbsCurve()
        
        # Create cylinder surface
        axis_vector = p2 - p1
        axis_vector.Unitize()
        
        # Create plane at P1 perpendicular to axis
        base_plane = rg.Plane(p1, axis_vector)
        circle = rg.Circle(base_plane, radius)
        
        # Create cylinder by extruding circle
        height = p1.DistanceTo(p2)
        cylinder = rg.Cylinder(circle, height)
        cylinder_surface = cylinder.ToBrep(True, True)
           
        if cylinder_surface:
            # Add transparent white cylinder surface
            surface_guid = scriptcontext.doc.Objects.AddBrep(cylinder_surface)
            if surface_guid:
                created.append(surface_guid)
                rs.ObjectColor(surface_guid, Color.White)
                cluster_str = str(cluster_id)
                count_str = str(i)
                rs.ObjectName(surface_guid, "Cylinder_Cluster" + cluster_str + "_Shape" + count_str + "_Surface")
                #print("  Added cylinder surface")
        
        if axis_curve:
            # Add thick red axis line
            axis_guid = scriptcontext.doc.Objects.AddCurve(axis_curve)
            if axis_guid:
                created.append(axis_guid)
                rs.ObjectColor(axis_guid, Color.Red)
                rs.ObjectPrintWidth(axis_guid, 2.0)  # Thick line
                cluster_str = str(cluster_id)
                count_str = str(i)
                rs.ObjectName(axis_guid, "Cylinder_Cluster" + cluster_str + "_Shape" + count_str + "_Axis")
                #print("  Added cylinder axis")

        return created
    
    def visualize_cone(self, coefficients, critical_points, shape_points, cluster_id, i):
        shape_type="cone"
        if len(shape_points) > 0:
            point_cloud_guid = self.add_pointcloud_to_rhino(shape_points, "cone", cluster_id, i)
            #if point_cloud_guid:
                # created_objects.append(guid)
                #print("  Added " + str(shape_type) + " point cloud to Rhino (" + str(len(shape_points)) + " points)")

        """Create transparent cone surface and red axis line from critical points"""
        if not critical_points or len(critical_points) != 2:
            print("Error: Need exactly 2 critical points for cone")
            return None, None
            
        # Handle both old array format and new named dictionary format for coefficients  
        if isinstance(coefficients, dict):
            required_keys = ["apex_x", "apex_y", "apex_z", "dx", "dy", "dz", "angle"]
            if not all(key in coefficients for key in required_keys):
                print("Error: Missing required cone coefficients")
                return None, None
            apex_x, apex_y, apex_z = coefficients["apex_x"], coefficients["apex_y"], coefficients["apex_z"]
            dx, dy, dz = coefficients["dx"], coefficients["dy"], coefficients["dz"]
            angle = coefficients["angle"]
        elif isinstance(coefficients, (list, tuple)) and len(coefficients) >= 7:
            apex_x, apex_y, apex_z, dx, dy, dz, angle = coefficients[:7]
        else:
            print("Error: Invalid cone coefficients format")
            return None, None
            
        # Critical points are P1 and P2 (cone ends along axis)
        p1 = rg.Point3d(critical_points[0][0], critical_points[0][1], critical_points[0][2])
        p2 = rg.Point3d(critical_points[1][0], critical_points[1][1], critical_points[1][2])
        
        # Create axis line
        axis_line = rg.Line(p1, p2)
        axis_curve = axis_line.ToNurbsCurve()
        
        # Create cone surface
        apex = rg.Point3d(apex_x, apex_y, apex_z)
        axis_vector = rg.Vector3d(dx, dy, dz)
        axis_vector.Unitize()
        
        # Find which critical point is farther from apex to use as base
        dist1 = apex.DistanceTo(p1)
        dist2 = apex.DistanceTo(p2)
        
        if dist1 > dist2:
            base_center = p1
            height = dist1
        else:
            base_center = p2  
            height = dist2
            
        # Calculate base radius using cone angle and height
        base_radius = height * abs(rg.RhinoMath.Tan(angle))
        
        # Create cone
        base_plane = rg.Plane(base_center, axis_vector)
        cone = rg.Cone(base_plane, height, base_radius)
        cone_surface = cone.ToBrep(True)
        
        if cone_surface:
            # Add transparent white cone surface
            surface_guid = scriptcontext.doc.Objects.AddBrep(cone_surface)
            if surface_guid:
                rs.ObjectColor(surface_guid, Color.White)
                rs.ObjectName(surface_guid, "Cone_Cluster" + str(cluster_id) + "_Shape" + str(i+1) + "_Surface")
                # created_objects.append(surface_guid)
                print("  Added cone surface")
        
        if axis_curve:
            # Add thick red axis line
            axis_guid = scriptcontext.doc.Objects.AddCurve(axis_curve)
            if axis_guid:
                rs.ObjectColor(axis_guid, Color.Red)
                rs.ObjectPrintWidth(axis_guid, 2.0)  # Thick line
                rs.ObjectName(axis_guid, "Cone_Cluster" + str(cluster_id) + "_Shape" + str(i+1) + "_Axis")
                # created_objects.append(axis_guid)
                print("  Added cone axis")

    def visualize_torus(self, coefficients, critical_points, shape_points, cluster_id, i):
        shape_type="torus"
        if len(shape_points) > 0:
            point_cloud_guid = self.add_pointcloud_to_rhino(shape_points, "torus", cluster_id, i)
            #if point_cloud_guid:
                # created_objects.append(guid)
                #print("  Added " + str(shape_type) + " point cloud to Rhino (" + str(len(shape_points)) + " points)")

        try:
            # Get torus parameters using create_torus_geometry logic
            if isinstance(coefficients, dict):
                center_x, center_y, center_z = coefficients["center_x"], coefficients["center_y"], coefficients["center_z"]
                nx, ny, nz = coefficients["nx"], coefficients["ny"], coefficients["nz"]
                major_radius = coefficients["major_radius"]
            elif isinstance(coefficients, (list, tuple)) and len(coefficients) >= 8:
                center_x, center_y, center_z, nx, ny, nz, major_radius, _ = coefficients[:8]
            else:
                raise Exception("Invalid torus coefficients format")
            center = rg.Point3d(center_x, center_y, center_z)
            normal = rg.Vector3d(nx, ny, nz)
            normal.Unitize()
            torus_plane = rg.Plane(center, normal)
            # Loop through all torus fitted points
            # Assign the three critical points to pt1, pt2, pt3
            if len(critical_points) == 3:
                pt1 = rg.Point3d(*critical_points[0])
                pt2 = rg.Point3d(*critical_points[1])
                pt3 = rg.Point3d(*critical_points[2])

            arc = rg.Arc(pt1, pt2, pt3)
            # Draw the arc spine as a red arc geometric object
            arc_guid = scriptcontext.doc.Objects.AddArc(arc)
            if arc_guid:
                rs.ObjectColor(arc_guid, Color.Red)
                rs.ObjectPrintWidth(arc_guid, 2.0)
                rs.ObjectName(arc_guid, "Torus_Cluster" + str(cluster_id) + "_Shape" + str(i+1) + "_ArcSpine")
                # created_objects.append(arc_guid)
                #print("  Added torus arc spine as geometric arc object")

                minor_radius = coefficients["minor_radius"]
                # Create a circle at the start of the arc
                arc_start = pt1
                arc_curve = arc.ToNurbsCurve()
                # arc_tangent = arc_curve.TangentAtStart
                # arc_plane = rg.Plane(arc_start, arc_tangent)
                # profile_circle = rg.Circle(arc_plane, minor_radius)

                # Sweep the circle along the arc
                pipe_breps = rg.Brep.CreatePipe(arc_curve, minor_radius, False, rg.PipeCapMode.Flat, True, 0.01, 0.01)
                if pipe_breps and len(pipe_breps) > 0:
                    pipe_guid = scriptcontext.doc.Objects.AddBrep(pipe_breps[0])
                    if pipe_guid:
                        rs.ObjectColor(pipe_guid, Color.White)
                        rs.ObjectName(pipe_guid, "Torus_Cluster" + str(cluster_id) + "_Shape" + str(i+1) + "_PipeSweep")
                        # created_objects.append(pipe_guid)
                        #print("  Added torus pipe sweep on arc with radius", minor_radius)
        except Exception as e:
            print("  Error creating torus: " + str(e))

    def add_pointcloud_to_rhino(self, points, shape_type, cluster_id, shape_id, color=None):
        """Add point cloud to Rhino document with attributes"""
        if not points or len(points) == 0:
            return None
            
        # Set colors by shape type
        if color is None:
            if shape_type == "plane":
                color = Color.Yellow
            elif shape_type == "cylinder":
                # Random shade of blue for pipes/cylinders
                blue_intensity = random.randint(100, 255)
                color = Color.FromArgb(0, 0, blue_intensity)
            elif shape_type == "cone":
                color = Color.Green
            elif shape_type == "torus":
                color = Color.Purple
            else:
                color = Color.Gray
        
        # Efficient bulk insertion using RhinoCommon PointCloud
        pc = rg.PointCloud()

        # If points is a NumPy array, handle bulk copy without building Python lists
        try:
            if np is not None and hasattr(points, 'ndim') and points.ndim == 2 and points.shape[1] >= 3:
                n = points.shape[0]
                # Try using PointCloud.Add for each point but avoid Python list allocations
                for i in range(n):
                    x = float(points[i, 0]); y = float(points[i, 1]); z = float(points[i, 2])
                    pt = rg.Point3d(x, y, z)
                    pc.Add(pt)

                # If normals are present (Nx6), set per-point normals
                try:
                    if points.shape[1] >= 6:
                        # interpret as x,y,z,nx,ny,nz
                        for i in range(n):
                            nx = float(points[i, 3]); ny = float(points[i, 4]); nz = float(points[i, 5])
                            # Rhino PointCloud stores normals per point as Vector3d when supported
                            try:
                                pc.SetPointNormal(i, rg.Vector3d(nx, ny, nz))
                            except Exception:
                                # older RhinoCommon may not expose SetPointNormal; ignore
                                pass
                except Exception:
                    pass
            else:
                # Generic iterable fallback
                count = 0
                for pt in points:
                    try:
                        if isinstance(pt, (list, tuple)) and len(pt) >= 3:
                            p = rg.Point3d(float(pt[0]), float(pt[1]), float(pt[2]))
                            pc.Add(p)
                            count += 1
                        else:
                            # skip invalid entries
                            continue
                    except Exception:
                        continue
                if count == 0:
                    print("  Warning: No valid points for " + str(shape_type) + " shape")
                    return None
        except Exception as e:
            print("  Warning: Failed to add points to Rhino PointCloud: " + str(e))
            return None
        


        # Add the constructed PointCloud to the Rhino document in one call
        try:
            guid = scriptcontext.doc.Objects.AddPointCloud(pc)
        except Exception as e:
            print("  Error adding PointCloud to Rhino document: " + str(e))
            return None

        if guid:
            # Set object attributes
            try:
                rs.ObjectColor(guid, color)
            except Exception:
                pass
            try:
                rs.ObjectName(guid, shape_type.title() + "_Cluster" + str(cluster_id) + "_Shape" + str(shape_id) + "_Points")
            except Exception:
                pass

        return guid
    
    def visualize_results(self, results):
        """Visualize processing results in Rhino as point clouds"""
        if "error" in results:
            print("Error in results: " + str(results.get('error')))
            return []
        #cgal_results = results.get("cgal_results", {})

        # Expect the new processor to write a root GenericShape JSON string under cgal_results.root (or top-level 'root')
        # Try multiple fallbacks for compatibility
        root = results

        created_objects = []

        # If results are in flat format with 'planes'/'cylinders', iterate directly
        if isinstance(results, dict) and ("planes" in results or "cylinders" in results):
            for i, p in enumerate(results.get('planes', [])):
                coeffs = p.get('coefficients')
                cps = p.get('critical_points', [])
                pts = p.get('shape_points', [])
                cid = p.get('cluster_id', 0)
                guids = self.visualize_plane(coeffs, cps, pts, cid, i)
                if guids:
                    created_objects.extend(guids)

            for i, c in enumerate(results.get('cylinders', [])):
                coeffs = c.get('coefficients')
                cps = c.get('critical_points', [])
                pts = c.get('shape_points', [])
                cid = c.get('cluster_id', 0)
                guids = self.visualize_cylinder(coeffs, cps, pts, cid, i)
                if guids:
                    created_objects.extend(guids)

            return created_objects



        # Color mapping helpers consistent with main.cpp
        def plane_color_for_label(label):
            # label: 0 -> yellow (r=255,g=255,b=0), 1 -> cyan (0,255,255), 2 -> magenta (255,0,255)
            if label == 0:
                return Color.FromArgb(255, 255, 0)
            elif label == 1:
                return Color.FromArgb(0, 255, 255)
            elif label == 2:
                return Color.FromArgb(255, 0, 255)
            else:
                return Color.Yellow

        def cylinder_color_for_label(label):
            # label: 0 -> red, 1 -> green, 2 -> blue
            if label == 0:
                return Color.FromArgb(255, 0, 0)
            elif label == 1:
                return Color.FromArgb(0, 255, 0)
            elif label == 2:
                return Color.FromArgb(0, 0, 255)
            else:
                return Color.FromArgb(200, 200, 200)

        # Recursive traversal
        def traverse_node(node, cluster_id=0, depth=0, idx_prefix="", created_objects=None):
            if node is None:
                return
            ntype = node.get("type", "generic")
            pts = node.get("points", [])
            coeffs = node.get("coefficients", [])
            cps = node.get("critical_points", [])
            plane_label = node.get("plane_label", -1)
            cyl_label = node.get("cylinder_label", -1)

            id_str = idx_prefix if idx_prefix else str(depth)

            if ntype == "plane":
                color = plane_color_for_label(plane_label)
                # visualize plane (this will add point cloud, surface, and boundary)
                guids = self.visualize_plane(coeffs, cps, pts, cluster_id, id_str)
                if guids and created_objects is not None:
                    created_objects.extend(guids)
                # recolor boundary appropriately
                # recolor logic happens inside visualize_plane's boundary creation
            elif ntype == "cylinder":
                color = cylinder_color_for_label(cyl_label)
                guids = self.visualize_cylinder(coeffs, cps, pts, cluster_id, id_str)
                if guids and created_objects is not None:
                    created_objects.extend(guids)
            else:
                # generic: add points in gray
                if pts:
                    guid = self.add_pointcloud_to_rhino(pts, "generic", cluster_id, id_str, color=Color.FromArgb(128,128,128))
                    if guid and created_objects is not None:
                        created_objects.append(guid)

            # recurse into children
            children = node.get("children", [])
            for i, c in enumerate(children):
                traverse_node(c, cluster_id, depth+1, id_str + "." + str(i), created_objects)
        traverse_node(root, 0, 0, "0", created_objects)

        return created_objects


def run_pcl_processing(solver_type):
    """Main function for Rhino point cloud processing"""
    print("=== Rhino PCL Point Cloud Processor ===")
    print("This tool processes point clouds using PCL+CGAL via WSL")
    print()
    
    # Ask user if they want to run test mode
    # test_mode = rs.GetString("Run in test mode? (y/N)", "N")
    # if test_mode and test_mode.lower() in ['y', 'yes']:
    #     test_pointcloud_visualization()
    #     return
    
    # Create processor
    processor = RhinoPointCloudProcessor()
    
    # Step 1: Select point cloud
    print("Step 1: Select Point Cloud")
    selection = processor.select_point_cloud()
    if not selection:
        print("No point cloud selected. Exiting.")
        return
    
    pointcloud, object_id = selection
    
    # Step 2: Get processing parameters
    print("\nStep 2: Processing Parameters")
    

    
    # Step 3: Convert and process
    print("\nStep 3: Processing " + str(pointcloud.Count) + " points...")
    
    # Convert to point list
    points = processor.pointcloud_to_list(pointcloud)
    print("Converted " + str(len(points)) + " points to processing format")
    
    # Process via WSL
    print("Sending for PCL processing...")
    results = processor.process_pointcloud(
        points, 
        solver_type=solver_type
    )
    
    # Step 4: Visualize results
    print("\nStep 4: Visualizing Results")
    if "error" not in results:
        created_objects = processor.visualize_results(results)
    # print("\nProcessing complete! Created " + str(len(created_objects)) + " geometry objects.")
        print("Check the Rhino viewport for detected shapes.")
        return True
    # , "Successfully processed " + str(len(points)) + " points and created " + str(len(created_objects)) + " objects"

    else:
        error_msg = "Processing failed: " + str(results.get('error'))
        print(error_msg)
        return False, error_msg
    


# Run the main function
if __name__ == "__main__":
    run_pcl_processing("BEST")
