#!/usr/bin/env python3
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
import random
from System.Drawing import Color
import scriptcontext

min_points_for_shape = 100  # Minimum points required to visualize a shape

class RhinoPointCloudProcessor:
    def visualize_pipesequence(self, pipeseq, cluster_id, i):
        """Visualize a piping sequence by reusing cylinder and torus visualization for nested shapes"""
        print(f"  Visualizing PipeSequence (cluster {cluster_id}, shape {i+1})")
        # PipeSequence is expected to have a 'shapes' array, each with type 'cylinder' or 'torus'
        shapes = pipeseq.get("shapes", [])
        for j, shape in enumerate(shapes):
            shape_type = shape.get("type", "unknown")
            shape_points = shape.get("points", [])
            shape_coefficients = shape.get("coefficients", [])
            critical_points = shape.get("critical_points", [])
            print(f"    PipeSequence sub-shape {j+1}: {shape_type}")
            if shape_type == "cylinder":
                self.visualize_cylinder(shape_coefficients, critical_points, shape_points, cluster_id, f"{i+1}_{j+1}")
            elif shape_type == "torus":
                self.visualize_torus(shape_coefficients, critical_points, shape_points, cluster_id, f"{i+1}_{j+1}")
            else:
                print(f"    Unknown sub-shape type in PipeSequence: {shape_type}")
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
            
        print(f"Selected point cloud with {pointcloud.Count} points")
        return pointcloud, objref.ObjectId
    
    def pointcloud_to_list(self, pointcloud):
        """Convert Rhino PointCloud to list of [x,y,z] coordinates"""
        points = []
        for i in range(pointcloud.Count):
            pt = pointcloud[i].Location
            points.append([float(pt.X), float(pt.Y), float(pt.Z)])
        return points
    
    def process_pointcloud_wsl(self, points, solver_type="BEST"):
        """Process point cloud using WSL headless processor"""
        try:
            print(f"Processing {len(points)} points...")
            print(f"Solver type: {solver_type}")
            
            # Write points in XYZ format (space-separated x y z coordinates)
            print(f"Writing XYZ data to {self.input_file}")
            with open(self.input_file, 'w') as f:
                for point in points:
                    f.write(f"{point[0]} {point[1]} {point[2]}\n")
            
            print(f"XYZ file written: {os.path.getsize(self.input_file)} bytes, {len(points)} points")
            
            # Call WSL script with parameters (script will call the headless processor and write output to data/ws_output.json)
            print("Calling WSL processing script...")
            # Use bash -c to cd into the WSL project and invoke the helper script
            cmd = ['wsl', 'bash', '-c', f'cd /home/sheldd/pcl/pcl_win && {self.wsl_script_path} "{self.input_file}"']
            print(f"Command: {' '.join(cmd)}")
            
            start_time = time.time()
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=300)
            processing_time = time.time() - start_time
            
            print(f"WSL processing completed in {processing_time:.2f} seconds")
            print(f"Return code: {result.returncode}")
            
            if result.stdout:
                print(f"WSL stdout: {result.stdout}")
            if result.stderr:
                print(f"WSL stderr: {result.stderr}")
            
            if result.returncode != 0:
                error_msg = f"WSL script failed with code {result.returncode}: {result.stderr}"
                print(error_msg)
                return {"error": error_msg}
            
            # Read results from the WSL-produced JSON via the Windows UNC path
            if not os.path.exists(self.output_file):
                error_msg = f"Output file not found: {self.output_file}"
                print(error_msg)
                return {"error": error_msg}

            print(f"Reading results from {self.output_file}")
            print(f"Output file size: {os.path.getsize(self.output_file)} bytes")

            with open(self.output_file, 'r') as f:
                results = json.load(f)
            
            print(f"Successfully loaded JSON results")
            return results
            
        except subprocess.TimeoutExpired:
            error_msg = "WSL processing timed out (>300 seconds)"
            print(error_msg)
            return {"error": error_msg}
        except json.JSONDecodeError as e:
            error_msg = f"Failed to parse JSON results: {str(e)}"
            print(error_msg)
            return {"error": error_msg}
        except Exception as e:
            error_msg = f"Processing error: {str(e)}"
            print(error_msg)
            return {"error": error_msg}
    
    def visualize_plane(self, coefficients, critical_points, shape_points, cluster_id, i):
        if len(shape_points) < min_points_for_shape:
            print(f"  Skipping plane visualization due to insufficient points ({len(shape_points)} < {min_points_for_shape})")
            return
        shape_type="plane"
        if len(shape_points) > 0:
            # color will be assigned by caller based on plane_label
            point_cloud_guid = self.add_pointcloud_to_rhino(shape_points, "plane", cluster_id, i)
            if point_cloud_guid:
                # created_objects.append(guid)
                print(f"  Added {shape_type} point cloud to Rhino ({len(shape_points)} points)")

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
                    rs.ObjectColor(surface_guid, Color.White)
                    rs.ObjectMaterialIndex(surface_guid, -1)  # Use default material
                    # Make transparent (this might need to be done differently in Rhino)
                    cluster_str = str(cluster_id)
                    count_str = str(i)
                    rs.ObjectName(surface_guid, f"Plane_Cluster{cluster_str}_Shape{count_str}_Surface")
                    # created_objects.append(surface_guid)
                    print(f"  Added plane surface")
            
            if boundary_curve:
                # Add boundary curve colored according to plane label (caller will recolor the point cloud)
                boundary_guid = scriptcontext.doc.Objects.AddCurve(boundary_curve)
                if boundary_guid:
                    # default yellow if caller doesn't change it
                    rs.ObjectColor(boundary_guid, Color.Yellow)
                    rs.ObjectPrintWidth(boundary_guid, 0.01)  # 1px width
                    cluster_str = str(cluster_id)
                    count_str = str(i)
                    rs.ObjectName(boundary_guid, f"Plane_Cluster{cluster_str}_Shape{count_str}_Boundary")
                    # created_objects.append(boundary_guid)
                    print(f"  Added plane boundary")
    
    def visualize_cylinder(self, coefficients, critical_points, shape_points, cluster_id, i):
        if len(shape_points) < min_points_for_shape:
            print(f"  Skipping cylinder visualization due to insufficient points ({len(shape_points)} < {min_points_for_shape})")
            return
        shape_type="cylider"
        if len(shape_points) > 0:
            # color will be assigned by caller based on cylinder_label
            point_cloud_guid = self.add_pointcloud_to_rhino(shape_points, "cylinder", cluster_id, i)
            if point_cloud_guid:
                # created_objects.append(guid)
                print(f"  Added {shape_type} point cloud to Rhino ({len(shape_points)} points)")

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
                rs.ObjectColor(surface_guid, Color.White)
                cluster_str = str(cluster_id)
                count_str = str(i)
                rs.ObjectName(surface_guid, f"Cylinder_Cluster{cluster_str}_Shape{count_str}_Surface")
                # created_objects.append(surface_guid)
                print(f"  Added cylinder surface")
        
        if axis_curve:
            # Add thick red axis line
            axis_guid = scriptcontext.doc.Objects.AddCurve(axis_curve)
            if axis_guid:
                rs.ObjectColor(axis_guid, Color.Red)
                rs.ObjectPrintWidth(axis_guid, 2.0)  # Thick line
                cluster_str = str(cluster_id)
                count_str = str(i)
                rs.ObjectName(axis_guid, f"Cylinder_Cluster{cluster_str}_Shape{count_str}_Axis")
                # created_objects.append(axis_guid)
                print(f"  Added cylinder axis")
    
    def visualize_cone(self, coefficients, critical_points, shape_points, cluster_id, i):
        shape_type="cone"
        if len(shape_points) > 0:
            point_cloud_guid = self.add_pointcloud_to_rhino(shape_points, "cone", cluster_id, i)
            if point_cloud_guid:
                # created_objects.append(guid)
                print(f"  Added {shape_type} point cloud to Rhino ({len(shape_points)} points)")

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
                rs.ObjectName(surface_guid, f"Cone_Cluster{cluster_id}_Shape{i+1}_Surface")
                # created_objects.append(surface_guid)
                print(f"  Added cone surface")
        
        if axis_curve:
            # Add thick red axis line
            axis_guid = scriptcontext.doc.Objects.AddCurve(axis_curve)
            if axis_guid:
                rs.ObjectColor(axis_guid, Color.Red)
                rs.ObjectPrintWidth(axis_guid, 2.0)  # Thick line
                rs.ObjectName(axis_guid, f"Cone_Cluster{cluster_id}_Shape{i+1}_Axis")
                # created_objects.append(axis_guid)
                print(f"  Added cone axis")

    def visualize_torus(self, coefficients, critical_points, shape_points, cluster_id, i):
        shape_type="torus"
        if len(shape_points) > 0:
            point_cloud_guid = self.add_pointcloud_to_rhino(shape_points, "torus", cluster_id, i)
            if point_cloud_guid:
                # created_objects.append(guid)
                print(f"  Added {shape_type} point cloud to Rhino ({len(shape_points)} points)")

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
                rs.ObjectName(arc_guid, f"Torus_Cluster{cluster_id}_Shape{i+1}_ArcSpine")
                # created_objects.append(arc_guid)
                print("  Added torus arc spine as geometric arc object")

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
                        rs.ObjectName(pipe_guid, f"Torus_Cluster{cluster_id}_Shape{i+1}_PipeSweep")
                        # created_objects.append(pipe_guid)
                        print("  Added torus pipe sweep on arc with radius", minor_radius)
        except Exception as e:
            print(f"  Error creating torus: {e}")

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
        
        # Convert points to Rhino Point3d objects
        rhino_points = []
        for pt in points:
            try:
                if isinstance(pt, list) and len(pt) >= 3:
                    # Standard [x, y, z] format
                    rhino_points.append(rg.Point3d(float(pt[0]), float(pt[1]), float(pt[2])))
                elif isinstance(pt, list) and len(pt) == 1:
                    # Single coordinate - likely an error in data format, skip these
                    continue
                elif isinstance(pt, (int, float)):
                    # Single number - likely an error in data format, skip these
                    continue
                else:
                    print(f"  Warning: Unrecognized point format: {pt}")
            except (ValueError, IndexError) as e:
                print(f"  Warning: Could not convert point {pt}: {e}")
                continue
        
        if len(rhino_points) == 0:
            print(f"  Warning: No valid points for {shape_type} shape")
            return None
        


        # 2. Create a new Rhino.Geometry.PointCloud object
        point_cloud = rg.PointCloud()

        # 3. Add the points to the point cloud
        for point in rhino_points:
            point_cloud.Add(point) # You can also add colors or normals here if needed

        # 4. Add the point cloud to the Rhino document
        # scriptcontext.doc.Objects.AddPointCloud() is a RhinoCommon method that adds the point cloud to the active Rhino document
        guid =scriptcontext.doc.Objects.AddPointCloud(point_cloud) 
        if guid:
            # Set object attributes
            rs.ObjectColor(guid, color)
            rs.ObjectName(guid, f"{shape_type.title()}_Cluster{cluster_id}_Shape{shape_id}_Points")
        
        return guid
    
    def visualize_results(self, results):
        """Visualize processing results in Rhino as point clouds"""
        if "error" in results:
            print(f"Error in results: {results['error']}")
            return []
        #cgal_results = results.get("cgal_results", {})

        # Expect the new processor to write a root GenericShape JSON string under cgal_results.root (or top-level 'root')
        # Try multiple fallbacks for compatibility
        root = results



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
        def traverse_node(node, cluster_id=0, depth=0, idx_prefix=""):
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
                # add points with color
                self.add_pointcloud_to_rhino(pts, "plane", cluster_id, id_str, color=color)
                # draw hull and boundary
                self.visualize_plane(coeffs, cps, pts, cluster_id, id_str)
                # recolor boundary appropriately
                # recolor logic happens inside visualize_plane's boundary creation
            elif ntype == "cylinder":
                color = cylinder_color_for_label(cyl_label)
                self.add_pointcloud_to_rhino(pts, "cylinder", cluster_id, id_str, color=color)
                self.visualize_cylinder(coeffs, cps, pts, cluster_id, id_str)
            else:
                # generic: add points in gray
                if pts:
                    self.add_pointcloud_to_rhino(pts, "generic", cluster_id, id_str, color=Color.FromArgb(128,128,128))

            # recurse into children
            children = node.get("children", [])
            for i, c in enumerate(children):
                traverse_node(c, cluster_id, depth+1, f"{id_str}.{i}")

        traverse_node(root, 0, 0, "0")

        return []


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
    print(f"\nStep 3: Processing {pointcloud.Count} points...")
    
    # Convert to point list
    points = processor.pointcloud_to_list(pointcloud)
    print(f"Converted {len(points)} points to processing format")
    
    # Process via WSL
    print("Sending to WSL for PCL+CGAL processing...")
    results = processor.process_pointcloud_wsl(
        points, 
        solver_type=solver_type
    )
    
    # Step 4: Visualize results
    print("\nStep 4: Visualizing Results")
    if "error" not in results:
        created_objects = processor.visualize_results(results)
        # print(f"\nProcessing complete! Created {len(created_objects)} geometry objects.")
        print("Check the Rhino viewport for detected shapes.")
        return True
        # , f"Successfully processed {len(points)} points and created {len(created_objects)} objects"

    else:
        error_msg = f"Processing failed: {results['error']}"
        print(error_msg)
        return False, error_msg
    


# Run the main function
if __name__ == "__main__":
    run_pcl_processing("BEST")
