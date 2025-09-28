"""
PCL Processor Settings Panel - Rhino Python Script with Eto Forms
This script creates a tabbed interface for managing headless_processor parameters
"""

import rhinoscriptsyntax as rs
import Rhino
import System
import System.Drawing as Drawing
import Eto.Drawing as EtoDrawing
import Eto.Forms as EtoForms
import json
import os
import rhino_client 


class PCLProcessorSettingsForm(EtoForms.Dialog[bool]):
    """Main settings form with tabbed interface"""
    
    def __init__(self):
        # Initialize the base dialog first
        EtoForms.Dialog[bool].__init__(self)
        
        self.Title = "PCL Processor Settings"
        self.Resizable = True
        self.Minimizable = False
        self.Maximizable = False
        self.ShowInTaskbar = False
        # self.Size = EtoDrawing.Size(600, 700)
        
        # Initialize parameter dictionaries with default values
        self.general_params = {
            "cluster_tolerance": 3.0,
            "min_cluster_size": 50,
            "max_cluster_size": 1000000,
            "max_shapes_per_cluster": 10,
            "min_points_threshold": 50,
            # New: angle tolerance used by ShapeFinder and some shape heuristics
            "angle_tolerance_deg": 5.0
        }
        
        self.plane_params = {
            "min_points": 10,
            "epsilon": 0.5,
            "cluster_epsilon": 1.0,
            "normal_threshold": 0.9,
            "probability": 0.05
        }
        
        self.cylinder_params = {
            "min_points": 8,
            "epsilon": 2.0,
            "cluster_epsilon": 2.5,
            "normal_threshold": 0.6,
            "probability": 0.01
        }
        
        self.cone_params = {
            "min_points": 8,
            "epsilon": 2.0,
            "cluster_epsilon": 2.5,
            "normal_threshold": 0.6,
            "probability": 0.01
        }
        
        self.torus_params = {
            "min_points": 8,
            "epsilon": 3.0,
            "cluster_epsilon": 3.5,
            "normal_threshold": 0.5,
            "probability": 0.01
        }
        
        # PCL-specific parameters
        self.pcl_params = {
            "distance_threshold_plane": 1.0,
            "distance_threshold_cylinder": 1.0,
            "normal_distance_weight": 0.1,
            "cylinder_radius_min": 1.0,
            "cylinder_radius_max": 10.0,
            "max_iterations_plane": 1000,
            "max_iterations_cylinder": 50000,
            "normal_weight_cylinder": 0.2
        }
        
        self.settings_file = r"C:\Temp\pcl_processing\settings.json"
        
        # Create the UI
        self.InitializeComponent()
        
        # Try to load existing settings
        self.LoadSettings()
    
    def InitializeComponent(self):
        """Create the UI components"""
        
        # Create tab control
        tab_control = EtoForms.TabControl()
        
        # General Parameters Tab
        general_tab = EtoForms.TabPage()
        general_tab.Text = "General"
        general_tab.Content = self.CreateGeneralTab()
        tab_control.Pages.Add(general_tab)
        

        
        # PCL Parameters Tab
        pcl_tab = EtoForms.TabPage()
        pcl_tab.Text = "PCL Shapes"
        pcl_tab.Content = self.CreatePCLTab()
        tab_control.Pages.Add(pcl_tab)
        
        # Create button panel
        button_panel = self.CreateButtonPanel()
        
        # Main layout
        layout = EtoForms.DynamicLayout()
        layout.Spacing = EtoDrawing.Size(5, 5)
        layout.Padding = EtoDrawing.Padding(10)
        
        layout.AddRow(tab_control)
        layout.AddRow(button_panel)
        
        self.Content = layout
    
    def CreateGeneralTab(self):
        """Create the General parameters tab"""
        layout = EtoForms.DynamicLayout()
        layout.Spacing = EtoDrawing.Size(5, 5)
        layout.Padding = EtoDrawing.Padding(10)
        
        # Clustering Parameters Section
        clustering_group = EtoForms.GroupBox()
        clustering_group.Text = "Clustering Parameters"
        
        clustering_layout = EtoForms.DynamicLayout()
        clustering_layout.Spacing = EtoDrawing.Size(5, 5)
        clustering_layout.Padding = EtoDrawing.Padding(10)
        
        # Cluster tolerance
        self.cluster_tolerance_box = EtoForms.NumericStepper()
        self.cluster_tolerance_box.DecimalPlaces = 2
        self.cluster_tolerance_box.MinValue = 0.1
        self.cluster_tolerance_box.MaxValue = 100.0
        self.cluster_tolerance_box.Increment = 0.1
        self.cluster_tolerance_box.Value = self.general_params["cluster_tolerance"]
        cluster_tolerance_label = EtoForms.Label()
        cluster_tolerance_label.Text = "Cluster Tolerance:"
        if hasattr(cluster_tolerance_label, 'ToolTip'):
            cluster_tolerance_label.ToolTip = "Distance tolerance for clustering points. Higher values may merge clusters."
        if hasattr(cluster_tolerance_label, 'Wrap'):
            cluster_tolerance_label.Wrap = EtoForms.WrapMode.Word
        clustering_layout.AddRow(cluster_tolerance_label, self.cluster_tolerance_box)

        # Angle tolerance (degrees) - new control on General tab
        self.angle_tolerance_box = EtoForms.NumericStepper()
        self.angle_tolerance_box.DecimalPlaces = 2
        self.angle_tolerance_box.MinValue = 0.0
        self.angle_tolerance_box.MaxValue = 180.0
        self.angle_tolerance_box.Increment = 0.5
        self.angle_tolerance_box.Value = self.general_params.get("angle_tolerance_deg", 5.0)
        angle_tolerance_label = EtoForms.Label()
        angle_tolerance_label.Text = "Angle Tolerance (deg):"
        if hasattr(angle_tolerance_label, 'ToolTip'):
            angle_tolerance_label.ToolTip = "Maximum allowed angular deviation in degrees for shape grouping/merging."
        clustering_layout.AddRow(angle_tolerance_label, self.angle_tolerance_box)

        # Min cluster size
        self.min_cluster_size_box = EtoForms.NumericStepper()
        self.min_cluster_size_box.DecimalPlaces = 0
        self.min_cluster_size_box.MinValue = 1
        self.min_cluster_size_box.MaxValue = 10000
        self.min_cluster_size_box.Increment = 1
        self.min_cluster_size_box.Value = self.general_params["min_cluster_size"]
        min_cluster_size_label = EtoForms.Label()
        min_cluster_size_label.Text = "Min Cluster Size:"
        clustering_layout.AddRow(min_cluster_size_label, self.min_cluster_size_box)
        
        # Max cluster size
        self.max_cluster_size_box = EtoForms.NumericStepper()
        self.max_cluster_size_box.DecimalPlaces = 0
        self.max_cluster_size_box.MinValue = 100
        self.max_cluster_size_box.MaxValue = 10000000
        self.max_cluster_size_box.Increment = 1000
        self.max_cluster_size_box.Value = self.general_params["max_cluster_size"]
        max_cluster_size_label = EtoForms.Label()
        max_cluster_size_label.Text = "Max Cluster Size:"
        if hasattr(max_cluster_size_label, 'ToolTip'):
            max_cluster_size_label.ToolTip = "Maximum number of points in a cluster. Set to 0 for no limit."
        if hasattr(max_cluster_size_label, 'Wrap'):
            max_cluster_size_label.Wrap = EtoForms.WrapMode.Word
        clustering_layout.AddRow(max_cluster_size_label, self.max_cluster_size_box)
        
        clustering_group.Content = clustering_layout
        
        # Shape Detection Parameters Section
        shape_group = EtoForms.GroupBox()
        shape_group.Text = "Shape Detection Parameters"
        
        shape_layout = EtoForms.DynamicLayout()
        shape_layout.Spacing = EtoDrawing.Size(5, 5)
        shape_layout.Padding = EtoDrawing.Padding(10)
        
        # Max shapes per cluster
        self.max_shapes_box = EtoForms.NumericStepper()
        self.max_shapes_box.DecimalPlaces = 0
        self.max_shapes_box.MinValue = 1
        self.max_shapes_box.MaxValue = 50
        self.max_shapes_box.Increment = 1
        self.max_shapes_box.Value = self.general_params["max_shapes_per_cluster"]
        max_shapes_label = EtoForms.Label()
        max_shapes_label.Text = "Max Shapes per Cluster:"
        shape_layout.AddRow(max_shapes_label, self.max_shapes_box)
        
        # Min points threshold
        self.min_points_threshold_box = EtoForms.NumericStepper()
        self.min_points_threshold_box.DecimalPlaces = 0
        self.min_points_threshold_box.MinValue = 5
        self.min_points_threshold_box.MaxValue = 1000
        self.min_points_threshold_box.Increment = 5
        self.min_points_threshold_box.Value = self.general_params["min_points_threshold"]
        min_points_threshold_label = EtoForms.Label()
        min_points_threshold_label.Text = "Min Points Threshold:"
        shape_layout.AddRow(min_points_threshold_label, self.min_points_threshold_box)
        
        shape_group.Content = shape_layout
        
        # Add info panel
        info_label = EtoForms.Label()
        info_label.Text = "General parameters that affect clustering and overall shape detection behavior."
        if hasattr(info_label, 'Wrap'):
            info_label.Wrap = EtoForms.WrapMode.Word
        
        layout.AddRow(clustering_group)
        layout.AddRow(shape_group)
        layout.AddRow(info_label)
        layout.AddRow(None)  # Spacer
        
        return layout
    

    
    def CreateShapeTab(self, shape_name, params_dict, prefix):
        """Create a generic shape parameters tab"""
        layout = EtoForms.DynamicLayout()
        layout.Spacing = EtoDrawing.Size(5, 5)
        layout.Padding = EtoDrawing.Padding(10)
        
        # CGAL RANSAC Parameters Section
        ransac_group = EtoForms.GroupBox()
        ransac_group.Text = shape_name

        ransac_layout = EtoForms.DynamicLayout()
        ransac_layout.Spacing = EtoDrawing.Size(5, 5)
        ransac_layout.Padding = EtoDrawing.Padding(10)

        # Create controls and store references
        controls = {}

        # Min Points
        controls[prefix + "_min_points"] = EtoForms.NumericStepper()
        controls[prefix + "_min_points"].DecimalPlaces = 0
        controls[prefix + "_min_points"].MinValue = 3
        controls[prefix + "_min_points"].MaxValue = 100
        controls[prefix + "_min_points"].Increment = 1
        controls[prefix + "_min_points"].Value = params_dict["min_points"]
        min_points_label = EtoForms.Label()
        min_points_label.Text = "Min Points:"
        ransac_layout.AddRow(min_points_label, controls[prefix + "_min_points"])

        # Epsilon
        controls[prefix + "_epsilon"] = EtoForms.NumericStepper()
        controls[prefix + "_epsilon"].DecimalPlaces = 2
        controls[prefix + "_epsilon"].MinValue = 0.1
        controls[prefix + "_epsilon"].MaxValue = 10.0
        controls[prefix + "_epsilon"].Increment = 0.1
        controls[prefix + "_epsilon"].Value = params_dict["epsilon"]
        epsilon_label = EtoForms.Label()
        epsilon_label.Text = "Epsilon (Distance Tolerance):"
        ransac_layout.AddRow(epsilon_label, controls[prefix + "_epsilon"])

        # Cluster Epsilon
        controls[prefix + "_cluster_epsilon"] = EtoForms.NumericStepper()
        controls[prefix + "_cluster_epsilon"].DecimalPlaces = 2
        controls[prefix + "_cluster_epsilon"].MinValue = 0.1
        controls[prefix + "_cluster_epsilon"].MaxValue = 10.0
        controls[prefix + "_cluster_epsilon"].Increment = 0.1
        controls[prefix + "_cluster_epsilon"].Value = params_dict["cluster_epsilon"]
        cluster_epsilon_label = EtoForms.Label()
        cluster_epsilon_label.Text = "Cluster Epsilon:"
        ransac_layout.AddRow(cluster_epsilon_label, controls[prefix + "_cluster_epsilon"])

        # Normal Threshold
        controls[prefix + "_normal_threshold"] = EtoForms.NumericStepper()
        controls[prefix + "_normal_threshold"].DecimalPlaces = 2
        controls[prefix + "_normal_threshold"].MinValue = 0.1
        controls[prefix + "_normal_threshold"].MaxValue = 1.0
        controls[prefix + "_normal_threshold"].Increment = 0.1
        controls[prefix + "_normal_threshold"].Value = params_dict["normal_threshold"]
        normal_threshold_label = EtoForms.Label()
        normal_threshold_label.Text = "Normal Threshold:"
        ransac_layout.AddRow(normal_threshold_label, controls[prefix + "_normal_threshold"])

        # Probability
        controls[prefix + "_probability"] = EtoForms.NumericStepper()
        controls[prefix + "_probability"].DecimalPlaces = 3
        controls[prefix + "_probability"].MinValue = 0.001
        controls[prefix + "_probability"].MaxValue = 0.5
        controls[prefix + "_probability"].Increment = 0.001
        controls[prefix + "_probability"].Value = params_dict["probability"]
        probability_label = EtoForms.Label()
        probability_label.Text = "Probability:"
        ransac_layout.AddRow(probability_label, controls[prefix + "_probability"])

        ransac_group.Content = ransac_layout

        # Store controls for later access
        setattr(self, prefix + "_controls", controls)

        # Add description
        description_text = self.GetShapeDescription(shape_name)
        info_label = EtoForms.Label()
        info_label.Text = description_text
        if hasattr(info_label, 'Wrap'):
            info_label.Wrap = EtoForms.WrapMode.Word

        layout.AddRow(ransac_group)
        layout.AddRow(info_label)
        layout.AddRow(None)  # Spacer

        return layout
    
    def CreatePCLTab(self):
        """Create the PCL parameters tab"""
        layout = EtoForms.DynamicLayout()
        layout.Spacing = EtoDrawing.Size(5, 5)
        layout.Padding = EtoDrawing.Padding(10)
        
        # PCL Plane Parameters
        plane_group = EtoForms.GroupBox()
        plane_group.Text = "PCL Plane Detection"
        
        plane_layout = EtoForms.DynamicLayout()
        plane_layout.Spacing = EtoDrawing.Size(5, 5)
        plane_layout.Padding = EtoDrawing.Padding(10)
        
        # Distance threshold
        self.pcl_plane_distance_box = EtoForms.NumericStepper()
        self.pcl_plane_distance_box.DecimalPlaces = 2
        self.pcl_plane_distance_box.MinValue = 0.1
        self.pcl_plane_distance_box.MaxValue = 10.0
        self.pcl_plane_distance_box.Increment = 0.1
        self.pcl_plane_distance_box.Value = self.pcl_params["distance_threshold_plane"]
        plane_distance_label = EtoForms.Label()
        plane_distance_label.Text = "Distance Threshold:"
        plane_layout.AddRow(plane_distance_label, self.pcl_plane_distance_box)
        
        # Normal distance weight
        self.pcl_normal_weight_box = EtoForms.NumericStepper()
        self.pcl_normal_weight_box.DecimalPlaces = 2
        self.pcl_normal_weight_box.MinValue = 0.01
        self.pcl_normal_weight_box.MaxValue = 1.0
        self.pcl_normal_weight_box.Increment = 0.01
        self.pcl_normal_weight_box.Value = self.pcl_params["normal_distance_weight"]
        normal_weight_label = EtoForms.Label()
        normal_weight_label.Text = "Normal Distance Weight:"
        plane_layout.AddRow(normal_weight_label, self.pcl_normal_weight_box)
        
        # Max iterations
        self.pcl_plane_iterations_box = EtoForms.NumericStepper()
        self.pcl_plane_iterations_box.DecimalPlaces = 0
        self.pcl_plane_iterations_box.MinValue = 100
        self.pcl_plane_iterations_box.MaxValue = 10000
        self.pcl_plane_iterations_box.Increment = 100
        self.pcl_plane_iterations_box.Value = self.pcl_params["max_iterations_plane"]
        plane_iterations_label = EtoForms.Label()
        plane_iterations_label.Text = "Max Iterations:"
        plane_layout.AddRow(plane_iterations_label, self.pcl_plane_iterations_box)
        
        plane_group.Content = plane_layout
        
        # PCL Cylinder Parameters
        cylinder_group = EtoForms.GroupBox()
        cylinder_group.Text = "PCL Cylinder Detection"
        
        cylinder_layout = EtoForms.DynamicLayout()
        cylinder_layout.Spacing = EtoDrawing.Size(5, 5)
        cylinder_layout.Padding = EtoDrawing.Padding(10)
        
        # Distance threshold
        self.pcl_cylinder_distance_box = EtoForms.NumericStepper()
        self.pcl_cylinder_distance_box.DecimalPlaces = 2
        self.pcl_cylinder_distance_box.MinValue = 0.1
        self.pcl_cylinder_distance_box.MaxValue = 10.0
        self.pcl_cylinder_distance_box.Increment = 0.1
        self.pcl_cylinder_distance_box.Value = self.pcl_params["distance_threshold_cylinder"]
        cylinder_distance_label = EtoForms.Label()
        cylinder_distance_label.Text = "Distance Threshold:"
        cylinder_layout.AddRow(cylinder_distance_label, self.pcl_cylinder_distance_box)
        
        # Normal distance weight for cylinder
        self.pcl_cylinder_normal_weight_box = EtoForms.NumericStepper()
        self.pcl_cylinder_normal_weight_box.DecimalPlaces = 2
        self.pcl_cylinder_normal_weight_box.MinValue = 0.01
        self.pcl_cylinder_normal_weight_box.MaxValue = 1.0
        self.pcl_cylinder_normal_weight_box.Increment = 0.01
        self.pcl_cylinder_normal_weight_box.Value = self.pcl_params["normal_weight_cylinder"]
        cylinder_normal_weight_label = EtoForms.Label()
        cylinder_normal_weight_label.Text = "Normal Distance Weight:"
        cylinder_layout.AddRow(cylinder_normal_weight_label, self.pcl_cylinder_normal_weight_box)
        
        # Radius limits
        self.pcl_radius_min_box = EtoForms.NumericStepper()
        self.pcl_radius_min_box.DecimalPlaces = 2
        self.pcl_radius_min_box.MinValue = 0.1
        self.pcl_radius_min_box.MaxValue = 100.0
        self.pcl_radius_min_box.Increment = 0.1
        self.pcl_radius_min_box.Value = self.pcl_params["cylinder_radius_min"]
        radius_min_label = EtoForms.Label()
        radius_min_label.Text = "Min Radius:"
        cylinder_layout.AddRow(radius_min_label, self.pcl_radius_min_box)
        
        self.pcl_radius_max_box = EtoForms.NumericStepper()
        self.pcl_radius_max_box.DecimalPlaces = 2
        self.pcl_radius_max_box.MinValue = 1.0
        self.pcl_radius_max_box.MaxValue = 1000.0
        self.pcl_radius_max_box.Increment = 1.0
        self.pcl_radius_max_box.Value = self.pcl_params["cylinder_radius_max"]
        radius_max_label = EtoForms.Label()
        radius_max_label.Text = "Max Radius:"
        cylinder_layout.AddRow(radius_max_label, self.pcl_radius_max_box)
        
        # Max iterations
        self.pcl_cylinder_iterations_box = EtoForms.NumericStepper()
        self.pcl_cylinder_iterations_box.DecimalPlaces = 0
        self.pcl_cylinder_iterations_box.MinValue = 1000
        self.pcl_cylinder_iterations_box.MaxValue = 100000
        self.pcl_cylinder_iterations_box.Increment = 1000
        self.pcl_cylinder_iterations_box.Value = self.pcl_params["max_iterations_cylinder"]
        cylinder_iterations_label = EtoForms.Label()
        cylinder_iterations_label.Text = "Max Iterations:"
        cylinder_layout.AddRow(cylinder_iterations_label, self.pcl_cylinder_iterations_box)
        
        cylinder_group.Content = cylinder_layout
        
        # Add info
        info_label = EtoForms.Label()
        info_label.Text = "PCL (Point Cloud Library) specific parameters for plane and cylinder detection using RANSAC."
        if hasattr(info_label, 'Wrap'):
            info_label.Wrap = EtoForms.WrapMode.Word
        
        layout.AddRow(plane_group)
        layout.AddRow(cylinder_group)
        layout.AddRow(info_label)
        layout.AddRow(None)  # Spacer
        
        return layout
    
    def CreateButtonPanel(self):
        """Create the button panel at the bottom"""
        layout = EtoForms.DynamicLayout()
        layout.Spacing = EtoDrawing.Size(5, 5)
        
        # Save button
        save_button = EtoForms.Button()
        save_button.Text = "Save Settings"
        save_button.Size = EtoDrawing.Size(100, 30)
        save_button.Click += self.OnSaveClick
        
        # Open button
        open_button = EtoForms.Button()
        open_button.Text = "Open Settings"
        open_button.Size = EtoDrawing.Size(100, 30)
        open_button.Click += self.OnOpenClick
        
        # Run button
        run_button = EtoForms.Button()
        run_button.Text = "Run Processor"
        run_button.Size = EtoDrawing.Size(100, 30)
        run_button.Click += self.OnRunClick
        
        # Reset button
        reset_button = EtoForms.Button()
        reset_button.Text = "Reset to Defaults"
        reset_button.Size = EtoDrawing.Size(120, 30)
        reset_button.Click += self.OnResetClick
        
        # Close button
        close_button = EtoForms.Button()
        close_button.Text = "Close"
        close_button.Size = EtoDrawing.Size(80, 30)
        close_button.Click += self.OnCloseClick
        
        layout.AddRow(save_button, open_button, run_button, reset_button, None, close_button)
        
        return layout
    
    def GetShapeDescription(self, shape_name):
        """Get description text for each shape type"""
        descriptions = {
            "Plane": "Min Points: Minimum number of points required to define a plane.\n"
                    "Epsilon: Maximum distance a point can be from the plane to be considered an inlier.\n"
                    "Cluster Epsilon: Maximum distance for clustering points into the same plane.\n"
                    "Normal Threshold: Minimum dot product between point normal and plane normal.\n"
                    "Probability: Probability that at least one uncontaminated sample is chosen.",
            
            "Cylinder": "Min Points: Minimum number of points required to define a cylinder.\n"
                       "Epsilon: Maximum distance a point can be from the cylinder surface.\n"
                       "Cluster Epsilon: Maximum distance for clustering points into the same cylinder.\n"
                       "Normal Threshold: Minimum alignment between point normal and cylinder surface normal.\n"
                       "Probability: Lower values = more iterations = better accuracy.",
            
            "Cone": "Min Points: Minimum number of points required to define a cone.\n"
                   "Epsilon: Maximum distance a point can be from the cone surface.\n"
                   "Cluster Epsilon: Maximum distance for clustering points into the same cone.\n"
                   "Normal Threshold: Minimum alignment between point normal and cone surface normal.\n"
                   "Probability: Lower values = more iterations = better accuracy.",
            
            "Torus": "Min Points: Minimum number of points required to define a torus.\n"
                    "Epsilon: Maximum distance a point can be from the torus surface.\n"
                    "Cluster Epsilon: Maximum distance for clustering points into the same torus.\n"
                    "Normal Threshold: Minimum alignment between point normal and torus surface normal.\n"
                    "Probability: Lower values = more iterations = better accuracy."
        }
        return descriptions.get(shape_name, "")
    
    def UpdateParametersFromUI(self):
        """Update parameter dictionaries from UI controls"""
        # General parameters
        self.general_params["cluster_tolerance"] = float(self.cluster_tolerance_box.Value)
        self.general_params["min_cluster_size"] = int(self.min_cluster_size_box.Value)
        self.general_params["max_cluster_size"] = int(self.max_cluster_size_box.Value)
        self.general_params["max_shapes_per_cluster"] = int(self.max_shapes_box.Value)
        self.general_params["min_points_threshold"] = int(self.min_points_threshold_box.Value)
        # Angle tolerance (degrees)
        try:
            self.general_params["angle_tolerance_deg"] = float(self.angle_tolerance_box.Value)
        except Exception:
            # In case the control is missing for some reason, keep existing value
            pass
        
        # Shape parameters (CGAL tabs removed; only update if controls exist)
        for shape_name, params_dict in [("plane", self.plane_params), ("cylinder", self.cylinder_params), 
                        ("cone", self.cone_params), ("torus", self.torus_params)]:
            if hasattr(self, shape_name + "_controls"):
                controls = getattr(self, shape_name + "_controls")
                params_dict["min_points"] = int(controls[shape_name + "_min_points"].Value)
                params_dict["epsilon"] = float(controls[shape_name + "_epsilon"].Value)
                params_dict["cluster_epsilon"] = float(controls[shape_name + "_cluster_epsilon"].Value)
                params_dict["normal_threshold"] = float(controls[shape_name + "_normal_threshold"].Value)
                params_dict["probability"] = float(controls[shape_name + "_probability"].Value)
                
                # PCL parameters
                self.pcl_params["distance_threshold_plane"] = float(self.pcl_plane_distance_box.Value)
                self.pcl_params["normal_distance_weight"] = float(self.pcl_normal_weight_box.Value)
                self.pcl_params["max_iterations_plane"] = int(self.pcl_plane_iterations_box.Value)
                self.pcl_params["distance_threshold_cylinder"] = float(self.pcl_cylinder_distance_box.Value)
                self.pcl_params["normal_weight_cylinder"] = float(self.pcl_cylinder_normal_weight_box.Value)
                self.pcl_params["cylinder_radius_min"] = float(self.pcl_radius_min_box.Value)
                self.pcl_params["cylinder_radius_max"] = float(self.pcl_radius_max_box.Value)
                self.pcl_params["max_iterations_cylinder"] = int(self.pcl_cylinder_iterations_box.Value)
    
    def UpdateUIFromParameters(self):
        """Update UI controls from parameter dictionaries"""
        # General parameters
        self.cluster_tolerance_box.Value = self.general_params["cluster_tolerance"]
        self.min_cluster_size_box.Value = self.general_params["min_cluster_size"]
        self.max_cluster_size_box.Value = self.general_params["max_cluster_size"]
        self.max_shapes_box.Value = self.general_params["max_shapes_per_cluster"]
        self.min_points_threshold_box.Value = self.general_params["min_points_threshold"]
        # Angle tolerance
        if hasattr(self, 'angle_tolerance_box'):
            self.angle_tolerance_box.Value = self.general_params.get("angle_tolerance_deg", 5.0)
        
        # Shape parameters
        for shape_name, params_dict in [("plane", self.plane_params), ("cylinder", self.cylinder_params), 
                                       ("cone", self.cone_params), ("torus", self.torus_params)]:
            if hasattr(self, shape_name + "_controls"):
                controls = getattr(self, shape_name + "_controls")
                controls[shape_name + "_min_points"].Value = params_dict["min_points"]
                controls[shape_name + "_epsilon"].Value = params_dict["epsilon"]
                controls[shape_name + "_cluster_epsilon"].Value = params_dict["cluster_epsilon"]
                controls[shape_name + "_normal_threshold"].Value = params_dict["normal_threshold"]
                controls[shape_name + "_probability"].Value = params_dict["probability"]
        
        # PCL parameters
        if hasattr(self, 'pcl_plane_distance_box'):
            self.pcl_plane_distance_box.Value = self.pcl_params["distance_threshold_plane"]
            self.pcl_normal_weight_box.Value = self.pcl_params["normal_distance_weight"]
            self.pcl_plane_iterations_box.Value = self.pcl_params["max_iterations_plane"]
            self.pcl_cylinder_distance_box.Value = self.pcl_params["distance_threshold_cylinder"]
            self.pcl_cylinder_normal_weight_box.Value = self.pcl_params["normal_weight_cylinder"]
            self.pcl_radius_min_box.Value = self.pcl_params["cylinder_radius_min"]
            self.pcl_radius_max_box.Value = self.pcl_params["cylinder_radius_max"]
            self.pcl_cylinder_iterations_box.Value = self.pcl_params["max_iterations_cylinder"]
    
    def OnSaveClick(self, sender, e):
        """Handle Save button click - Opens file dialog to save settings"""
        try:
            self.UpdateParametersFromUI()
            
            # Create save file dialog
            save_dialog = EtoForms.SaveFileDialog()
            save_dialog.Title = "Save PCL Settings"
            save_dialog.Filters.Add(EtoForms.FileFilter("JSON Files", ".json"))
            save_dialog.Filters.Add(EtoForms.FileFilter("All Files", ".*"))
            save_dialog.FileName = "pcl_settings.json"
            
            if save_dialog.ShowDialog(self) == EtoForms.DialogResult.Ok:
                self.SaveSettingsToFile(save_dialog.FileName)
                rs.MessageBox("Settings saved successfully to:\n" + str(save_dialog.FileName))
            
        except Exception as ex:
            rs.MessageBox("Error saving settings:" + str(ex), "Error")
    
    def OnOpenClick(self, sender, e):
        """Handle Open button click - Opens file dialog to load settings"""
        try:
            # Create open file dialog
            open_dialog = EtoForms.OpenFileDialog()
            open_dialog.Title = "Open PCL Settings"
            open_dialog.Filters.Add(EtoForms.FileFilter("JSON Files", ".json"))
            open_dialog.Filters.Add(EtoForms.FileFilter("All Files", ".*"))
            
            if open_dialog.ShowDialog(self) == EtoForms.DialogResult.Ok:
                if self.LoadSettingsFromFile(open_dialog.FileName):
                    self.UpdateUIFromParameters()
                    rs.MessageBox("Settings loaded successfully from:\n"+ open_dialog.FileName)
                else:
                    rs.MessageBox("Failed to load settings file or file is empty.")
            
        except Exception as ex:
            rs.MessageBox("Error loading settings: " + str(ex), "Error")
    
    def OnRunClick(self, sender, e):
        """Handle Run button click - Saves settings to specific location and runs processor"""
        self.Close()
        try:
            self.UpdateParametersFromUI()
            # Save current settings to the processor location
            processor_settings_file = "C:\\Temp\\pcl_processing\\settings.json"
            self.SaveSettingsToFile(processor_settings_file)
            # Run PCL processing using the runPCL module
            print("Starting PCL processing...")
            success = rhino_client.run_pcl_processing("BEST")
            if success:
                full_message = "Settings saved to: " + processor_settings_file + "\n\n"
                full_message += "PCL Processing completed successfully!\n\n"
                #full_message += message
                rs.MessageBox(full_message)
                # Re-open the main modal dialog box
                ShowPCLProcessorSettings()
            else:
                full_message = "Settings saved to: " + processor_settings_file + "\n\n"
                full_message += "PCL Processing failed:\n\n"
                #full_message += message
                rs.MessageBox(full_message)
        except Exception as ex:
            rs.MessageBox("Error running processor: " + str(ex))
    
    def OnResetClick(self, sender, e):
        """Handle Reset button click"""
        try:
            import json, os
            settings_path = os.path.join('data', 'base_settings')
            with open(settings_path, 'r') as f:
                settings = json.load(f)
            self.general_params = settings.get('general', {})
            self.plane_params = settings.get('plane', {})
            self.cylinder_params = settings.get('cylinder', {})
            self.cone_params = settings.get('cone', {})
            self.torus_params = settings.get('torus', {})
            self.pcl_params = settings.get('pcl', {})
            self.UpdateUIFromParameters()
            rs.MessageBox("Settings reset from base_settings!")
            
        except Exception as ex:
            rs.MessageBox("Error resetting settings: " + str(ex), "Error")
    
    def OnCloseClick(self, sender, e):
        """Handle Close button click"""
        self.Close()
    
    def SaveSettings(self):
        """Save settings to default JSON file (for backward compatibility)"""
        self.SaveSettingsToFile(self.settings_file)
    
    def SaveSettingsToFile(self, file_path):
        """Save settings to specified JSON file"""
        # Ensure directory exists
        directory = os.path.dirname(file_path)
        if not os.path.exists(directory):
            os.makedirs(directory)
        
        settings_data = {
            "general": self.general_params,
            "plane": self.plane_params,
            "cylinder": self.cylinder_params,
            "cone": self.cone_params,
            "torus": self.torus_params,
            "pcl": self.pcl_params,
            "version": "1.0",
            "timestamp": System.DateTime.Now.ToString()
        }
        
        with open(file_path, 'w') as f:
            json.dump(settings_data, f, indent=4)
    
    def LoadSettings(self):
        """Load settings from default JSON file (for backward compatibility)"""
        return self.LoadSettingsFromFile(self.settings_file)
    
    def LoadSettingsFromFile(self, file_path):
        """Load settings from specified JSON file"""
        if not os.path.exists(file_path):
            return False
        
        try:
            with open(file_path, 'r') as f:
                settings_data = json.load(f)
            
            if "general" in settings_data:
                self.general_params.update(settings_data["general"])
            if "plane" in settings_data:
                self.plane_params.update(settings_data["plane"])
            if "cylinder" in settings_data:
                self.cylinder_params.update(settings_data["cylinder"])
            if "cone" in settings_data:
                self.cone_params.update(settings_data["cone"])
            if "torus" in settings_data:
                self.torus_params.update(settings_data["torus"])
            if "pcl" in settings_data:
                self.pcl_params.update(settings_data["pcl"])
            
            return True
        except Exception as ex:
            print("Error loading settings: " + str(ex))
            return False


def ShowPCLProcessorSettings():
    """Main function to show the settings dialog"""
    result = True
    while(result):
        try:
            form = PCLProcessorSettingsForm()
            result = ( form.ShowModal(Rhino.UI.RhinoEtoApp.MainWindow))
        except Exception as ex:
            rs.MessageBox("Error creating form: " + str(ex), 0)
            return False


# Run the script
if __name__ == "__main__":
    ShowPCLProcessorSettings()
