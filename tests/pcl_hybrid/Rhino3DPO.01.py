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
import rhino_client  # Import our PCL processing module


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
            "min_points_threshold": 50
        }
        
        # ShapeFinder / clustering pipeline parameters (mapped from C++ ShapeFinder.cpp)
        self.shapefinder_params = {
            "parent_cluster_tolerance": 0.1,
            "parent_min_cluster_size": 50,
            "parent_max_cluster_size": 10000000,
            "small_angle_deg": 3.0,
            "angle_tolerance_deg": 5.0,
            "preg_min_cluster_size": 25,
            "preg_max_cluster_size": 1000000,
            "preg_number_of_neighbours": 50,
            "parent_ksearch": 50
        }
        
        # (CGAL shape tabs removed — settings moved/centralized in ShapeFinder)
        
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
        
        # PCWinPointCloud / low-level import parameters
        self.pcwin_params = {
            "normal_ksearch": 16,
            "import_reserve": 100000
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
        
    # (CGAL shape tabs removed — edit shape-related parameters in ShapeFinder)
        
        # ShapeFinder Parameters Tab
        sf_tab = EtoForms.TabPage()
        sf_tab.Text = "ShapeFinder"
        sf_tab.Content = self.CreateShapeFinderTab()
        tab_control.Pages.Add(sf_tab)

        # PCWinPointCloud Tab (low-level import / normals)
        pcwin_tab = EtoForms.TabPage()
        pcwin_tab.Text = "PCWinPointCloud"
        pcwin_tab.Content = self.CreatePCWinTab()
        tab_control.Pages.Add(pcwin_tab)
        
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
    
    # CGAL shape tabs removed; shape params are managed centrally in ShapeFinder
    
    def CreateShapeTab(self, shape_name, params_dict, prefix):
        """Create a generic shape parameters tab"""
        layout = EtoForms.DynamicLayout()
        layout.Spacing = EtoDrawing.Size(5, 5)
        layout.Padding = EtoDrawing.Padding(10)
        
        # CGAL RANSAC Parameters Section
        ransac_group = EtoForms.GroupBox()
        ransac_group.Text = f"CGAL Efficient RANSAC - {shape_name}"
        
        ransac_layout = EtoForms.DynamicLayout()
        ransac_layout.Spacing = EtoDrawing.Size(5, 5)
        ransac_layout.Padding = EtoDrawing.Padding(10)
        
        # Create controls and store references
        controls = {}
        
        # Min Points
        controls[f"{prefix}_min_points"] = EtoForms.NumericStepper()
        controls[f"{prefix}_min_points"].DecimalPlaces = 0
        controls[f"{prefix}_min_points"].MinValue = 3
        controls[f"{prefix}_min_points"].MaxValue = 100
        controls[f"{prefix}_min_points"].Increment = 1
        controls[f"{prefix}_min_points"].Value = params_dict["min_points"]
        min_points_label = EtoForms.Label()
        min_points_label.Text = "Min Points:"
        ransac_layout.AddRow(min_points_label, controls[f"{prefix}_min_points"])
        
        # Epsilon
        controls[f"{prefix}_epsilon"] = EtoForms.NumericStepper()
        controls[f"{prefix}_epsilon"].DecimalPlaces = 2
        controls[f"{prefix}_epsilon"].MinValue = 0.1
        controls[f"{prefix}_epsilon"].MaxValue = 10.0
        controls[f"{prefix}_epsilon"].Increment = 0.1
        controls[f"{prefix}_epsilon"].Value = params_dict["epsilon"]
        epsilon_label = EtoForms.Label()
        epsilon_label.Text = "Epsilon (Distance Tolerance):"
        ransac_layout.AddRow(epsilon_label, controls[f"{prefix}_epsilon"])
        
        # Cluster Epsilon
        controls[f"{prefix}_cluster_epsilon"] = EtoForms.NumericStepper()
        controls[f"{prefix}_cluster_epsilon"].DecimalPlaces = 2
        controls[f"{prefix}_cluster_epsilon"].MinValue = 0.1
        controls[f"{prefix}_cluster_epsilon"].MaxValue = 10.0
        controls[f"{prefix}_cluster_epsilon"].Increment = 0.1
        controls[f"{prefix}_cluster_epsilon"].Value = params_dict["cluster_epsilon"]
        cluster_epsilon_label = EtoForms.Label()
        cluster_epsilon_label.Text = "Cluster Epsilon:"
        ransac_layout.AddRow(cluster_epsilon_label, controls[f"{prefix}_cluster_epsilon"])
        
        # Normal Threshold
        controls[f"{prefix}_normal_threshold"] = EtoForms.NumericStepper()
        controls[f"{prefix}_normal_threshold"].DecimalPlaces = 2
        controls[f"{prefix}_normal_threshold"].MinValue = 0.1
        controls[f"{prefix}_normal_threshold"].MaxValue = 1.0
        controls[f"{prefix}_normal_threshold"].Increment = 0.1
        controls[f"{prefix}_normal_threshold"].Value = params_dict["normal_threshold"]
        normal_threshold_label = EtoForms.Label()
        normal_threshold_label.Text = "Normal Threshold:"
        ransac_layout.AddRow(normal_threshold_label, controls[f"{prefix}_normal_threshold"])
        
        # Probability
        controls[f"{prefix}_probability"] = EtoForms.NumericStepper()
        controls[f"{prefix}_probability"].DecimalPlaces = 3
        controls[f"{prefix}_probability"].MinValue = 0.001
        controls[f"{prefix}_probability"].MaxValue = 0.5
        controls[f"{prefix}_probability"].Increment = 0.001
        controls[f"{prefix}_probability"].Value = params_dict["probability"]
        probability_label = EtoForms.Label()
        probability_label.Text = "Probability:"
        ransac_layout.AddRow(probability_label, controls[f"{prefix}_probability"])
        
        ransac_group.Content = ransac_layout
        
        # Store controls for later access
        setattr(self, f"{prefix}_controls", controls)
        
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

    def CreateShapeFinderTab(self):
        """Create a tab for ShapeFinder / clustering pipeline parameters"""
        layout = EtoForms.DynamicLayout()
        layout.Spacing = EtoDrawing.Size(5, 5)
        layout.Padding = EtoDrawing.Padding(10)

        group = EtoForms.GroupBox()
        group.Text = "ShapeFinder / Clustering"
        gl = EtoForms.DynamicLayout()
        gl.Spacing = EtoDrawing.Size(5,5)
        gl.Padding = EtoDrawing.Padding(10)

        # Parent cluster tolerance
        self.sf_parent_tol = EtoForms.NumericStepper()
        self.sf_parent_tol.DecimalPlaces = 3
        self.sf_parent_tol.MinValue = 0.0
        self.sf_parent_tol.MaxValue = 10.0
        self.sf_parent_tol.Increment = 0.01
        self.sf_parent_tol.Value = self.shapefinder_params["parent_cluster_tolerance"]
        gl.AddRow(EtoForms.Label(Text="Parent Cluster Tolerance:"), self.sf_parent_tol)

        # Parent cluster sizes
        self.sf_parent_min = EtoForms.NumericStepper()
        self.sf_parent_min.DecimalPlaces = 0
        self.sf_parent_min.MinValue = 1
        self.sf_parent_min.MaxValue = 10000000
        self.sf_parent_min.Increment = 1
        self.sf_parent_min.Value = self.shapefinder_params["parent_min_cluster_size"]
        gl.AddRow(EtoForms.Label(Text="Parent Min Cluster Size:"), self.sf_parent_min)

        self.sf_parent_max = EtoForms.NumericStepper()
        self.sf_parent_max.DecimalPlaces = 0
        self.sf_parent_max.MinValue = 1
        self.sf_parent_max.MaxValue = 100000000
        self.sf_parent_max.Increment = 100
        self.sf_parent_max.Value = self.shapefinder_params["parent_max_cluster_size"]
        gl.AddRow(EtoForms.Label(Text="Parent Max Cluster Size:"), self.sf_parent_max)

        # Small angle deg
        self.sf_small_angle = EtoForms.NumericStepper()
        self.sf_small_angle.DecimalPlaces = 2
        self.sf_small_angle.MinValue = 0.0
        self.sf_small_angle.MaxValue = 90.0
        self.sf_small_angle.Increment = 0.1
        self.sf_small_angle.Value = self.shapefinder_params["small_angle_deg"]
        gl.AddRow(EtoForms.Label(Text="Small Angle (deg):"), self.sf_small_angle)

        # Preg cluster sizes and neighbours
        self.sf_preg_min = EtoForms.NumericStepper()
        self.sf_preg_min.DecimalPlaces = 0
        self.sf_preg_min.MinValue = 1
        self.sf_preg_min.MaxValue = 1000000
        self.sf_preg_min.Increment = 1
        self.sf_preg_min.Value = self.shapefinder_params["preg_min_cluster_size"]
        gl.AddRow(EtoForms.Label(Text="PREG Min Cluster Size:"), self.sf_preg_min)

        self.sf_preg_max = EtoForms.NumericStepper()
        self.sf_preg_max.DecimalPlaces = 0
        self.sf_preg_max.MinValue = 1
        self.sf_preg_max.MaxValue = 100000000
        self.sf_preg_max.Increment = 100
        self.sf_preg_max.Value = self.shapefinder_params["preg_max_cluster_size"]
        gl.AddRow(EtoForms.Label(Text="PREG Max Cluster Size:"), self.sf_preg_max)

        self.sf_preg_neigh = EtoForms.NumericStepper()
        self.sf_preg_neigh.DecimalPlaces = 0
        self.sf_preg_neigh.MinValue = 1
        self.sf_preg_neigh.MaxValue = 1000
        self.sf_preg_neigh.Increment = 1
        self.sf_preg_neigh.Value = self.shapefinder_params["preg_number_of_neighbours"]
        gl.AddRow(EtoForms.Label(Text="PREG Neighbours (k):"), self.sf_preg_neigh)

        self.sf_parent_ksearch = EtoForms.NumericStepper()
        self.sf_parent_ksearch.DecimalPlaces = 0
        self.sf_parent_ksearch.MinValue = 1
        self.sf_parent_ksearch.MaxValue = 500
        self.sf_parent_ksearch.Increment = 1
        self.sf_parent_ksearch.Value = self.shapefinder_params["parent_ksearch"]
        gl.AddRow(EtoForms.Label(Text="Parent Normal k-search:"), self.sf_parent_ksearch)

        group.Content = gl
        layout.AddRow(group)
        layout.AddRow(None)
        return layout

    def CreatePCWinTab(self):
        """Create a tab for PCWinPointCloud / import parameters"""
        layout = EtoForms.DynamicLayout()
        layout.Spacing = EtoDrawing.Size(5,5)
        layout.Padding = EtoDrawing.Padding(10)

        group = EtoForms.GroupBox()
        group.Text = "PCWinPointCloud (import / normals)"
        gl = EtoForms.DynamicLayout()
        gl.Spacing = EtoDrawing.Size(5,5)
        gl.Padding = EtoDrawing.Padding(10)

        self.pc_norm_k = EtoForms.NumericStepper()
        self.pc_norm_k.DecimalPlaces = 0
        self.pc_norm_k.MinValue = 1
        self.pc_norm_k.MaxValue = 200
        self.pc_norm_k.Increment = 1
        self.pc_norm_k.Value = self.pcwin_params["normal_ksearch"]
        gl.AddRow(EtoForms.Label(Text="Normal Estimation k-search:"), self.pc_norm_k)

        self.pc_import_reserve = EtoForms.NumericStepper()
        self.pc_import_reserve.DecimalPlaces = 0
        self.pc_import_reserve.MinValue = 0
        self.pc_import_reserve.MaxValue = 100000000
        self.pc_import_reserve.Increment = 100
        self.pc_import_reserve.Value = self.pcwin_params["import_reserve"]
        gl.AddRow(EtoForms.Label(Text="Import reserve (hint):"), self.pc_import_reserve)

        group.Content = gl
        layout.AddRow(group)
        layout.AddRow(None)
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
        
        # Shape parameters
        for shape_name, params_dict in [("plane", self.plane_params), ("cylinder", self.cylinder_params), 
                                       ("cone", self.cone_params), ("torus", self.torus_params)]:
            controls = getattr(self, f"{shape_name}_controls")
            params_dict["min_points"] = int(controls[f"{shape_name}_min_points"].Value)
            params_dict["epsilon"] = float(controls[f"{shape_name}_epsilon"].Value)
            params_dict["cluster_epsilon"] = float(controls[f"{shape_name}_cluster_epsilon"].Value)
            params_dict["normal_threshold"] = float(controls[f"{shape_name}_normal_threshold"].Value)
            params_dict["probability"] = float(controls[f"{shape_name}_probability"].Value)
        
        # PCL parameters
        self.pcl_params["distance_threshold_plane"] = float(self.pcl_plane_distance_box.Value)
        self.pcl_params["normal_distance_weight"] = float(self.pcl_normal_weight_box.Value)
        self.pcl_params["max_iterations_plane"] = int(self.pcl_plane_iterations_box.Value)
        self.pcl_params["distance_threshold_cylinder"] = float(self.pcl_cylinder_distance_box.Value)
        self.pcl_params["normal_weight_cylinder"] = float(self.pcl_cylinder_normal_weight_box.Value)
        self.pcl_params["cylinder_radius_min"] = float(self.pcl_radius_min_box.Value)
        self.pcl_params["cylinder_radius_max"] = float(self.pcl_radius_max_box.Value)
        self.pcl_params["max_iterations_cylinder"] = int(self.pcl_cylinder_iterations_box.Value)
        # ShapeFinder parameters
        self.shapefinder_params["parent_cluster_tolerance"] = float(self.sf_parent_tol.Value)
        self.shapefinder_params["parent_min_cluster_size"] = int(self.sf_parent_min.Value)
        self.shapefinder_params["parent_max_cluster_size"] = int(self.sf_parent_max.Value)
        self.shapefinder_params["small_angle_deg"] = float(self.sf_small_angle.Value)
        self.shapefinder_params["preg_min_cluster_size"] = int(self.sf_preg_min.Value)
        self.shapefinder_params["preg_max_cluster_size"] = int(self.sf_preg_max.Value)
        self.shapefinder_params["preg_number_of_neighbours"] = int(self.sf_preg_neigh.Value)
        self.shapefinder_params["parent_ksearch"] = int(self.sf_parent_ksearch.Value)

        # PCWin parameters
        self.pcwin_params["normal_ksearch"] = int(self.pc_norm_k.Value)
        self.pcwin_params["import_reserve"] = int(self.pc_import_reserve.Value)
    
    def UpdateUIFromParameters(self):
        """Update UI controls from parameter dictionaries"""
        # General parameters
        self.cluster_tolerance_box.Value = self.general_params["cluster_tolerance"]
        self.min_cluster_size_box.Value = self.general_params["min_cluster_size"]
        self.max_cluster_size_box.Value = self.general_params["max_cluster_size"]
        self.max_shapes_box.Value = self.general_params["max_shapes_per_cluster"]
        self.min_points_threshold_box.Value = self.general_params["min_points_threshold"]
        
        # Shape parameters
        for shape_name, params_dict in [("plane", self.plane_params), ("cylinder", self.cylinder_params), 
                                       ("cone", self.cone_params), ("torus", self.torus_params)]:
            if hasattr(self, f"{shape_name}_controls"):
                controls = getattr(self, f"{shape_name}_controls")
                controls[f"{shape_name}_min_points"].Value = params_dict["min_points"]
                controls[f"{shape_name}_epsilon"].Value = params_dict["epsilon"]
                controls[f"{shape_name}_cluster_epsilon"].Value = params_dict["cluster_epsilon"]
                controls[f"{shape_name}_normal_threshold"].Value = params_dict["normal_threshold"]
                controls[f"{shape_name}_probability"].Value = params_dict["probability"]
        
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

        # ShapeFinder UI
        if hasattr(self, 'sf_parent_tol'):
            self.sf_parent_tol.Value = self.shapefinder_params.get("parent_cluster_tolerance", self.shapefinder_params["parent_cluster_tolerance"])
            self.sf_parent_min.Value = self.shapefinder_params.get("parent_min_cluster_size", self.shapefinder_params["parent_min_cluster_size"])
            self.sf_parent_max.Value = self.shapefinder_params.get("parent_max_cluster_size", self.shapefinder_params["parent_max_cluster_size"])
            self.sf_small_angle.Value = self.shapefinder_params.get("small_angle_deg", self.shapefinder_params["small_angle_deg"])
            self.sf_preg_min.Value = self.shapefinder_params.get("preg_min_cluster_size", self.shapefinder_params["preg_min_cluster_size"])
            self.sf_preg_max.Value = self.shapefinder_params.get("preg_max_cluster_size", self.shapefinder_params["preg_max_cluster_size"])
            self.sf_preg_neigh.Value = self.shapefinder_params.get("preg_number_of_neighbours", self.shapefinder_params["preg_number_of_neighbours"])
            self.sf_parent_ksearch.Value = self.shapefinder_params.get("parent_ksearch", self.shapefinder_params["parent_ksearch"])

        # PCWin UI
        if hasattr(self, 'pc_norm_k'):
            self.pc_norm_k.Value = self.pcwin_params.get("normal_ksearch", self.pcwin_params["normal_ksearch"])
            self.pc_import_reserve.Value = self.pcwin_params.get("import_reserve", self.pcwin_params["import_reserve"])
    
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
                rs.MessageBox(f"Settings saved successfully to:\n{save_dialog.FileName}")
            
        except Exception as ex:
            rs.MessageBox(f"Error saving settings: {str(ex)}", "Error")
    
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
                    rs.MessageBox(f"Settings loaded successfully from:\n{open_dialog.FileName}")
                else:
                    rs.MessageBox("Failed to load settings file or file is empty.")

        except Exception as ex:
            rs.MessageBox(f"Error loading settings: {str(ex)}", "Error")
    
    def OnRunClick(self, sender, e):
        """Handle Run button click - Saves settings to specific location and runs processor"""
        self.Close()
        try:
            self.UpdateParametersFromUI()
            # Save current settings to the processor location
            processor_settings_file = self.settings_file
            proc_settings = self.ExportSettingsForProcessor()
            directory = os.path.dirname(processor_settings_file)
            if not os.path.exists(directory): os.makedirs(directory)
            with open(processor_settings_file, 'w') as f:
                json.dump(proc_settings, f, indent=4)
            # Run PCL processing using the runPCL module
            print("Starting PCL processing...")
            try:
                success, message = rhino_client.run_pcl_processing("BEST", settings_path=processor_settings_file)
            except TypeError:
                # older signature: run_pcl_processing(mode)
                success, message = rhino_client.run_pcl_processing("BEST")

            if success:
                full_message = f"Settings saved to: {processor_settings_file}\n\n"
                full_message += f"✓ PCL Processing completed successfully!\n\n"
                full_message += message
                rs.MessageBox(full_message)
                # Re-open the main modal dialog box
                ShowPCLProcessorSettings()
            else:
                full_message = f"Settings saved to: {processor_settings_file}\n\n"
                full_message += f"✗ PCL Processing failed:\n\n"
                full_message += message
                rs.MessageBox(full_message)
        except Exception as ex:
            rs.MessageBox(f"Error running processor: {str(ex)}")
    
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
            rs.MessageBox(f"Error resetting settings: {str(ex)}", "Error")
    
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
            "shapefinder": self.shapefinder_params,
            "pcwin": self.pcwin_params,
            "version": "1.0",
            "timestamp": System.DateTime.Now.ToString()
        }
        
        with open(file_path, 'w') as f:
            json.dump(settings_data, f, indent=4)

    def ExportSettingsForProcessor(self):
        """Return a combined settings dict that the headless processor expects.

        This collates general, pcl, shapefinder and pcwin sections so the
        external processor can consume a single JSON file.
        """
        combined = {
            "general": self.general_params,
            "pcl": self.pcl_params,
            "shapefinder": self.shapefinder_params,
            "pcwin": self.pcwin_params,
            "version": "1.0",
            "timestamp": System.DateTime.Now.ToString()
        }
        return combined
    
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
            if "shapefinder" in settings_data:
                self.shapefinder_params.update(settings_data["shapefinder"])
            if "pcwin" in settings_data:
                self.pcwin_params.update(settings_data["pcwin"])
            
            return True
        except Exception as ex:
            print(f"Error loading settings: {str(ex)}")
            return False


def ShowPCLProcessorSettings():
    """Main function to show the settings dialog"""
    result = True
    while(result):
        try:
            form = PCLProcessorSettingsForm()
            result = ( form.ShowModal(Rhino.UI.RhinoEtoApp.MainWindow))
        except Exception as ex:
            rs.MessageBox(f"Error creating form: {str(ex)}", 0)
            return False


# Run the script
if __name__ == "__main__":
    ShowPCLProcessorSettings()
