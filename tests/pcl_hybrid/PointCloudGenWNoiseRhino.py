import rhinoscriptsyntax as rs
import random

def get_surface_areas(surfaces):
    return [rs.SurfaceArea(srf)[0] for srf in surfaces]

def pick_surface_weighted(surfaces, areas, total_area):
    r = random.uniform(0, total_area)
    cumulative = 0.0
    for i, area in enumerate(areas):
        cumulative += area
        if r <= cumulative:
            return surfaces[i]
    return surfaces[-1]

def generate_random_point_on_surface(srf, max_attempts=20):
    u_dom = rs.SurfaceDomain(srf, 0)
    v_dom = rs.SurfaceDomain(srf, 1)
    for _ in range(max_attempts):
        u = random.uniform(u_dom[0], u_dom[1])
        v = random.uniform(v_dom[0], v_dom[1])
        pt = rs.EvaluateSurface(srf, u, v)
        if rs.IsPointOnSurface(srf, pt):
            return pt
    return None  # Fallback if no valid point is found

def add_noise_to_point(pt, noise):
    dx = random.uniform(-noise, noise)
    dy = random.uniform(-noise, noise)
    dz = random.uniform(-noise, noise)
    return rs.CreatePoint(pt.X + dx, pt.Y + dy, pt.Z + dz)

def export_points_to_xyz(points):
    default_path = rs.SaveFileName("Save points as .xyz file", "XYZ Files (*.xyz)|*.xyz||", filename="points.xyz")
    if not default_path:
        print("Export canceled by user.")
        return

    try:
        with open(default_path, 'w') as f:
            for pt in points:
                line = f"{pt.X} {pt.Y} {pt.Z}\n"
                f.write(line)
        print(f"Exported {len(points)} points to {default_path}")
    except Exception as e:
        print(f"Error writing to file: {e}")

def main():
    surfaces = rs.GetObjects("Select one or more surfaces", rs.filter.surface, preselect=True)
    if not surfaces:
        print("No surfaces selected.")
        return

    num_points = rs.GetInteger("How many points to generate?", number=1000, minimum=1)
    if num_points is None:
        print("User canceled input.")
        return

    noise = rs.GetReal("Enter noise factor (in model units)", number=0.0, minimum=0.0)
    if noise is None:
        print("User canceled noise input.")
        return

    areas = get_surface_areas(surfaces)
    total_area = sum(areas)
    points = []

    for _ in range(num_points):
        srf = pick_surface_weighted(surfaces, areas, total_area)
        pt = generate_random_point_on_surface(srf)
        if pt:
            noisy_pt = add_noise_to_point(pt, noise)
            points.append(noisy_pt)

    rs.AddPoints(points)
    print(f"{len(points)} points generated across {len(surfaces)} surfaces with noise ±{noise}.")

    export_points_to_xyz(points)

main()
