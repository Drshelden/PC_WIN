# Example usage of the Python bindings for pcl_hybrid
from pcl_hybrid_py import PCWin_PointCloud, ShapeFinder

pc = PCWin_PointCloud()
res = pc.importPoints('data/example.pcd')
print('importPoints returned', res)

sf = ShapeFinder()
res2 = sf.findShapes(pc)
print('findShapes returned', res2)

print('clusters:', sf.clusters_size())
for i in range(sf.clusters_size()):
    print(' cluster', i, 'size=', sf.get_cluster_size(i))

print('root json preview:')
print(sf.get_root_json()[:400])
