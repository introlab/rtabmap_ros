"""
A SLAM graph as the playback tests keep and compare it, exported from rtabmap's
database by rtabmap-export (export_graph):
  <prefix>.g2o  the graph (--poses_format 4). OptimizerG2O::saveGraph() writes each
                edge's link type as a column past the information matrix; that is
                what tells the loop closures apart.
  <prefix>.tum  the optimized poses with their stamps (--poses_format 10: stamp x y z
                qx qy qz qw). The test replays a golden one as ground truth, from which
                rtabmap computes the trajectory error itself (Gt/* statistics).
"""

import shutil
import subprocess
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Tuple

import numpy as np

# rtabmap::Link::Type
NEIGHBOR, GLOBAL_CLOSURE, LOCAL_SPACE_CLOSURE, LOCAL_TIME_CLOSURE = 0, 1, 2, 3


@dataclass
class Graph:
    poses: Dict[int, Tuple[float, ...]] = field(default_factory=dict)  # x y z qx qy qz qw
    link_types: List[int] = field(default_factory=list)

    @classmethod
    def load(cls, g2o_path: str) -> 'Graph':
        """Read the vertices and the edges' types; a 2D graph (Reg/Force3DoF) is SE2."""
        graph = cls()
        # Fields each edge tag defines; the link type is the column after them.
        edge_fields = {'EDGE_SE2': 12, 'EDGE_SE3:QUAT': 31}
        with open(g2o_path) as f:
            for line in f:
                v = line.split()
                if not v:
                    continue
                if v[0] == 'VERTEX_SE3:QUAT':
                    graph.poses[int(v[1])] = tuple(float(x) for x in v[2:9])
                elif v[0] == 'VERTEX_SE2':
                    yaw = float(v[4])
                    graph.poses[int(v[1])] = (float(v[2]), float(v[3]), 0.0,
                                              0.0, 0.0, np.sin(yaw / 2), np.cos(yaw / 2))
                elif v[0] in edge_fields:
                    n = edge_fields[v[0]]
                    graph.link_types.append(int(v[n]) if len(v) > n else NEIGHBOR)
        return graph

    def summary(self) -> dict:
        ids = sorted(self.poses)
        xyz = np.array([self.poses[i][:3] for i in ids]).reshape(-1, 3)
        return {
            'nodes': len(self.poses),
            'global_closures': sum(t == GLOBAL_CLOSURE for t in self.link_types),
            'local_closures': sum(t in (LOCAL_SPACE_CLOSURE, LOCAL_TIME_CLOSURE)
                                  for t in self.link_types),
            'path_length': round(float(np.linalg.norm(np.diff(xyz, axis=0), axis=1).sum()), 2),
        }


def export_graph(database: Path, prefix: Path):
    """Write <prefix>.g2o and <prefix>.tum from an rtabmap database.

    --opt 2 takes the optimized poses rtabmap saved in the database when it closed, so
    they are the run's own, not a re-optimization.
    """
    tool = shutil.which('rtabmap-export')
    if tool is None:
        raise FileNotFoundError('rtabmap-export not found on PATH (RTAB-Map built without '
                                'its tools?)')
    out_dir = database.parent
    for poses_format, extension, suffix in ((4, 'g2o', '.g2o'), (10, 'txt', '.tum')):
        subprocess.run([tool, '--poses', '--poses_format', str(poses_format), '--opt', '2',
                        '--output_dir', str(out_dir), str(database)],
                       check=True, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, text=True)
        exported = out_dir / f'{database.stem}_poses.{extension}'
        exported.replace(str(prefix) + suffix)


def load_tum(path: str) -> List[Tuple[float, Tuple[float, ...]]]:
    """(stamp, (x y z qx qy qz qw)) of each line of a TUM trajectory file."""
    trajectory = []
    with open(path) as f:
        for line in f:
            v = line.split()
            if len(v) == 8 and not line.startswith('#'):
                trajectory.append((float(v[0]), tuple(float(x) for x in v[1:])))
    return trajectory
