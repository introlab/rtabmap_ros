import ast
import pathlib
import sys
import unittest


LAUNCH_DIR = pathlib.Path(__file__).resolve().parents[1] / "launch"
sys.path.insert(0, str(LAUNCH_DIR))

from rtabmap_launch_args import filter_rtabmap_arguments  # noqa: E402


class FilterRtabmapArgumentsTest(unittest.TestCase):
    def test_keeps_non_odom_arguments(self):
        kept, removed = filter_rtabmap_arguments(
            '--Vis/CorType 0 --Mem/IncrementalMemory false'
        )

        self.assertEqual(
            kept,
            ['--Vis/CorType', '0', '--Mem/IncrementalMemory', 'false'],
        )
        self.assertEqual(removed, [])

    def test_filters_odom_parameter_pairs(self):
        kept, removed = filter_rtabmap_arguments(
            '--Vis/CorType 0 --Odom/ResetCountdown 20 --delete_db_on_start'
        )

        self.assertEqual(kept, ['--Vis/CorType', '0', '--delete_db_on_start'])
        self.assertEqual(removed, ['--Odom/ResetCountdown', '20'])

    def test_filters_inline_assignment(self):
        kept, removed = filter_rtabmap_arguments(
            '--Odom/ResetCountdown=20 --Grid/RangeMax 4.0'
        )

        self.assertEqual(kept, ['--Grid/RangeMax', '4.0'])
        self.assertEqual(removed, ['--Odom/ResetCountdown=20'])

    def test_launch_setup_filters_rtabmap_args_source(self):
        launch_file = LAUNCH_DIR / 'rtabmap.launch.py'
        tree = ast.parse(launch_file.read_text(encoding='utf-8'))

        filter_calls = [
            node for node in ast.walk(tree)
            if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Name)
            and node.func.id == 'filter_rtabmap_arguments'
        ]
        self.assertEqual(len(filter_calls), 1)

        filter_arg = filter_calls[0].args[0]
        self.assertIsInstance(filter_arg, ast.Call)
        self.assertIsInstance(filter_arg.func, ast.Attribute)
        self.assertEqual(filter_arg.func.attr, 'perform')
        self.assertIsInstance(filter_arg.func.value, ast.Call)
        self.assertIsInstance(filter_arg.func.value.func, ast.Name)
        self.assertEqual(filter_arg.func.value.func.id, 'LaunchConfiguration')
        self.assertEqual(filter_arg.func.value.args[0].value, 'rtabmap_args')

    def test_odometry_nodes_still_use_args_alias(self):
        launch_file = LAUNCH_DIR / 'rtabmap.launch.py'
        source = launch_file.read_text(encoding='utf-8')

        self.assertIn('arguments=[LaunchConfiguration("args"), LaunchConfiguration("odom_args")', source)


if __name__ == '__main__':
    unittest.main()
