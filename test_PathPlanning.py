"""Annotate out all slicer and vtk modules in PathPlanning.py before running unit test."""

import unittest
import numpy as np
import PathPlanning as pp
from scipy.spatial.transform import Rotation as R


class TestPathPlanningMethods(unittest.TestCase):

    def test_scale_arrays(self):
        volume1 = np.array([[[1, 0],[0, 0]],
                            [[0, 0],[0, 1]]])
        volume2 = np.ones((2, 2, 2))
        markup1 = np.array([1, 1, 1])
        markup2 = np.array([114514, 1919, 810])
        rate = 2

        expected_volume1 = np.zeros((4, 4, 4))
        expected_volume1[:2, :2, :2] = 1
        expected_volume1[-2:, -2:, -2:] = 1
        expected_volume2 = np.ones((4, 4, 4))
        expected_markup1 = np.array([2, 2, 2])
        expected_markup2 = np.array([114514*rate, 1919*rate, 810*rate])

        result_volume1, result_volume2, result_markup1, result_markup2 = pp.scale_arrays(volume1, volume2, markup1, markup2,
                                                                                     rate)

        np.testing.assert_array_equal(result_volume1, expected_volume1)
        np.testing.assert_array_equal(result_volume2, expected_volume2)
        np.testing.assert_array_equal(result_markup1, expected_markup1)
        np.testing.assert_array_equal(result_markup2, expected_markup2)

    def test_Bresenham3D(self):
        entry = np.array([-1, 1, 1])
        target = np.array([5, 3, -1])
        ListOfPoints = pp.Bresenham3D(entry, target)
        np.testing.assert_array_equal(ListOfPoints, np.array([(-1, 1, 1), (0, 1, 1), (1, 2, 0), (2, 2, 0), (3, 2, 0), (4, 3, -1), (5, 3, -1)]))

    def test_LineLength(self):
        entry = np.array([-1, 1, 1])
        target = np.array([5, 3, -1])
        length = pp.LineLength(entry, target)
        self.assertEqual(round(length, 2), 6.63)

    def test_removeduplicates_positive(self):
        arr = np.array([[1, 2, 3], [4, 5, 6], [4, 5, 6]])
        out = pp.removeduplicates(arr)
        np.testing.assert_array_equal(out, np.array([[1, 2, 3], [4, 5, 6]]))

    def test_removeduplicates_negative(self):
        arr = np.array([[1, 2, 3], [4, 5, 6], [7, 8, 9]])
        out = pp.removeduplicates(arr)
        np.testing.assert_array_equal(out, np.array([[1, 2, 3], [4, 5, 6], [7, 8, 9]]))

    def test_RotationMatrix(self):
        nparrA, nparrB, nparrC = np.array([0, 0, 0]), np.array([1, 0, 0]), np.array([0, 1, 0])
        expected_ro = R.from_rotvec(np.array([0, 0, np.pi / 2]))
        expected_rotation_matrix = expected_ro.as_matrix()
        np.testing.assert_array_equal(pp.RotationMatrix(nparrA, nparrB, nparrC), expected_rotation_matrix)

    def test_RotationMatrix_Identity(self):
        nparrA, nparrB, nparrC = np.array([0, 0, 0]), np.array([1, 0, 0]), np.array([1, 0, 0])
        expected_rotation_matrix = np.eye(3)
        np.testing.assert_array_equal(pp.RotationMatrix(nparrA, nparrB, nparrC), expected_rotation_matrix)

    def test_distinct_points(self):
        # Points A, B, C are distinct
        A = np.array([1, 0, 0])
        B = np.array([0, 1, 0])
        C = np.array([0, -1, 0])
        result = pp.RotationMatrix(A, B, C)
        result = result.astype(np.int8)  # floating points from calculation will cause false error
        np.testing.assert_array_equal(result, np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]]))

    def test_parallel(self):
        # Vectors are parallel and unit vectors. So, the result should be an Identity matrix
        A = np.array([0, 0, 0])
        B = np.array([1, 0, 0])
        C = np.array([2, 0, 0])
        result = pp.RotationMatrix(A, B, C)
        result = result.astype(np.int8)
        np.testing.assert_array_equal(result, np.identity(3))

    def test_antiparallel_vectors(self):
        # Vectors are anti-parallel.
        A = np.array([0, 0, 0])
        B = np.array([1, 0, 0])
        C = np.array([-1, 0, 0])
        result = pp.RotationMatrix(A, B, C)
        result = result.astype(np.int8)
        np.testing.assert_array_equal(result, np.array([[-1, 0, 0], [0, 1, 0], [0, 0, -1]]))

    def test_equal_points_AB(self):
        # Points A, B are same and it should throw ValueError
        A = np.array([1, 2, 3])
        B = np.array([1, 2, 3])
        C = np.array([4, 5, 6])
        with self.assertRaises(ValueError):
            pp.RotationMatrix(A, B, C)

    def test_equal_points_AC(self):
        # Points A, C are same it should throw a ValueError
        A = np.array([1, 2, 3])
        B = np.array([4, 5, 6])
        C = np.array([1, 2, 3])
        with self.assertRaises(ValueError):
            pp.RotationMatrix(A, B, C)

if __name__ == '__main__':
    unittest.main()
