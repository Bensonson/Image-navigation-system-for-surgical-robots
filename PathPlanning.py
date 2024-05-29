import time
import vtk
import vtk.util.numpy_support
import numpy as np
from scipy.spatial import distance
from scipy.spatial.transform import Rotation as R
import slicer
from slicer.ScriptedLoadableModule import *
from concurrent.futures import ThreadPoolExecutor, as_completed


def convert_nodes_to_arrays(volume_node_name, markup_node_name):
    """This function converts a locations of vtkMRMLMarkupsFiducialNode into a
    numpy array. Its value is w.r.t the shape vtkMRMLLabelMapVolumeNode,
    i.e., within the label volume space."""
    if not isinstance(volume_node_name, str):
        raise TypeError('volume node name must be a string')
    if not isinstance(markup_node_name, str):
        raise TypeError('markup node name must be a string')

    # Get the vtkMRMLLabelMapVolumeNode and vtkMRMLMarkupsFiducialNode
    volume_node = slicer.util.getNode(volume_node_name)
    markup_node = slicer.util.getNode(markup_node_name)

    # Convert vtkMRMLLabelMapVolumeNode to numpy array
    volume_array = slicer.util.arrayFromVolume(volume_node)

    # Extract points from vtkMRMLMarkupsFiducialNode, convert them to IJK,
    # and store these in a numpy array
    number_of_markup_points = markup_node.GetNumberOfControlPoints()
    markup_points_array = np.zeros((number_of_markup_points, 3))

    ras_to_ijk_matrix = vtk.vtkMatrix4x4()
    volume_node.GetRASToIJKMatrix(ras_to_ijk_matrix)

    for i in range(number_of_markup_points):
        ras = [0, 0, 0]
        markup_node.GetNthControlPointPosition(i, ras)
        ras.append(1)  # Append an extra value to make it compatible for multiplication below
        ijk = ras_to_ijk_matrix.MultiplyFloatPoint(ras)
        markup_points_array[i] = ijk[:3]  # Discard the extra component

    return volume_array, markup_points_array


def nodes2arrays(vesselname, ventriclename, entriesname, targetsname):
    vessel, entries = convert_nodes_to_arrays(vesselname, entriesname)
    ventricle, targets = convert_nodes_to_arrays(ventriclename, targetsname)
    return vessel, ventricle, entries, targets


def scale_arrays(volume1, volume2, markup1, markup2, rate):
    """markups will be rounded when calculating the path,
    so I upsample the space in order to improve the fidelity of markup locations."""
    if not isinstance(rate, int):
        raise TypeError('Scaling rate must be an integer.')

    volume1 = np.kron(volume1, np.ones((rate, rate, rate)))
    volume2 = np.kron(volume2, np.ones((rate, rate, rate)))

    # Entries and targets will also be rounded here
    markup1 = markup1 * rate
    markup1 = markup1.astype(int)
    markup2 = markup2 * rate
    markup2 = markup2.astype(int)
    return volume1, volume2, markup1, markup2


def removeduplicates(ListOfPoints):
    return np.unique(ListOfPoints, axis=0)


"""Bresenham's algorithm is copy and modified from
https://www.geeksforgeeks.org/bresenhams-algorithm-for-3-d-line-drawing"""
def Bresenham3D(entry, target):
    x1, y1, z1 = entry
    x2, y2, z2 = target
    ListOfPoints = []
    ListOfPoints.append((x1, y1, z1))
    dx = abs(x2 - x1)
    dy = abs(y2 - y1)
    dz = abs(z2 - z1)
    if x2 > x1:
        xs = 1
    else:
        xs = -1
    if y2 > y1:
        ys = 1
    else:
        ys = -1
    if z2 > z1:
        zs = 1
    else:
        zs = -1

    # Driving axis is X-axis
    if dx >= dy and dx >= dz:
        p1 = 2 * dy - dx
        p2 = 2 * dz - dx
        while x1 != x2:
            x1 += xs
            if p1 >= 0:
                y1 += ys
                p1 -= 2 * dx
            if p2 >= 0:
                z1 += zs
                p2 -= 2 * dx
            p1 += 2 * dy
            p2 += 2 * dz
            ListOfPoints.append((x1, y1, z1))

    # Driving axis is Y-axis
    elif dy >= dx and dy >= dz:
        p1 = 2 * dx - dy
        p2 = 2 * dz - dy
        while y1 != y2:
            y1 += ys
            if p1 >= 0:
                x1 += xs
                p1 -= 2 * dy
            if p2 >= 0:
                z1 += zs
                p2 -= 2 * dy
            p1 += 2 * dx
            p2 += 2 * dz
            ListOfPoints.append((x1, y1, z1))

    # Driving axis is Z-axis
    else:
        p1 = 2 * dy - dz
        p2 = 2 * dx - dz
        while z1 != z2:
            z1 += zs
            if p1 >= 0:
                y1 += ys
                p1 -= 2 * dz
            if p2 >= 0:
                x1 += xs
                p2 -= 2 * dz
            p1 += 2 * dy
            p2 += 2 * dx
            ListOfPoints.append((x1, y1, z1))

    ListOfPoints = np.array(ListOfPoints)

    return ListOfPoints


def LineLength(start_point, end_point):
    if start_point.shape == (3,) and end_point.shape == (3,):
        x1, y1, z1 = start_point
        x2, y2, z2 = end_point
        d = np.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2 + (z2 - z1) ** 2)
        return d
    else:
        raise TypeError('start_point and end_point should be NumPy array of (3,) shape.')


def array2node(path, volume_node_name, nodename):
    """Convert the numpy array (markup_points_array) back into a vtkMRMLMarkupsFiducialNode"""

    # Get vtkMRMLVolumeNode
    volume_node = slicer.util.getNode(volume_node_name)

    # Create new vtkMRMLMarkupsFiducialNode
    new_fiducial_node = slicer.vtkMRMLMarkupsFiducialNode()
    new_fiducial_node.SetName(nodename)

    # Get IJK to RAS matrix from volume node
    ijk_to_ras_matrix = vtk.vtkMatrix4x4()
    volume_node.GetIJKToRASMatrix(ijk_to_ras_matrix)

    # Add points from the numpy array to new_fiducial_node
    for i in range(path.shape[0]):
        ijk = list(path[i])
        ijk.append(1)  # Add the homogeneous coordinate
        ras = ijk_to_ras_matrix.MultiplyFloatPoint(ijk)
        new_fiducial_node.AddControlPoint(ras[:3])

    # Add the new node to the scene
    slicer.mrmlScene.AddNode(new_fiducial_node)

    return new_fiducial_node


def compute_best_path(entry, s_targets, critical, max_length):
    """This function find the best path between an entry and targets"""
    max_d = 0
    best_path = None
    for target in s_targets:
        length = LineLength(entry, target)
        if length > max_length:
            continue
        path = Bresenham3D(entry, target)
        d = np.min(distance.cdist(path, critical))
        if d > max_d:
            best_path = path
            max_d = d
    return best_path, max_d


def findpath(entriesname, targetsname, vesselname, ventriclename, max_length=np.inf, scale_rate=1):
    """This is the one-for-all function that you put into the 3D Slicer console."""
    # Timing for performance validation
    start_time = time.time()
    # Convert vtknodes to nparrs
    vessel, ventricle, entries, targets = nodes2arrays(vesselname, ventriclename, entriesname, targetsname)

    # Scale the nodes
    s_vessel, s_ventricle, s_entries, s_targets = scale_arrays(vessel, ventricle, entries, targets, scale_rate)

    """When scale_rate = 1, two entry nodes will go into a same location after rounding.
    This function removes the duplicates. This problem does not exist in target nodes, and when scale_rate > 1"""
    if scale_rate == 1:
        s_entries = removeduplicates(s_entries)

    # Retrieve the coordinates of critical structures
    s_vessel = np.argwhere(s_vessel)
    s_ventricle = np.argwhere(s_ventricle)
    critical = np.vstack((s_vessel, s_ventricle))

    # Find the best path
    paths = []

    # Different entries find their best path in different CPU thread
    with ThreadPoolExecutor() as executor:
        futures = {executor.submit(compute_best_path, entry, s_targets, critical, max_length): entry for entry in
                   s_entries}

        for future in as_completed(futures):
            best_path, max_d = future.result()
            paths.append((best_path, max_d))

    # Get the path that has the max distance
    best_path, max_d = max(paths, key=lambda x: x[1])
    if max_d == 0:
        print('No achievable path. Try increasing the allowed length.')
        end_time = time.time()
        total_time = end_time - start_time
        print(f"The calculation takes {int(total_time / 60)} minutes")
        return None
    best_path = best_path / scale_rate

    array2node(best_path[0, :].reshape(1, 3), vesselname, 'entry')
    array2node(best_path[-1, :].reshape(1, 3), vesselname, 'target')

    end_time = time.time()
    total_time = end_time - start_time
    print(f"The best path is found. The calculation takes {int(total_time/60)} minutes")

    return best_path, max_d


'''This function calculate the rotation matrix from line AB to AC.
The method for calculating the angle is from:
https://dnmtechs.com/calculating-angles-between-n-dimensional-vectors-in-python-3/'''
def RotationMatrix(nparrA, nparrB, nparrC):

    if np.array_equal(nparrA, nparrB) or np.array_equal(nparrA, nparrC):
        raise ValueError('Point A should be different from B and C.')

    # Generate unit vectors
    AB = nparrB - nparrA
    AC = nparrC - nparrA
    u = AB / np.linalg.norm(AB)
    v = AC / np.linalg.norm(AC)


    # Compute the rotation vector
    angle = np.arccos(np.dot(u, v))
    axis = np.cross(u, v)
    if np.allclose(axis, 0):
        if np.allclose(angle, np.pi):  # anti-parallel
            axis = np.array([u[2], u[0], -u[1]])
        else:  # parallel
            return np.identity(3)
    axis = axis / np.linalg.norm(axis)
    vector = axis * angle

    # Generate rotation matrix
    ro = R.from_rotvec(vector)
    rotation_matrix = ro.as_matrix()

    return rotation_matrix


def gettransform(entry_name, target_name, patient_position = [0, 0, 0]):
    if not isinstance(entry_name, str):
        raise TypeError('volume node name must be a string')
    if not isinstance(target_name, str):
        raise TypeError('markup node name must be a string')
    if not isinstance(patient_position, list):
        raise TypeError('patient position must be a list')
    if len(patient_position) != 3:
        raise ValueError('Patient position should be a list with 3')

    # Get the coordination of entry and target
    entry = slicer.util.getNode(entry_name)
    target = slicer.util.getNode(target_name)
    entryCoords = [0, 0, 0]
    targetCoords = [0, 0, 0]
    entry.GetNthControlPointPositionWorld(0, entryCoords)
    target.GetNthControlPointPositionWorld(0, targetCoords)
    new_entry = []
    new_target = []
    for i, j, k in zip(entryCoords, targetCoords, patient_position):
        new_entry.append(i + k)
        new_target.append(j + k)
    entryCoords = new_entry
    targetCoords = new_target

    # Generate the trajectory line node
    length = LineLength(np.array(entryCoords), np.array(targetCoords))
    lineNode = slicer.vtkMRMLMarkupsLineNode()
    slicer.mrmlScene.AddNode(lineNode)
    lineNode.AddControlPoint(vtk.vtkVector3d([0, 0, 0]))
    lineEndCoords = [length, 0, 0]
    lineNode.AddControlPoint(vtk.vtkVector3d(lineEndCoords))

    # Calculate the transformation matrix
    matrix = np.identity(4)
    matrix[:-1, -1] = entryCoords
    matrix[:-1, :-1] = RotationMatrix(np.array(entryCoords),
                                      np.array(entryCoords) + np.array(lineEndCoords),
                                      np.array(targetCoords))

    # Generate LinearTransformNode
    vtk_matrix = vtk.vtkMatrix4x4()
    for i in range(4):
        for j in range(4):
            vtk_matrix.SetElement(i, j, matrix[i, j])
    transformNode = slicer.vtkMRMLLinearTransformNode()
    transformNode.SetMatrixTransformToParent(vtk_matrix)
    slicer.mrmlScene.AddNode(transformNode)
    lineNode.SetAndObserveTransformNodeID(transformNode.GetID())

    return lineNode, transformNode



