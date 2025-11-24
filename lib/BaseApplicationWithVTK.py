## -------------------------------------------------------------------------
## @author Leonardo Florez-Valencia (florez-l@javeriana.edu.co)
## @modified by Sofía 
## -------------------------------------------------------------------------

from .BaseApplication import BaseApplication
import vtk
import Ogre

class BaseApplicationWithVTK(BaseApplication):

    def __init__(self, title, res_file):
        super().__init__(title, res_file)
        self.m_ResourcesFile = res_file

    # ---------------------------------------------------------
    # Convertir PolyData → (P, N, T, C)
    # ---------------------------------------------------------
    def _parametric_data_from_polydata(self, mesh):
        normals = mesh.GetPointData().GetNormals()
        textures = mesh.GetPointData().GetTCoords()

        P, N, T = [], [], []

        for i in range(mesh.GetNumberOfPoints()):
            P.append(mesh.GetPoint(i))
            N.append(normals.GetTuple(i) if normals else (0, 1, 0))
            T.append(textures.GetTuple(i) if textures else (0, 0))

        C = []
        cells = mesh.GetPolys()
        ids = vtk.vtkIdList()

        cells.InitTraversal()
        while cells.GetNextCell(ids):
            C.append([ids.GetId(j) for j in range(ids.GetNumberOfIds())])

        return (P, N, T, C)

    # ---------------------------------------------------------
    # Crear objeto manual de OGRE
    # ---------------------------------------------------------
    def _createManualObject(self, data, name, material):
        print(f"Creando manual object: {name}")

        P, N, T, C = data

        mo = self.m_SceneMgr.createManualObject(name)
        mo.begin(material, Ogre.RenderOperation.OT_TRIANGLE_LIST)

        for i in range(len(P)):
            mo.position(*P[i])
            mo.normal(*N[i])
            mo.textureCoord(*T[i])

        for face in C:
            if len(face) == 3:
                mo.triangle(face[0], face[1], face[2])
            elif len(face) == 4:
                mo.quad(face[0], face[1], face[2], face[3])

        mo.end()

        node = self.m_SceneMgr.getRootSceneNode().createChildSceneNode()
        node.attachObject(mo)

        print(f"Manual object terminado: {name}")
        return node

    # ---------------------------------------------------------
    # Métodos auxiliares VTK 
    # ---------------------------------------------------------
    def _create_vtk_cylinder(self, radius, height, resolution=32):
        src = vtk.vtkCylinderSource()
        src.SetRadius(radius)
        src.SetHeight(height)
        src.SetResolution(resolution)
        src.CappingOn()
        src.Update()
        tf = vtk.vtkTransform()
        tf.RotateX(90)
        f = vtk.vtkTransformPolyDataFilter()
        f.SetInputData(src.GetOutput())
        f.SetTransform(tf)
        f.Update()
        return self._parametric_data_from_polydata(f.GetOutput())

    def _create_vtk_box(self, sx, sy, sz):
        box = vtk.vtkCubeSource()
        box.SetXLength(sx)
        box.SetYLength(sy)
        box.SetZLength(sz)
        box.Update()
        return self._parametric_data_from_polydata(box.GetOutput())

    def _create_vtk_cone(self, height, radius, truncate_ratio=1.3):
        cone = vtk.vtkConeSource()
        cone.SetHeight(height)
        cone.SetRadius(radius)
        cone.SetResolution(50)
        cone.Update()

        # Rotar con transform
        transform = vtk.vtkTransform()
        transform.RotateX(-90)
        transformFilter = vtk.vtkTransformPolyDataFilter()
        transformFilter.SetInputConnection(cone.GetOutputPort())
        transformFilter.SetTransform(transform)
        transformFilter.Update()

        # Normales y texturas
        normal_filter = vtk.vtkPolyDataNormals()
        normal_filter.SetInputConnection(transformFilter.GetOutputPort())
        normal_filter.ComputePointNormalsOn()
        normal_filter.ComputeCellNormalsOff()
        normal_filter.ConsistencyOn()
        normal_filter.AutoOrientNormalsOn()
        normal_filter.SplittingOff()
        normal_filter.Update()

        texture_filter = vtk.vtkTextureMapToCylinder()
        texture_filter.SetInputData(normal_filter.GetOutput())
        texture_filter.Update()

        mesh = texture_filter.GetOutput()
        return self._parametric_data_from_polydata(mesh)

# end class