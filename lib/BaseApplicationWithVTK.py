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

    def _create_vtk_cone(self, radius, height, resolution=32):
        cone = vtk.vtkConeSource()
        cone.SetRadius(radius)
        cone.SetHeight(height)
        cone.SetResolution(int(resolution))
        cone.SetDirection(0, 1, 0)  # Orientado hacia +Y
        cone.CappingOn()
        cone.Update()

        poly = cone.GetOutput()

        # Evitar UVs vacíos o corruptos (ya que algunos materiales no usan textura)
        if poly.GetPointData().GetTCoords():
            poly.GetPointData().SetTCoords(None)

        # Regenerar normales para tener normales validas
        normals = vtk.vtkPolyDataNormals()
        normals.SetInputData(poly)
        normals.ComputePointNormalsOn()
        normals.SplittingOff()
        normals.Update()

        # Convertir polydata a (P, N, T, C)
        return self._parametric_data_from_polydata(normals.GetOutput())
    
    def _create_tejo_model(self):
        import vtk

        radius = 0.085   # radio realista
        height = 0.018   # altura bajita (tejo plano)
        resolution = 64

        # Base: un cilindro
        cylinder = vtk.vtkCylinderSource()
        cylinder.SetRadius(radius)
        cylinder.SetHeight(height)
        cylinder.SetResolution(resolution)
        cylinder.CappingOn()
        cylinder.Update()

        cyl = cylinder.GetOutput()

        # Suavizado para bordes redondeados
        smooth = vtk.vtkSmoothPolyDataFilter()
        smooth.SetInputData(cyl)
        smooth.SetNumberOfIterations(30)
        smooth.SetRelaxationFactor(0.05)
        smooth.FeatureEdgeSmoothingOff()
        smooth.BoundarySmoothingOn()
        smooth.Update()

        smoothed = smooth.GetOutput()

        # Normales suaves
        normals = vtk.vtkPolyDataNormals()
        normals.SetInputData(smoothed)
        normals.ComputePointNormalsOn()
        normals.ComputeCellNormalsOff()
        normals.SplittingOff()
        normals.Update()

        poly = normals.GetOutput()

        # UV mapping cilíndrico (evita seams)
        tex = vtk.vtkTextureMapToCylinder()
        tex.SetInputData(poly)
        tex.PreventSeamOn()
        tex.Update()

        uv_mapped = tex.GetOutput()

        # Convertimos a P, N, T, C
        return self._parametric_data_from_polydata(uv_mapped)
