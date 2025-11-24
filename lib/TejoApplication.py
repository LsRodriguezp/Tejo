## -------------------------------------------------------------------------
## @author Sofía — Juego del Tejo (Ogre + VTK + PyBullet ligero)
## Basado en BaseApplicationWithVTK de Leonardo Florez-Valencia
## -------------------------------------------------------------------------

import os
import sys
import pybullet
import math
from .BaseApplicationWithVTK import BaseApplicationWithVTK
from .TejoListener import TejoListener

class TejoApplication(BaseApplicationWithVTK):

    def __init__(self):
        super().__init__('Tejo v0.1', '')
        self.m_ResourcesFile = os.path.join(os.path.dirname(__file__), '..', 'resources.cfg')

        # Parámetros de lanzamiento
        self.launch_pos = [-3.0, 1.0, 0.0]  # Posición inicial del tejo
        self.angle_vert = 40.0             # Ángulo vertical (grados)
        self.angle_horiz = 0.0              # Ángulo horizontal (grados)
        self.power = 12.0                   # Fuerza del lanzamiento
        self.g = 9.81                       # Gravedad

        # Estado del juego
        self.launching = False
        self.t = 0.0
        self.dt = 0.016
        self.score = 0
        self.tejo_node = None
        self.mecha_nodes = {}               # Diccionario de mechas
        self.bocin_node = None
        self.mesa_node = None
        self.preview_node = None
        self.static_bodies = {}             # Cuerpos estáticos de PyBullet
        self.moving_body = None             # Cuerpo dinámico del tejo

        # Inicializar PyBullet (sin GUI)
        pybullet.connect(pybullet.DIRECT)  # Modo headless
        pybullet.setGravity(0, -self.g, 0)

    def _loadScene(self):
        # Crear geometría
        self._create_mesa()
        self._create_mechas()  # 4 mechas en cruz
        self._create_bocin()
        self._create_tejo()

        # Cámara
        self._createCamera(
            position=[6, 2, 8],
            look_at=[0, 0.7, 0],
            top_speed=5,
            cam_style=OgreBites.CS_FREELOOK
        )

        # Luz
        light = self.m_SceneMgr.createLight("MainLight")
        light.setType(Ogre.Light.LT_POINT)
        light.setDiffuseColour(0.9, 0.9, 0.8)
        light.setSpecularColour(0.5, 0.5, 0.4)
        light_node = self.m_SceneMgr.getRootSceneNode().createChildSceneNode()
        light_node.setPosition(Ogre.Vector3(-5, 5, 5))
        light_node.attachObject(light)

        # Input listener
        self.m_Listener = TejoListener(self)
        self.addInputListener(self.m_Listener)

        # Iniciar simulación
        self.update_tejo_position()

    def _create_mesa(self):
        # Mesa (campo de juego)
        data = self._create_vtk_box(2.5, 0.04, 1.2)
        self.mesa_node = self._createManualObject(data, "Mesa", "field")
        self.mesa_node.setPosition(0, 0.6, 0)

        # Collider de la mesa en PyBullet
        shape = pybullet.createCollisionShape(pybullet.GEOM_BOX, halfExtents=[1.25, 0.02, 0.6])
        body = pybullet.createMultiBody(
            baseMass=0.0,
            baseCollisionShapeIndex=shape,
            basePosition=[0, 0.6, 0]
        )
        pybullet.changeDynamics(body, -1, restitution=0.5, lateralFriction=0.6)
        self.static_bodies['mesa'] = body

    def _create_mechas(self):
        # Posiciones de las mechas (en cruz)
        positions = [
            [0.0, 0.6, 0.3],   # arriba
            [0.0, 0.6, -0.3],  # abajo
            [-0.3, 0.6, 0.0],  # izquierda
            [0.3, 0.6, 0.0]    # derecha
        ]
        for i, pos in enumerate(positions):
            # Geometría de la mecha (cono pequeño)
            data = self._create_vtk_cone(0.08, 0.03, 1.3)
            node = self._createManualObject(data, f"mecha_{i}", "red_material")  # O usa "BaseRed"
            node.setPosition(*pos)
            node.setScale(0.8, 0.8, 0.8)

            # Collider esférico para la mecha
            shape = pybullet.createCollisionShape(pybullet.GEOM_SPHERE, radius=0.03)
            body = pybullet.createMultiBody(
                baseMass=0.0,
                baseCollisionShapeIndex=shape,
                basePosition=pos
            )
            pybullet.changeDynamics(body, -1, restitution=0.5, lateralFriction=0.6)
            self.static_bodies[f'mecha_{i}'] = body
            self.mecha_nodes[f'mecha_{i}'] = node

    def _create_bocin(self):
        # Bocín (círculo en el centro)
        data = self._create_vtk_cylinder(0.05, 0.01)
        self.bocin_node = self._createManualObject(data, "Bocin", "target_material")  # O usa "BaseYellow"
        self.bocin_node.setPosition(0, 0.6, 0)

        # Collider cilíndrico para el bocín
        shape = pybullet.createCollisionShape(pybullet.GEOM_CYLINDER, radius=0.05, height=0.01)
        body = pybullet.createMultiBody(
            baseMass=0.0,
            baseCollisionShapeIndex=shape,
            basePosition=[0, 0.6, 0]
        )
        pybullet.changeDynamics(body, -1, restitution=0.5, lateralFriction=0.6)
        self.static_bodies['bocin'] = body

    def _create_tejo(self):
        # Tejo (cono pequeño)
        data = self._create_vtk_cone(0.08, 0.03, 1.3)
        self.tejo_node = self._createManualObject(data, "Tejo", "tejo_metal")
        self.tejo_node.setScale(0.8, 0.8, 0.8)

    def update_tejo_position(self):
        if self.tejo_node:
            self.tejo_node.setPosition(*self.launch_pos)
            # Rotación horizontal (opcional)
            q = Ogre.Quaternion(Ogre.Radian(math.radians(-self.angle_horiz)), Ogre.Vector3(0, 1, 0))
            self.tejo_node.setOrientation(q)

    def launch_tejo(self):
        if self.launching or not self.tejo_node:
            return

        self.launching = True
        self.t = 0.0

        # Crear cuerpo dinámico en PyBullet
        shape = pybullet.createCollisionShape(pybullet.GEOM_SPHERE, radius=0.03)
        mass = 0.017  # masa realista del tejo
        self.moving_body = pybullet.createMultiBody(
            baseMass=mass,
            baseCollisionShapeIndex=shape,
            basePosition=self.launch_pos
        )
        pybullet.changeDynamics(self.moving_body, -1, restitution=0.5, lateralFriction=0.6)

        # Calcular velocidad inicial
        av = math.radians(self.angle_vert)
        ah = math.radians(self.angle_horiz)
        vx = self.power * math.cos(av) * math.cos(ah)
        vy = self.power * math.sin(av)
        vz = self.power * math.cos(av) * math.sin(ah)
        pybullet.resetBaseVelocity(self.moving_body, linearVelocity=[vx, vy, vz])

        # Ocultar preview si existe
        if self.preview_node:
            self.preview_node.setVisible(False)

    def frameRenderingQueued(self, evt):
        if self.launching and self.moving_body:
            dt = evt.timeSinceLastFrame
            pybullet.stepSimulation()

            # Obtener posición y orientación del tejo
            pos, orn = pybullet.getBasePositionAndOrientation(self.moving_body)
            self.tejo_node.setPosition(pos)
            self.tejo_node.setOrientation(Ogre.Quaternion(orn[3], orn[0], orn[1], orn[2]))

            # Detectar colisiones
            contacts = []
            for name, body in self.static_bodies.items():
                contact_points = pybullet.getContactPoints(bodyA=self.moving_body, bodyB=body)
                if contact_points:
                    contacts.append(name)

            # Si hay contacto con la mesa o mechas, terminar lanzamiento
            if 'mesa' in contacts or any(f'mecha_' in c for c in contacts) or 'bocin' in contacts:
                self._on_impact(pos, contacts)
                self.launching = False
                pybullet.removeBody(self.moving_body)
                self.moving_body = None

        elif not self.launching:
            self.update_tejo_position()

        return True

    def _on_impact(self, pos, contacts):
        x, y, z = pos
        points = 0

        # Verificar colisión con mechas
        mecha_hits = [c for c in contacts if 'mecha_' in c]
        if mecha_hits:
            points += 3  # 3 puntos por cada mecha

        # Verificar colisión con bocín
        if 'bocin' in contacts:
            points += 6  # 6 puntos por bocín

        # Verificar si fue moniña (mecha + bocín)
        if len(mecha_hits) >= 1 and 'bocin' in contacts:
            points = 9  # Moniña

        self.score += points
        print(f"Puntos: {points} | Total: {self.score}")
        self._update_score_display(f"Puntos: {self.score} (último: {points})")

        # Resetear después de un breve delay
        self.getRoot().addFrameListener(self._delayed_reset)

    def _delayed_reset(self, evt):
        self.launching = False
        if self.tejo_node:
            self.tejo_node.setPosition(*self.launch_pos)
        if self.preview_node:
            self.preview_node.setVisible(True)
        self.update_tejo_position()
        return False  # Remove listener after one call

    def reset(self):
        self.launch_pos = [-3.0, 1.0, 0.0]
        self.angle_vert = 40.0
        self.angle_horiz = 0.0
        self.power = 12.0
        self.score = 0
        self._update_score_display("Puntos: 0")
        self.update_tejo_position()
        if self.moving_body:
            pybullet.removeBody(self.moving_body)
            self.moving_body = None

    def _update_score_display(self, text):
        #Solo imprime en consola
        print(text)

# end class