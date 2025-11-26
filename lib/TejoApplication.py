## -------------------------------------------------------------------------
## @author Sofía — Juego del Tejo (Ogre + VTK + PyBullet ligero)
## Basado en BaseApplicationWithVTK de Leonardo Florez-Valencia
## -------------------------------------------------------------------------

import os
import math
import pybullet
from .BaseApplicationWithVTK import BaseApplicationWithVTK
import Ogre.Bites as OgreBites
import Ogre
from .BaseApplication import BaseApplication
from .TejoListener import TejoListener

class TejoApplication(BaseApplicationWithVTK):

    def __init__(self):
        super().__init__('Tejo v0.1', '')
        self.m_ResourcesFile = os.path.join(os.path.dirname(__file__), '..', 'resources.cfg')

        # Parámetros de lanzamiento
        self.launch_pos = [-3.0, 1.0, 0.0]
        self.angle_vert = 40.0
        self.angle_horiz = 0.0
        self.power = 12.0
        self.g = 9.81

        # Estado del juego
        self.launching = False
        self.t = 0.0
        self.dt = 0.016
        self.score = 0
        self.tejo_node = None
        self.tejo_data = None               # Guardar geometría
        self.mecha_nodes = {}
        self.bocin_node = None
        self.mesa_node = None
        self.preview_node = None
        self.static_bodies = {}
        self.moving_body = None

        # Contador de frames para evitar crash
        self._frames = 0

        # Inicializar PyBullet
        pybullet.connect(pybullet.DIRECT)
        pybullet.setGravity(0, -self.g, 0)

    def _loadScene(self):
        self._create_mesa()
        self._create_mechas()
        self._create_bocin()
        self._create_tejo()

        self._createCamera(
            position=[6, 2, 8],
            look_at=[0, 0.7, 0],
            top_speed=5,
            cam_style=OgreBites.CS_FREELOOK
        )

        light = self.m_SceneMgr.createLight("MainLight")
        light.setType(Ogre.Light.LT_POINT)
        light.setDiffuseColour(0.9, 0.9, 0.8)
        light.setSpecularColour(0.5, 0.5, 0.4)
        light_node = self.m_SceneMgr.getRootSceneNode().createChildSceneNode()
        light_node.setPosition(Ogre.Vector3(-5, 5, 5))
        light_node.attachObject(light)

        self.m_Listener = TejoListener(self)
        self.addInputListener(self.m_Listener)

        self.update_tejo_position()

    # ----------------------------
    # OBJETOS DEL JUEGO
    # ----------------------------
    def _create_mesa(self):
        data = self._create_vtk_box(2.5, 0.04, 1.2)
        self.mesa_node = self._createManualObject(data, "Mesa", "field")
        self.mesa_node.setPosition(0, 0.6, 0)

        shape = pybullet.createCollisionShape(pybullet.GEOM_BOX, halfExtents=[1.25, 0.02, 0.6])
        body = pybullet.createMultiBody(baseMass=0.0, baseCollisionShapeIndex=shape, basePosition=[0, 0.6, 0])
        pybullet.changeDynamics(body, -1, restitution=0.5, lateralFriction=0.6)
        self.static_bodies['mesa'] = body

    def _create_mechas(self):
        positions = [
            [0.0, 0.6, 0.3],
            [0.0, 0.6, -0.3],
            [-0.3, 0.6, 0.0],
            [0.3, 0.6, 0.0]
        ]
        for i, pos in enumerate(positions):
            data = self._create_vtk_cone(0.08, 0.03, 1.3)
            node = self._createManualObject(data, f"mecha_{i}", "red_material")
            node.setPosition(*pos)
            node.setScale(0.8, 0.8, 0.8)

            shape = pybullet.createCollisionShape(pybullet.GEOM_SPHERE, radius=0.03)
            body = pybullet.createMultiBody(baseMass=0.0, baseCollisionShapeIndex=shape, basePosition=pos)
            pybullet.changeDynamics(body, -1, restitution=0.5, lateralFriction=0.6)
            self.static_bodies[f'mecha_{i}'] = body
            self.mecha_nodes[f'mecha_{i}'] = node

    def _create_bocin(self):
        data = self._create_vtk_cylinder(0.05, 0.01)
        self.bocin_node = self._createManualObject(data, "Bocin", "target_material")
        self.bocin_node.setPosition(0, 0.6, 0)

        shape = pybullet.createCollisionShape(pybullet.GEOM_BOX, halfExtents=[0.05, 0.005, 0.05])
        body = pybullet.createMultiBody(baseMass=0.0, baseCollisionShapeIndex=shape, basePosition=[0, 0.6, 0])
        pybullet.changeDynamics(body, -1, restitution=0.5, lateralFriction=0.6)
        self.static_bodies['bocin'] = body

    def _create_tejo(self):
        # Tejo (cono pequeño)
        data = self._create_vtk_cone(0.08, 0.03, 1.3)
        self.tejo_node = self._createManualObject(data, "Tejo", "tejo_metal")
        self.tejo_node.setScale(0.8, 0.8, 0.8)

    # ----------------------------
    # Actualización y lanzamiento
    # ----------------------------
    def update_tejo_position(self):
        if self.tejo_node:
            self.tejo_node.setPosition(*self.launch_pos)
            q = Ogre.Quaternion(Ogre.Radian(math.radians(-self.angle_horiz)), Ogre.Vector3(0, 1, 0))
            self.tejo_node.setOrientation(q)

    def launch_tejo(self):
        if self.launching or not self.tejo_node:
            return

        self.launching = True
        self.t = 0.0

        shape = pybullet.createCollisionShape(pybullet.GEOM_SPHERE, radius=0.085)
        mass = 0.017
        self.moving_body = pybullet.createMultiBody(baseMass=mass, baseCollisionShapeIndex=shape, basePosition=self.launch_pos)
        pybullet.changeDynamics(self.moving_body, -1, restitution=0.5, lateralFriction=0.6)

        av = math.radians(self.angle_vert)
        ah = math.radians(self.angle_horiz)
        vx = self.power * math.cos(av) * math.cos(ah)
        vy = self.power * math.sin(av)
        vz = self.power * math.cos(av) * math.sin(ah)
        pybullet.resetBaseVelocity(self.moving_body, linearVelocity=[vx, vy, vz])

        if self.preview_node:
            self.preview_node.setVisible(False)

    # ----------------------------
    # Frame update
    # ----------------------------
    def frameRenderingQueued(self, evt):
        self._frames += 1
        if self._frames < 2:
            return True  # saltar primer frame para Ogre

        if self.launching and self.moving_body:
            dt = evt.timeSinceLastFrame
            pybullet.stepSimulation()

            pos, orn = pybullet.getBasePositionAndOrientation(self.moving_body)
            self.tejo_node.setPosition(pos)
            self.tejo_node.setOrientation(Ogre.Quaternion(orn[3], orn[0], orn[1], orn[2]))

            contacts = []
            for name, body in self.static_bodies.items():
                contact_points = pybullet.getContactPoints(bodyA=self.moving_body, bodyB=body)
                if contact_points:
                    contacts.append(name)

            if 'mesa' in contacts or any(f'mecha_' in c for c in contacts) or 'bocin' in contacts:
                self._on_impact(pos, contacts)
                self.launching = False
                pybullet.removeBody(self.moving_body)
                self.moving_body = None
        else:
            self.update_tejo_position()

        return True

    # ----------------------------
    # Impactos y puntaje
    # ----------------------------
    def _on_impact(self, pos, contacts):
        x, y, z = pos
        points = 0

        mecha_hits = [c for c in contacts if 'mecha_' in c]
        if mecha_hits:
            points += 3

        if 'bocin' in contacts:
            points += 6

        if len(mecha_hits) >= 1 and 'bocin' in contacts:
            points = 9

        self.score += points
        print(f"Puntos: {points} | Total: {self.score}")
        self._update_score_display(f"Puntos: {self.score} (último: {points})")

        self.getRoot().addFrameListener(self._delayed_reset)

    def _delayed_reset(self, evt):
        self.launching = False
        if self.tejo_node:
            self.tejo_node.setPosition(*self.launch_pos)
        if self.preview_node:
            self.preview_node.setVisible(True)
        self.update_tejo_position()
        return False

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
        print(text)

# end class
