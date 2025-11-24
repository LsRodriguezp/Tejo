## -------------------------------------------------------------------------
## @author Leonardo Florez-Valencia (florez-l@javeriana.edu.co)
## @modified by Sofía — Juego del Tejo
## -------------------------------------------------------------------------

import os, sys, Ogre, Ogre.Bites as OgreBites

class BaseApplication(OgreBites.ApplicationContext):

    m_Listener = None
    m_ResourcesFile = None
    m_SceneMgr = None
    m_CamMan = None

    def __init__(self, title, res_file):
        super(BaseApplication, self).__init__(title)
        self.m_ResourcesFile = res_file

    def locateResources(self):
        pass

    def loadResources(self):
        parent_path = os.path.dirname(self.m_ResourcesFile)
        res_mgr = Ogre.ResourceGroupManager.getSingleton()
        cf = Ogre.ConfigFile()
        cf.loadDirect(self.m_ResourcesFile)
        settings = cf.getSettingsBySection()
        for k in settings:
            for i, j in settings[k].items():
                fname = j.replace('./', '')
                res_mgr.addResourceLocation(os.path.join(parent_path, fname), i, k)
        res_mgr.initialiseAllResourceGroups()
        for k in settings:
            if k != '':
                res_mgr.loadResourceGroup(k)

    def setup(self, mgr_type='DefaultSceneManager'):
        self._initSceneManager(mgr_type)
        self.setWindowGrab(True)
        self._loadScene()

    def keyPressed(self, evt):
        if evt.keysym.sym == OgreBites.SDLK_ESCAPE:
            self.getRoot().queueEndRendering()
        return True

    def go(self):
        self.initApp()
        self.getRoot().startRendering()
        self.closeApp()

    def _initSceneManager(self, mgr_type):
        super(BaseApplication, self).setup()
        if self.m_Listener is None:
            self.m_Listener = BaseListener(self.getRoot())
        self.addInputListener(self.m_Listener)
        root = self.getRoot()
        self.m_SceneMgr = root.createSceneManager(mgr_type)

    def _loadScene(self):
        pass

    def _ground(self, i, l):
        c = [(l[1] + l[0]) * 0.5, 0.0, (l[3] + l[2]) * 0.5]
        p = Ogre.Plane(0, 1, 0, 0)
        m = Ogre.MeshManager.getSingleton().createPlane(
            i, 'General', p,
            l[1] - l[0],
            l[3] - l[2],
            20, 20, True, 1, 5, 5, [0, 0, 1]
        )
        e = self.m_SceneMgr.createEntity(i, i)
        e.setMaterialName(i)
        n = self.m_SceneMgr.getRootSceneNode().createChildSceneNode()
        n.attachObject(e)
        n.setPosition(c)
        return n

    def _createCamera(
        self,
        clip_distance=5e-3,
        position=[0, 1, 0],
        look_at=[0, 0, 0],
        cam_type=Ogre.Node.TS_WORLD,
        cam_style=OgreBites.CS_ORBIT,
        top_speed=3,
        cam_name='MainCamera',
        background=[0, 0, 0]
    ):
        win = self.getRenderWindow()
        root = self.getRoot()
        root_node = self.m_SceneMgr.getRootSceneNode()
        cam = self.m_SceneMgr.createCamera(cam_name)
        cam.setNearClipDistance(clip_distance)
        cam.setAutoAspectRatio(True)
        camnode = root_node.createChildSceneNode()
        camnode.setPosition(position)
        camnode.lookAt(look_at, cam_type)
        camnode.attachObject(cam)
        self.m_CamMan = OgreBites.CameraMan(camnode)
        self.m_CamMan.setStyle(cam_style)
        self.m_CamMan.setTopSpeed(top_speed)
        self.m_CamMan.setFixedYaw(True)
        self.addInputListener(self.m_CamMan)
        vp = win.addViewport(cam)
        vp.setBackgroundColour(background)

# end class