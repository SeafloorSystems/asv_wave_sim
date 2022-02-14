// Copyright (C) 2022  Rhys Mainwaring
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#include "WavesVisual.hh"

#include "Ogre2OceanVisual.hh"
#include "Ogre2OceanGeometry.hh"

#include "ignition/marine/OceanTile.hh"

#include <ignition/common/Profiler.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/rendering/Material.hh>
#include <ignition/rendering/RenderingIface.hh>
#include <ignition/rendering/Scene.hh>
#include <ignition/rendering/ShaderParams.hh>
#include <ignition/rendering/Visual.hh>

#include <ignition/rendering/Grid.hh>

#include <ignition/rendering/ogre2.hh>
#include <ignition/rendering/ogre2/Ogre2MeshFactory.hh>
#include <ignition/rendering/ogre2/Ogre2Scene.hh>

#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/SourceFilePath.hh>
#include <ignition/gazebo/rendering/Events.hh>
#include <ignition/gazebo/rendering/RenderUtil.hh>
#include <ignition/gazebo/Util.hh>

#include <sdf/Element.hh>

#include <chrono>
#include <list>
#include <mutex>
#include <vector>
#include <string>

namespace ignition
{
namespace rendering
{
inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {

  // Subclass from Ogre2Mesh and Ogre2MeshFactory to get
  // indirect access to protected members and override any
  // behaviour that tries to load a common::Mesh which we
  // are not using.

  //////////////////////////////////////////////////
  class IGNITION_RENDERING_OGRE2_VISIBLE Ogre2MeshExt :
      public Ogre2Mesh
  {
    /// \brief Destructor
    public: virtual ~Ogre2MeshExt() {}

    /// \brief Constructor
    protected: explicit Ogre2MeshExt() : Ogre2Mesh() {}

    /// \brief Allow intercept of pre-render call for this mesh
    public: virtual void PreRender() override
    {
      Ogre2Mesh::PreRender();
    }

    /// \brief Work-around the protected accessors and protected methods in Scene
    public: void InitObject(Ogre2ScenePtr _scene, unsigned int _id, const std::string &_name)
    {
      this->id = _id;
      this->name = _name;
      this->scene = _scene;

      // initialize object
      this->Load();
      this->Init();
    }

    /// \brief Used by friend class (Ogre2MeshFactoryExt)
    protected: void SetOgreItem(Ogre::Item *_ogreItem)
    {
      this->ogreItem = _ogreItem;
    }

    private: friend class Ogre2MeshFactoryExt;
  };

  typedef std::shared_ptr<Ogre2MeshExt> Ogre2MeshExtPtr;

  //////////////////////////////////////////////////
  class IGNITION_RENDERING_OGRE2_VISIBLE Ogre2MeshFactoryExt :
      public Ogre2MeshFactory
  {
    /// \brief Destructor
    public: virtual ~Ogre2MeshFactoryExt() {}

    /// \brief Constructor - construct from an Ogre2ScenePtr
    public: explicit Ogre2MeshFactoryExt(Ogre2ScenePtr _scene) :
        Ogre2MeshFactory(_scene), ogre2Scene(_scene) {}

    /// \brief Override - use an extension of Ogre2Mesh
    public: virtual Ogre2MeshPtr Create(const MeshDescriptor &_desc) override
    {
      // create ogre entity
      Ogre2MeshExtPtr mesh(new Ogre2MeshExt);
      MeshDescriptor normDesc = _desc;
      // \todo do this? override MeshDescriptor behaviour as we're not using common::Mesh
      normDesc.Load();
      mesh->SetOgreItem(this->OgreItem(normDesc));

      // check if invalid mesh
      if (!mesh->ogreItem)
      {
        ignerr << "Failed to get Ogre item for [" << _desc.meshName << "]"
                << std::endl;
        return nullptr;
      }

      // create sub-mesh store
      Ogre2SubMeshStoreFactory subMeshFactory(this->scene, mesh->ogreItem);
      mesh->subMeshes = subMeshFactory.Create();
      for (unsigned int i = 0; i < mesh->subMeshes->Size(); i++)
      {
        Ogre2SubMeshPtr submesh =
            std::dynamic_pointer_cast<Ogre2SubMesh>(mesh->subMeshes->GetById(i));
        submesh->SetMeshName(this->MeshName(_desc));
      }
      return mesh;
    }

    /// \brief Override \todo: may be able to use base class implementation...
    protected: virtual Ogre::Item * OgreItem(const MeshDescriptor &_desc) override
    {
      if (!this->Load(_desc))
      {
        return nullptr;
      }

      std::string name = this->MeshName(_desc);
      ignmsg << "Get Ogre::SceneManager\n";
      Ogre::SceneManager *sceneManager = this->scene->OgreSceneManager();

      ignmsg << "Check for v2 mesh\n";
      // check if a v2 mesh already exists
      Ogre::MeshPtr mesh =
          Ogre::MeshManager::getSingleton().getByName(name);

      // if not, it probably has not been imported from v1 yet
      if (!mesh)
      {
        ignmsg << "Check for v1 mesh\n";
        Ogre::v1::MeshPtr v1Mesh =
            Ogre::v1::MeshManager::getSingleton().getByName(name);
        if (!v1Mesh)
        {
          ignerr << "Did not find v1 mesh [" << name << "]\n";
          return nullptr;
        }

        // examine v1 mesh properties
        v1Mesh->load();
        ignmsg << "v1 mesh: isLoaded: " << v1Mesh->isLoaded() << "\n";

        ignmsg << "Creating v2 mesh\n";
        // create v2 mesh from v1
        mesh = Ogre::MeshManager::getSingleton().createManual(
            name, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);

        ignmsg << "Importing v2 mesh\n";
        mesh->importV1(v1Mesh.get(), false, true, true);
        this->ogreMeshes.push_back(name);
      }

      return sceneManager->createItem(mesh, Ogre::SCENE_DYNAMIC);
    }

    /// \brief Override
    protected: virtual bool LoadImpl(const MeshDescriptor &_desc) override
    {
      /// \todo: currently assume we've already loaded from the tile
      /// but should handle better
      return true;
    }

    /// \brief Override \todo: may be able to use base class implementation...
    public: virtual bool Validate(const MeshDescriptor &_desc) override
    {
      if (!_desc.mesh && _desc.meshName.empty())
      {
        ignerr << "Invalid mesh-descriptor, no mesh specified" << std::endl;
        return false;
      }

      if (!_desc.mesh)
      {
        ignerr << "Cannot load null mesh [" << _desc.meshName << "]" << std::endl;
        return false;
        // Override MeshDescriptor behaviour as we're not using common::Mesh
        // return true;
      }

      if (_desc.mesh->SubMeshCount() == 0)
      {
        ignerr << "Cannot load mesh with zero sub-meshes" << std::endl;
        return false;
      }

      return true;
    }

    /// \brief Pointer to the derived Ogre2Scene
    private: rendering::Ogre2ScenePtr ogre2Scene;

  };

  typedef std::shared_ptr<Ogre2MeshFactoryExt> Ogre2MeshFactoryExtPtr;
}
}
}

using namespace ignition;
using namespace gazebo;
using namespace systems;

class ignition::gazebo::systems::WavesVisualPrivate
{
  ////////// BEGIN SHADER_PARAM

  /// \brief Data structure for storing shader param info
  public: class ShaderParamValue
  {
    /// \brief shader type: vertex or fragment
    public: std::string shader;

    /// \brief variable type: int, float, float_array, int_array,
    ///        texture, texture_cube
    /// \todo(anyone) support samplers
    public: std::string type;

    /// \brief variable name of param
    public: std::string name;

    /// \brief param value
    public: std::string value;
  };

  /// \brief Path to vertex shader
  public: std::string vertexShaderUri;

  /// \brief Path to fragment shader
  public: std::string fragmentShaderUri;

  /// \brief A list of shader params
  public: std::vector<ShaderParamValue> shaderParams;

  /// \brief Time params that will be updated every iteration
  public: std::vector<ShaderParamValue> timeParams;

  ////////// END SHADER_PARAM

  /// \brief Path to the model
  public: std::string modelPath;

  /// \brief All rendering operations must happen within this call
  public: void OnUpdate();

  /// \brief Name of visual this plugin is attached to
  public: std::string visualName;

  /// \brief Pointer to visual
  public: rendering::VisualPtr visual;

  /// \brief Pointer to ocean visual
  public: rendering::VisualPtr oceanVisual;
  // public: std::vector<rendering::Ogre2OceanVisualPtr> oceanVisuals;

  public: std::vector<rendering::VisualPtr> oceanVisuals;
  public: rendering::Ogre2OceanGeometryPtr oceanGeometry;

  /// \brief Material used by this visual
  public: rendering::MaterialPtr material;

  /// \brief Pointer to scene
  public: rendering::ScenePtr scene;

  /// \brief Entity id of the visual
  public: Entity entity = kNullEntity;


  /// \brief Entity id of the ocean visual
  public: Entity oceanEntity = kNullEntity;

  /// \brief Current sim time
  public: std::chrono::steady_clock::duration currentSimTime;

  /////////////////
  /// OceanTile
  // std::string mAboveOceanMeshName = "AboveOceanTileMesh";
  // std::string mBelowOceanMeshName = "BelowOceanTileMesh";

  public: marine::visual::OceanTilePtr oceanTile;

  /// \brief Used in DynamicMesh example
  public: common::MeshPtr oceanTileMesh;

  /// \brief Mutex to protect sim time updates.
  public: std::mutex mutex;

  /// \brief Connection to pre-render event callback
  public: ignition::common::ConnectionPtr connection{nullptr};
};

/////////////////////////////////////////////////
WavesVisual::WavesVisual()
    : System(), dataPtr(std::make_unique<WavesVisualPrivate>())
{
}

/////////////////////////////////////////////////
WavesVisual::~WavesVisual()
{
}

/////////////////////////////////////////////////
void WavesVisual::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &_eventMgr)
{
  IGN_PROFILE("WavesVisual::Configure");

  ignmsg << "WavesVisual: configuring\n";

  // Ugly, but needed because the sdf::Element::GetElement is not a const
  // function and _sdf is a const shared pointer to a const sdf::Element.
  auto sdf = const_cast<sdf::Element *>(_sdf.get());

  ////////// BEGIN SHADER_PARAM

  if (sdf->HasElement("param"))
  {
    // loop and parse all shader params
    sdf::ElementPtr paramElem = sdf->GetElement("param");
    while (paramElem)
    {
      if (!paramElem->HasElement("shader") ||
          !paramElem->HasElement("name"))
      {
        ignerr << "<param> must have <shader> and <name> sdf elements"
               << std::endl;
        paramElem = paramElem->GetNextElement("param");
        continue;
      }
      std::string shaderType = paramElem->Get<std::string>("shader");
      std::string paramName = paramElem->Get<std::string>("name");

      std::string type = paramElem->Get<std::string>("type", "float").first;
      std::string value = paramElem->Get<std::string>("value", "").first;

      WavesPrivate::ShaderParamValue spv;
      spv.shader = shaderType;
      spv.name = paramName;
      spv.value = value;
      spv.type = type;
      this->dataPtr->shaderParams.push_back(spv);
      paramElem = paramElem->GetNextElement("param");
    }
  }

  // parse path to shaders
  if (sdf->HasElement("shader"))
  {
    sdf::ElementPtr shaderElem = sdf->GetElement("shader");
    if (!shaderElem->HasElement("vertex") ||
        !shaderElem->HasElement("fragment"))
    {
      ignerr << "<shader> must have <vertex> and <fragment> sdf elements"
             << std::endl;
    }
    else
    {
      auto modelEntity = topLevelModel(_entity, _ecm);
      this->dataPtr->modelPath =
          _ecm.ComponentData<components::SourceFilePath>(modelEntity).value();
      sdf::ElementPtr vertexElem = shaderElem->GetElement("vertex");
      this->dataPtr->vertexShaderUri = common::findFile(
          asFullPath(vertexElem->Get<std::string>(),
              this->dataPtr->modelPath));
      sdf::ElementPtr fragmentElem = shaderElem->GetElement("fragment");
      this->dataPtr->fragmentShaderUri = common::findFile(
          asFullPath(fragmentElem->Get<std::string>(),
              this->dataPtr->modelPath));
    }
  }

  ////////// END SHADER_PARAM

  // capture entity 
  this->dataPtr->entity = _entity;
  auto nameComp = _ecm.Component<components::Name>(_entity);
  this->dataPtr->visualName = nameComp->Data();

  // create new entity for the ocean visual
  this->dataPtr->oceanEntity = _ecm.CreateEntity();

  // connect to the SceneUpdate event
  // the callback is executed in the rendering thread so do all
  // rendering operations in that thread
  this->dataPtr->connection =
      _eventMgr.Connect<ignition::gazebo::events::SceneUpdate>(
      std::bind(&WavesVisualPrivate::OnUpdate, this->dataPtr.get()));
}

//////////////////////////////////////////////////
void WavesVisual::PreUpdate(
  const UpdateInfo &_info,
  EntityComponentManager &)
{
  IGN_PROFILE("WavesVisual::PreUpdate");
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  this->dataPtr->currentSimTime = _info.simTime;
}

//////////////////////////////////////////////////
WavesVisualPrivate::~WavesVisualPrivate()
{
  if (this->oceanVisual != nullptr)
  {
    this->oceanVisual->Destroy();
    this->oceanVisual.reset();
  }

  for (auto& ogreVisual : this->oceanVisuals)
  {
    rendering::VisualPtr visual = ogreVisual;
    if (visual != nullptr)
    {
      visual->Destroy();
      visual.reset();
    }
  }
};

//////////////////////////////////////////////////
enum class OceanVisualMethod : uint16_t
{
  /// \internal
  /// \brief Indicator used to create an iterator over the
  /// enum. Do not use this.
  OCEAN_VISUAL_METHOD_BEGIN = 0,

  /// \brief Unknown graphics interface
  UNKNOWN = OCEAN_VISUAL_METHOD_BEGIN,

  /// \brief Ogre::Mesh (v2)
  OGRE2_MESH = 2,

  /// \brief Ogre2DynamicGeometry
  OGRE2_DYNAMIC_GEOMETRY = 3,

  /// \brief Ogre2DynamicGeometry
  PLANE = 4,

  /// \internal
  /// \brief Indicator used to create an iterator over the
  /// enum. Do not use this.
  OCEAN_VISUAL_METHOD_END
};

void WavesVisualPrivate::OnUpdate()
{
  std::lock_guard<std::mutex> lock(this->mutex);
  if (this->visualName.empty())
    return;

  if (!this->scene)
  {
    ignmsg << "WavesVisual: retrieving scene from render engine\n";
    this->scene = rendering::sceneFromFirstRenderEngine();
  }

  if (!this->scene)
    return;

  if (!this->visual)
  {
    ignmsg << "WavesVisual: searching for visual\n";

    // this does a breadth first search for visual with the entity id
    // \todo(anyone) provide a helper function in RenderUtil to search for
    // visual by entity id?
    auto rootVis = scene->RootVisual();
    std::list<rendering::NodePtr> nodes;
    nodes.push_back(rootVis);
    while (!nodes.empty())
    {
      auto n = nodes.front();
      nodes.pop_front();
      if (n && n->HasUserData("gazebo-entity"))
      {
        // RenderUtil stores gazebo-entity user data as int
        // \todo(anyone) Change this to uint64_t in Ignition H?
        auto variant = n->UserData("gazebo-entity");
        const int *value = std::get_if<int>(&variant);
        if (value && *value == static_cast<int>(this->entity))
        {
          this->visual = std::dynamic_pointer_cast<rendering::Visual>(n);
          ignmsg << "Waves: found visual [" << this->visual->Name() << "]\n";
          break;
        }
      }
      for (unsigned int i = 0; i < n->ChildCount(); ++i)
        nodes.push_back(n->ChildByIndex(i));
    }
  }

  if (!this->visual)
    return;

  if (!this->scene->MaterialRegistered("OceanBlue"))
  {
    ignmsg << "WavesVisual: creating material `OceanBlue`\n";

    auto mat = this->scene->CreateMaterial("OceanBlue");
    mat->SetAmbient(0.0, 0.0, 0.3);
    mat->SetDiffuse(0.0, 0.0, 0.8);
    mat->SetSpecular(0.8, 0.8, 0.8);
    mat->SetShininess(50);
    mat->SetReflectivity(0);
  }

  double simTime = (std::chrono::duration_cast<std::chrono::nanoseconds>(
      this->currentSimTime).count()) * 1e-9;

  OceanVisualMethod method = OceanVisualMethod::OGRE2_MESH;
  switch (method)
  {
    case OceanVisualMethod::OGRE2_DYNAMIC_GEOMETRY:
    {
      ignmsg << "Waves: creating dynamic geometry ocean visual\n";

      // create ocean tile
      int N = 128;
      double L = 256.0;
      double u = 5.0;

      this->oceanTile.reset(new rendering::OceanTile(N, L));
      this->oceanTile->SetWindVelocity(u, 0.0);

      // do not pass to common::MeshManager as we are going to modify
      this->oceanTileMesh.reset(this->oceanTile->CreateMesh());

      // create visual
      rendering::Ogre2OceanVisualPtr ogre2Visual =
          std::make_shared<rendering::Ogre2OceanVisual>(); 

      // Scene: initialisation work-around
      rendering::Ogre2ScenePtr ogre2Scene =
          std::dynamic_pointer_cast<rendering::Ogre2Scene>(this->scene);
      ogre2Visual->InitObject(ogre2Scene, 50010, "ocean-visual");
      // visual->LoadCube();
      // ogre2Visual->LoadOceanTile(this->oceanTile);
      ogre2Visual->LoadMesh(this->oceanTileMesh);

      rendering::VisualPtr visual = ogre2Visual;
      visual->SetLocalPosition(0.0, 0.0, 0.0);
      visual->SetLocalRotation(0.0, 0.0, 0.0);
      visual->SetLocalScale(1.0, 1.0, 1.0);
      visual->SetMaterial("OceanBlue");
      

      // Required? These are used in ignition::gazebo::VisualizationCapabilities
      // ignition::gazebo::Entity entityId;
      // visual->SetUserData("gazebo-entity", static_cast<int>(entityId));
      // visual->SetUserData("pause-update", static_cast<int>(0));

      // add visual to parent
      // auto parent = this->visual->Parent();
      // parent->AddChild(visual);

      // keep reference
      this->oceanVisual = visual;
    }

    if (!this->oceanVisual)
      return;

    // Simple update (reload entire mesh...)
    // update the tile (recalculates vertices)
    this->oceanTile->UpdateMesh(simTime, this->oceanTileMesh.get());

    // update the dynamic renderable (CPU => GPU)
    rendering::Ogre2OceanVisualPtr ogre2Visual =
        std::dynamic_pointer_cast<rendering::Ogre2OceanVisual>(
            this->oceanVisual);
    // ogre2Visual->UpdateOceanTile(this->oceanTile);
    ogre2Visual->UpdateMesh(this->oceanTileMesh);

    break;
  }
  case OceanVisualMethod::OGRE1_MESH:
  {
    // Test attaching an Ogre2 mesh to the entity (import from Ogre::v1::Mesh)
    if (!this->ogre2OceanTile)
    {
      ignmsg << "Waves: creating Ogre::v1::Mesh ocean visual\n";

      // \todo(srmainwaring) synchronise visual with physics...
      int N = 128;
      double L = 256.0;
      double u = 5.0;

      // create static ocean tile using common::mesh - this to populate
      // an object in the MeshManager with the correct name
      this->oceanTile.reset(new rendering::OceanTile(N, L));
      this->oceanTile->SetWindVelocity(u, 0.0);
      std::unique_ptr<common::Mesh> newMesh(this->oceanTile->CreateMesh());
      auto mesh = newMesh.get();
      common::MeshManager::Instance()->AddMesh(newMesh.release());

      ignmsg << "Waves: mesh name " << mesh->Name() << "\n";
      ignmsg << "Waves: mesh resource path " << mesh->Path() << "\n";

      this->ogre2OceanTile.reset(new rendering::Ogre2OceanTile(N, L));
      this->ogre2OceanTile->SetWindVelocity(u, 0.0);
      this->ogre2OceanTile->Create();
      // this->ogre2OceanTile->Update(0.0);

      // ogre2 specific
      rendering::Ogre2ScenePtr ogre2Scene =
          std::dynamic_pointer_cast<rendering::Ogre2Scene>(this->scene);    

      auto meshFactory = rendering::Ogre2MeshFactoryExtPtr(
            new rendering::Ogre2MeshFactoryExt(ogre2Scene));

      rendering::MeshDescriptor meshDescriptor;
      meshDescriptor.mesh = mesh;
      meshDescriptor.meshName = "AboveOceanTileMesh"; // mesh->Name();

      /////////////////////////////////////////////
      // BEGIN_LAMDAS
      //
      // Override functions from Scene, BaseScene, 
      // and Ogre2Scene
      //
      /////////////////////////////////////////////

      // bool Ogre2Scene::InitObject(Ogre2ObjectPtr _object, unsigned int _id,
      //     const std::string &_name)
      auto Ogre2Scene_InitObject = [&](
          rendering::Ogre2ScenePtr _scene,
          rendering::Ogre2ObjectPtr _object,
          unsigned int _id,
          const std::string &_name) -> bool
      {
        ignmsg << "WavesVisual: creating dynamic geometry ocean visual\n";

        // retrive the material from the visual's geometry (it's not set on the visual)
        ignmsg << "WavesVisual: Visual Name:          " << this->visual->Name() << "\n";
        ignmsg << "WavesVisual: Visual GeometryCount: " << this->visual->GeometryCount() << "\n";
        auto visualGeometry = this->visual->GeometryByIndex(0);
        auto material = visualGeometry->Material();
        // auto material = this->visual->Material();
        if (!material)
          ignerr << "WavesVisual: invalid material\n";

        // create ocean tile
        this->oceanTile.reset(new marine::visual::OceanTile(N, L));
        this->oceanTile->SetWindVelocity(u, 0.0);

        // create mesh - do not store in MeshManager as it will be modified
        this->oceanTileMesh.reset(this->oceanTile->CreateMesh());

        // scene: use in object initialisation work-around
        rendering::Ogre2ScenePtr ogre2Scene =
            std::dynamic_pointer_cast<rendering::Ogre2Scene>(this->scene);

        // NOTE: this approach is not feasible - multiple copies of the mesh
        // rather than multiple visuals referencing one mesh and relocating it...

        // create geometry
        this->oceanGeometry =
            std::make_shared<rendering::Ogre2OceanGeometry>();

        unsigned int objId = 50000;
        std::stringstream ss;
        ss << "OceanGeometry(" << objId << ")";
        std::string objName = ss.str();

        this->oceanGeometry->InitObject(ogre2Scene, objId, objName);
        this->oceanGeometry->LoadMesh(this->oceanTileMesh);

        // Water tiles -nX, -nX + 1, ...,0, 1, ..., nX, etc.
        const int nX = 3;
        const int nY = 3;
        // unsigned int id = 50000;
        // for (int iy=-nY; iy<=nY; ++iy)
        int iy = 0;
        {
          // for (int ix=-nX; ix<=nX; ++ix)
          int ix = 0;
          {
            /// \todo: include the current entity position 
            ignition::math::Vector3d position(
              /* this->Position() + */ ix * L,
              /* this->Position() + */ iy * L,
              /* this->Position() + */ 0.0
            );

            // create visual
            // rendering::Ogre2OceanVisualPtr ogreVisual =
            //     std::make_shared<rendering::Ogre2OceanVisual>(); 

            // unsigned int objId = id++;
            // std::stringstream ss;
            // ss << "OceanVisual(" << objId << ")";
            // std::string objName = ss.str();

            // ogreVisual->InitObject(ogre2Scene, objId, objName);
            // ogreVisual->LoadMesh(this->oceanTileMesh);

            // rendering::VisualPtr visual = ogreVisual;
            // visual->SetLocalPosition(position);

            // if (!material)
            //   visual->SetMaterial("OceanBlue");
            // else
            //   visual->SetMaterial(material);

            // add visual to parent
            // auto parent = this->visual->Parent();
            // parent->AddChild(visual);

            // oceanVisuals.push_back(ogreVisual);

            auto visual = this->scene->CreateVisual();
            visual->AddGeometry(this->oceanGeometry);
            visual->SetLocalPosition(position);

            if (!material)
              visual->SetMaterial("OceanBlue");
            else
              visual->SetMaterial(material);

            oceanVisuals.push_back(visual);
          }
        }
      }

      if (this->oceanVisuals.empty())
        return;

      // update the tile (recalculates vertices)
      this->oceanTile->UpdateMesh(simTime, this->oceanTileMesh.get());

      // update the dynamic renderable (CPU => GPU)
      // for (auto& visual : this->oceanVisuals)
      // {
      //   visual->UpdateMesh(this->oceanTileMesh);
      // }
      this->oceanGeometry->UpdateMesh(this->oceanTileMesh);
      break;
    }
    case OceanVisualMethod::OGRE2_MESH:
    {
      // Test attaching a common::Mesh to the entity
      if (!this->oceanTile)
      {
        ignmsg << "WavesVisual: creating Ogre::Mesh ocean visual\n";

        // create ocean tile
        this->oceanTile.reset(new marine::visual::OceanTile(N, L));
        this->oceanTile->SetWindVelocity(u, 0.0);
        std::unique_ptr<common::Mesh> newMesh(this->oceanTile->CreateMesh());
        auto mesh = newMesh.get();
        common::MeshManager::Instance()->AddMesh(newMesh.release());
        // this->oceanTile->Update(0.0);

        //convert common::Mesh to rendering::Mesh
        auto geometry = this->scene->CreateMesh(mesh);

      //convert common::Mesh to rendering::Mesh
      auto geometry = this->scene->CreateMesh(mesh);
      geometry->SetMaterial("OceanBlue");

      // remove existing geometry
      auto visual = this->visual;
      if (visual->GeometryCount() > 0)
      {
        for (auto i=0; i<visual->GeometryCount(); ++i)
        {
          visual->RemoveGeometryByIndex(i);
        }
      }

      // create ocean tile visual
      // auto visual = this->scene->CreateVisual("ocean-tile");

      visual->AddGeometry(geometry);
      visual->SetLocalPosition(0.0, 0.0, 0.0);
      visual->SetLocalRotation(0.0, 0.0, 0.0);
      visual->SetLocalScale(1.0, 1.0, 1.0);

      // add visual to parent
      // auto parent = this->visual->Parent();
      // parent->AddChild(visual);

      if (!this->oceanTile)
        return;

      // Update the tile
      this->oceanTile->Update(simTime);
      break;
    }

    if (!this->oceanTile)
      return;

    // Update the tile
    this->oceanTile->Update(simTime);
    break;
  }
  case OceanVisualMethod::PLANE:
  {
    // Test attaching another visual to the entity
    if (!this->oceanVisual)
    {
      // Test attaching another visual to the entity
      if (!this->oceanVisual)
      {
        ignmsg << "WavesVisual: creating default ocean visual\n";

      // create plane
      auto geometry = this->scene->CreatePlane();
      geometry->SetMaterial("OceanBlue");

      // remove existing geometry
      auto visual = this->visual;
      if (visual->GeometryCount() > 0)
      {
        for (auto i=0; i<visual->GeometryCount(); ++i)
        {
          visual->RemoveGeometryByIndex(i);
        }
      }

      // create visual
      // auto visual = this->scene->CreateVisual("ocean-tile");

      visual->AddGeometry(geometry);
      visual->SetLocalPosition(0.0, 0.0, 0.0);
      visual->SetLocalRotation(0.0, 0.0, 0.0);
      visual->SetLocalScale(100.0, 100.0, 100.0);

      // See SceneManager.cc L196-198
      // visual->SetUserData("gazebo-entity", static_cast<int>(this->oceanEntity));
      // visual->SetUserData("pause-update", static_cast<int>(0));

      // visual->SetMaterial("OceanBlue");
      // visual->SetMaterial(this->material);

      // auto material = this->visual->Material();
      // if (material)
      // {
      //   ignmsg << "Waves: set material [" << material->Name() << "]\n";
      //   visual->SetMaterial(material);
      // }

      // keep reference
      this->oceanVisual = visual;
    }

    if (!this->oceanVisual)
      return;

    break;
  }
  default:
    break;
  }


  ////////// BEGIN SHADER_PARAM

  // get the material and set shaders
  if (!this->material)
  {
    auto mat = scene->CreateMaterial();
    mat->SetVertexShader(this->vertexShaderUri);
    mat->SetFragmentShader(this->fragmentShaderUri);
    this->visual->SetMaterial(mat);
    scene->DestroyMaterial(mat);
    this->material = this->visual->Material();
    ignmsg << "Waves: create material [" << this->material->Name() << "]\n";
  }

  if (!this->material)
    return;

  // set the shader params read from SDF
  // this is only done once
  for (const auto & spv : this->shaderParams)
  {
    std::vector<std::string> values = common::split(spv.value, " ");

    int intValue = 0;
    float floatValue = 0;
    std::vector<float> floatArrayValue;
    std::string stringValue = "";

    rendering::ShaderParam::ParamType paramType =
        rendering::ShaderParam::PARAM_NONE;

    rendering::ShaderParamsPtr params;
    if (spv.shader == "fragment")
    {
      params = this->material->FragmentShaderParams();
    }
    else if (spv.shader == "vertex")
    {
      params = this->material->VertexShaderParams();
    }

    // if no <value> is specified, this could be a constant
    if (values.empty())
    {
      // \todo handle args for constants
      (*params)[spv.name] = intValue;
    }
    // float / int
    else if (values.size() == 1u)
    {
      std::string str = values[0];

      // TIME is reserved keyword for sim time
      if (str == "TIME")
      {
        this->timeParams.push_back(spv);
        continue;
      }

      // if <type> is not empty, respect the specified type
      if (!spv.type.empty())
      {
        if (spv.type == "int")
        {
          intValue = std::stoi(str);
          paramType = rendering::ShaderParam::PARAM_INT;
        }
        else if (spv.type == "float")
        {
          floatValue = std::stof(str);
          paramType = rendering::ShaderParam::PARAM_FLOAT;
        }
        else if (spv.type == "texture")
        {
          stringValue = str;
          paramType = rendering::ShaderParam::PARAM_TEXTURE;
        }
        else if (spv.type == "texture_cube")
        {
          stringValue = str;
          paramType = rendering::ShaderParam::PARAM_TEXTURE_CUBE;
        }
        else
        {
          // \todo(anyone) support texture samplers
        }
      }
      // else do our best guess at what the type is
      else
      {
        std::string::size_type sz;
        int n = std::stoi(str, &sz);
        if ( sz == str.size())
        {
          intValue = n;
          paramType = rendering::ShaderParam::PARAM_INT;
        }
        else
        {
          floatValue = std::stof(str);
          paramType = rendering::ShaderParam::PARAM_FLOAT;
        }
      }
    }
    // arrays
    else
    {
      // int array
      if (!spv.type.empty() && spv.type == "int_array")
      {
        for (const auto &v : values)
          floatArrayValue.push_back(std::stoi(v));
        paramType = rendering::ShaderParam::PARAM_INT_BUFFER;
      }
      // treat everything else as float_array
      else
      {
        for (const auto &v : values)
          floatArrayValue.push_back(std::stof(v));
        paramType = rendering::ShaderParam::PARAM_FLOAT_BUFFER;
      }
    }

    // set the params
    if (paramType == rendering::ShaderParam::PARAM_INT)
    {
      (*params)[spv.name] = intValue;
    }
    else if (paramType == rendering::ShaderParam::PARAM_FLOAT)
    {
      (*params)[spv.name] = floatValue;
    }
    else if (paramType == rendering::ShaderParam::PARAM_TEXTURE)
    {
      std::string textureUri = common::findFile(
          asFullPath(stringValue, this->modelPath));

      // \todo(anyone) support selecting texture coordinate set
      (*params)[spv.name].SetTexture(textureUri,
          rendering::ShaderParam::ParamType::PARAM_TEXTURE, 0u);
    }
    else if (paramType == rendering::ShaderParam::PARAM_TEXTURE_CUBE)
    {
      std::string textureUri = common::findFile(
          asFullPath(stringValue, this->modelPath));

      // \todo(anyone) support selecting texture coordinate set
      (*params)[spv.name].SetTexture(textureUri,
          rendering::ShaderParam::ParamType::PARAM_TEXTURE_CUBE, 1u);
    }
    else if (paramType == rendering::ShaderParam::PARAM_INT_BUFFER ||
        paramType == rendering::ShaderParam::PARAM_FLOAT_BUFFER)
    {
      (*params)[spv.name].InitializeBuffer(floatArrayValue.size());
      float *fv = &floatArrayValue[0];
      (*params)[spv.name].UpdateBuffer(fv);
    }
  }
  this->shaderParams.clear();

  // time variables need to be updated every iteration
  for (const auto & spv : this->timeParams)
  {
    float floatValue = (std::chrono::duration_cast<std::chrono::nanoseconds>(
        this->currentSimTime).count()) * 1e-9;
    rendering::ShaderParamsPtr params;
    if (spv.shader == "fragment")
      params = this->material->FragmentShaderParams();
    else if (spv.shader == "vertex")
      params = this->material->VertexShaderParams();
    (*params)[spv.name] = floatValue;
  }

  ////////// END SHADER_PARAM

}

//////////////////////////////////////////////////
IGNITION_ADD_PLUGIN(WavesVisual,
                    ignition::gazebo::System,
                    WavesVisual::ISystemConfigure,
                    WavesVisual::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(WavesVisual,
  "ignition::gazebo::systems::WavesVisual")
