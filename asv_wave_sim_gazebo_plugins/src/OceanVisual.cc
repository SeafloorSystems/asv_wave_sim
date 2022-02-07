#include "asv_wave_sim_gazebo_plugins/OceanVisual.hh"
#include "asv_wave_sim_gazebo_plugins/OceanTile.hh"

#include "asv_wave_sim_gazebo_plugins/Gazebo.hh"

#include <ignition/common.hh>
#include <ignition/common/MeshManager.hh>
// #include <ignition/rendering/ogre_gazebo.h>
#include <ignition/rendering.hh>
// #include <ignition/rendering/RenderTypes.hh>
// #include <ignition/rendering/Visual.hh>

#include <cmath>
#include <memory>
#include <thread>
#include <vector>

using namespace gazebo;

namespace asv
{

///////////////////////////////////////////////////////////////////////////////
// OceanVisualPrivate

  class OceanVisualPrivate
  {
    /// \brief The visuals to attach
    public: rendering::VisualPtr aboveOceanVisual;
    public: rendering::VisualPtr belowOceanVisual;

    /// \brief The visual plugin SDF.
    public: sdf::ElementPtr sdf;

    /// \brief Generated mesh name (duplicated in tile->mesh...)
    public: std::string aboveOceanMeshName;
    public: std::string belowOceanMeshName;

    /// \brief World stats.
    public: double simTime, realTime, pauseTime;
    public: bool paused;

    /// \brief Ocean tile synchronises the mesh with the GPU
    public: std::unique_ptr<OceanTile> oceanTile;

    /// \brief Prevent Loading visual twice...
    public: bool isInitialised = false;

    /// \brief Mutex
    public: std::recursive_mutex mutex;

    /// \brief Event based connections.
    public: event::ConnectionPtr connection;

    /// \brief Node used to establish communication with gzserver.
    public: transport::NodePtr node;

    /// \brief Subscribe to gztopic "~/world_stats".
    public: transport::SubscriberPtr statsSub;
  };

///////////////////////////////////////////////////////////////////////////////
// OceanVisual

  OceanVisual::~OceanVisual()
  {
    this->data->connection.reset();
    this->data->statsSub.reset();
    this->data->node.reset();
  }

  OceanVisual::OceanVisual(
    const std::string &_name,
    rendering::VisualPtr _parent) :
    rendering::Visual(_name, _parent),
    data(new OceanVisualPrivate)
  {
    ignmsg << "Constructing OceanVisual..." << std::endl;

    rendering::Visual::SetType(VT_VISUAL);
  }

  void OceanVisual::Load(sdf::ElementPtr _sdf)
  {
    std::lock_guard<std::recursive_mutex> lock(this->data->mutex);

    ignmsg << "Loading OceanVisual..." << std::endl;
 
    GZ_ASSERT(_sdf != nullptr, "SDF Element is NULL");

    // Capture the sdf
    this->data->sdf = _sdf;

    this->Load();

    ignmsg << "Done loading OceanVisual." << std::endl;
  }

  void OceanVisual::Load()
  {
    std::lock_guard<std::recursive_mutex> lock(this->data->mutex);

    if (!this->data->isInitialised)
    {
      rendering::Visual::Load();

      // Transport
      this->data->node = transport::NodePtr(new transport::Node());
      this->data->node->Init();

      // Publishers

      // Subscribers
      this->data->statsSub = this->data->node->Subscribe(
        "~/world_stats", &OceanVisual::OnStatsMsg, this);

      // Bind the update method to ConnectPreRender events
      this->data->connection = event::Events::ConnectPreRender(
          std::bind(&OceanVisual::OnUpdate, this));

      // Shader visuals
      std::string aboveOceanVisualName  = this->Name() + "_ABOVE_OCEAN";
      std::string belowOceanVisualName  = this->Name() + "_BELOW_OCEAN";

      // @TODO: pass name to OceanTile where it is created independently but must match.
      this->data->aboveOceanMeshName = "AboveOceanTileMesh";
      this->data->belowOceanMeshName = "BelowOceanTileMesh";

      // @TODO Synchronise visual with physics...
      int N = 128;
      double L = 256.0;
      double u = 5.0;

      this->data->oceanTile.reset(new OceanTile(N, L));
      this->data->oceanTile->SetWindVelocity(u, 0.0);
      this->data->oceanTile->Create();
      this->data->oceanTile->Update(0.0);

      // Mesh Visual: Above
      {
        this->data->aboveOceanVisual.reset(new rendering::Visual(aboveOceanVisualName, shared_from_this()));
        this->data->aboveOceanVisual->Load();
        ignition::rendering::AttachMesh(*this->data->aboveOceanVisual, this->data->aboveOceanMeshName);
        this->data->aboveOceanVisual->SetPosition(this->Position());
        this->data->aboveOceanVisual->SetType(rendering::Visual::VT_VISUAL);

        // Set the material from the parent visual
        auto materialName = this->GetMaterialName();
        // std::string materialName("Gazebo/Green");
        this->data->aboveOceanVisual->SetMaterial(materialName);
      }

      // Mesh Visual: Below
      {
        this->data->belowOceanVisual.reset(new rendering::Visual(belowOceanVisualName, shared_from_this()));
        this->data->belowOceanVisual->Load();
        ignition::rendering::AttachMesh(*this->data->belowOceanVisual, this->data->belowOceanMeshName);
        this->data->belowOceanVisual->SetPosition(this->Position());
        this->data->belowOceanVisual->SetType(rendering::Visual::VT_VISUAL);

        // Set the material from the parent visual
        auto materialName = this->GetMaterialName();
        // std::string materialName("Gazebo/Orange");
        this->data->belowOceanVisual->SetMaterial(materialName);
      }

#if DEBUG
      ignmsg << "AboveOceanVisual..." << std::endl;
      ignmsg << "Name: "                 << this->data->aboveOceanVisual->Name() << std::endl;
      ignmsg << "Id: "                   << this->data->aboveOceanVisual->GetId() << std::endl;
      ignmsg << "MaterialName: "         << this->data->aboveOceanVisual->GetMaterialName() << std::endl;
      ignmsg << "MeshName: "             << this->data->aboveOceanVisual->GetMeshName() << std::endl;
      ignmsg << "ShaderType: "           << this->data->aboveOceanVisual->GetShaderType() << std::endl;
      ignmsg << "AttachedObjectCount: "  << this->data->aboveOceanVisual->GetAttachedObjectCount() << std::endl;

      ignmsg << "BelowOceanVisual..." << std::endl;
      ignmsg << "Name: "                 << this->data->belowOceanVisual->Name() << std::endl;
      ignmsg << "Id: "                   << this->data->belowOceanVisual->GetId() << std::endl;
      ignmsg << "MaterialName: "         << this->data->belowOceanVisual->GetMaterialName() << std::endl;
      ignmsg << "MeshName: "             << this->data->belowOceanVisual->GetMeshName() << std::endl;
      ignmsg << "ShaderType: "           << this->data->belowOceanVisual->GetShaderType() << std::endl;
      ignmsg << "AttachedObjectCount: "  << this->data->belowOceanVisual->GetAttachedObjectCount() << std::endl;
#endif

      this->SetVisibilityFlags(GZ_VISIBILITY_ALL);    
      this->data->isInitialised = true;
    }

  }

  /// Update the vertex buffer directly:
  /// http://wiki.ogre3d.org/RetrieveVertexData
  /// https://forums.ogre3d.org/viewtopic.php?t=68347
  /// https://forums.ogre3d.org/viewtopic.php?t=53882
  ///
  /// Notes: 
  /// 1. We must use a custom InsertMesh method as the vertex buffer needs to be created with
  ///   HardwareBuffer::Usage = Ogre::HardwareBuffer::HBU_DYNAMIC
  ///   useShadowBuffer = true
  ///
  /// vBuf = Ogre::HardwareBufferManager::getSingleton().createVertexBuffer(
  ///           vertexDecl->getVertexSize(0),
  ///           vertexData->vertexCount,
  ///           Ogre::HardwareBuffer::HBU_DYNAMIC,
  ///           true);
  ///
  void OceanVisual::OnUpdate()
  {
    std::lock_guard<std::recursive_mutex> lock(this->data->mutex);

    // ignmsg << "Updating OceanVisual..." << std::endl;

    if (this->data->paused)
      return;

    double time = this->data->simTime;
    // double time = std::fmod(this->data->simTime, _cycleTime);
    // ignmsg << "Time: " << time << std::endl;

    this->data->oceanTile->Update(time);

    // ignmsg << "Done updating OceanVisual." << std::endl;
  }

  void OceanVisual::OnStatsMsg(ConstWorldStatisticsPtr &_msg)
  {
    std::lock_guard<std::recursive_mutex> lock(this->data->mutex);

    this->data->simTime = ignition::msgs::Convert(_msg->sim_time()).Double();
    this->data->realTime = ignition::msgs::Convert(_msg->real_time()).Double();
    this->data->pauseTime = ignition::msgs::Convert(_msg->pause_time()).Double();
    this->data->paused = _msg->paused();
  }

} // namespace asv
