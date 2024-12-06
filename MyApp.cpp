
#include "MyApp.hh"

#include <gmCore/FileResolver.hh>
#include <gmCore/TimeTools.hh>
#include <gmCore/Updateable.hh>

#include <gmTrack/ButtonsMapper.hh>

#include <gmNetwork/RunSync.hh>
#include <gmNetwork/DataSync.hh>
#include <gmNetwork/SyncSData.hh>
#include <gmNetwork/SyncMData.hh>

#include <gmGraphics/Group.hh>
#include <gmGraphics/MatrixTransform.hh>
#include <gmGraphics/PoseTransform.hh>
#include <gmGraphics/ObjRenderer.hh>

#include <gmGraphics/IntersectionVisitor.hh>

#include <filesystem>

#include <string>
#include <limits>

using namespace gramods;

typedef gmNetwork::SyncSData<Eigen::Vector3f> SyncSVec;
typedef gmNetwork::SyncSData<Eigen::Quaternionf> SyncSQuat;

/**
 * Declarations of the internal code of MyApp
 */
struct MyApp::Impl : public gmCore::Updateable {
  Impl(std::vector<std::shared_ptr<gmNetwork::SyncNode>> sync_nodes,
       std::vector<std::shared_ptr<gmTrack::Controller>> controllers,
       std::shared_ptr<gmTrack::SinglePoseTracker> head);

  void setup_sync(std::vector<std::shared_ptr<gmNetwork::SyncNode>> sync_nodes);

  void setup_wand(std::vector<std::shared_ptr<gmTrack::Controller>> controllers);

  //My own stuff

  void MyApp::Impl::loadObj(const std::string& filepath, const Eigen::Vector3f& pos, const Eigen::Vector3f& scale);

  // task6: check intersection and highlight closest

  gmGraphics::ObjRenderer* selectedObject = nullptr;
  gmGraphics::MatrixTransform* selectedObjTransformMat = nullptr;
  gmGraphics::ObjRenderer::Material originalMaterial;
  
  //task 9: grabbing
  bool grabbing = false;

  void highlightObj(gmGraphics::ObjRenderer& obj);

  void restoreOriginalMat(gmGraphics::ObjRenderer& obj);

  void checkIntersection(const Eigen::Vector3f& wandPos, const Eigen::Vector3f& headPos, const Eigen::Quaternionf& wandRot);

  void teleport(const Eigen::Vector3f& direction, const float jumpDist);

  //Task 9:
  Eigen::Matrix4f Mdelta = Eigen::Matrix4f::Identity(); 

  void grabObj();

  //Task 10:
  bool HOMERmode = true;

  /**
   * From gmCore::Updateable. Automatically called from main() via
   * gmCore::Updateable::updateAll();
   */
  void update(clock::time_point time, size_t frame) override;

  void update_data(clock::time_point time);

  void update_states(clock::time_point time);

  // Cluster synchronization handler
  std::shared_ptr<gmNetwork::SyncNode> sync_node;
  bool is_primary = true;

  // Wand (if this exists in the configuration)
  std::shared_ptr<gmTrack::Controller> wand;

  // Head (if this exists in the configuration)
  std::shared_ptr<gmTrack::SinglePoseTracker> head;

  /// ----- Containers for synchronized data -----

  // steady time
  std::shared_ptr<gmNetwork::SyncSFloat64> sync_time =
      std::make_shared<gmNetwork::SyncSFloat64>();

  // steady frame number
  std::shared_ptr<gmNetwork::SyncSUInt64> sync_frame_number =
      std::make_shared<gmNetwork::SyncSUInt64>(0);

  // wand analogs
  std::shared_ptr<gmNetwork::SyncMFloat32> sync_analogs =
      std::make_shared<gmNetwork::SyncMFloat32>();

  // wand main button
  std::shared_ptr<gmNetwork::SyncSBool> sync_main_button =
      std::make_shared<gmNetwork::SyncSBool>();

  // wand second button
  std::shared_ptr<gmNetwork::SyncSBool> sync_second_button =
      std::make_shared<gmNetwork::SyncSBool>();

  // wand menu button
  std::shared_ptr<gmNetwork::SyncSBool> sync_menu_button =
      std::make_shared<gmNetwork::SyncSBool>();

  // wand pose (position + orientation)
  std::shared_ptr<SyncSVec> sync_wand_position = std::make_shared<SyncSVec>();
  std::shared_ptr<SyncSQuat> sync_wand_orientation =
      std::make_shared<SyncSQuat>(Eigen::Quaternionf::Identity());

  // head pose (position + orientation)
  std::shared_ptr<SyncSVec> sync_head_position = std::make_shared<SyncSVec>();
  std::shared_ptr<SyncSQuat> sync_head_orientation =
      std::make_shared<SyncSQuat>(Eigen::Quaternionf::Identity());

  /// ----- Scene Graph Stuff -----

  std::shared_ptr<gmGraphics::Group> scenegraph_root;
  std::shared_ptr<gmGraphics::MatrixTransform> wand_transform_node;
  std::shared_ptr<gmGraphics::ObjRenderer> wand_object_node;
  std::shared_ptr<gmGraphics::PoseTransform> nav_transform_node;

  void initSG();
};


/// ----- External interface implementation -----

MyApp::MyApp(std::vector<std::shared_ptr<gmNetwork::SyncNode>> sync_nodes,
             std::vector<std::shared_ptr<gmTrack::Controller>> controllers,
             std::shared_ptr<gmTrack::SinglePoseTracker> head)
  : _impl(std::make_unique<Impl>(sync_nodes, controllers, head)) {}

MyApp::~MyApp() {}

std::shared_ptr<gmGraphics::Node> MyApp::getSGRoot() {
  return _impl->scenegraph_root;
}


/// ----- Internal implementation -----


MyApp::Impl::Impl(
    std::vector<std::shared_ptr<gmNetwork::SyncNode>> sync_nodes,
    std::vector<std::shared_ptr<gmTrack::Controller>> controllers,
    std::shared_ptr<gmTrack::SinglePoseTracker> head)
  : head(head) {

  setup_sync(sync_nodes);
  setup_wand(controllers);
  
  initSG();
}

void MyApp::Impl::loadObj(const std::string& filepath,
    const Eigen::Vector3f& pos,
    const Eigen::Vector3f& scale) {
    
    auto objRenderer = std::make_shared<gmGraphics::ObjRenderer>();
    objRenderer->setFile(filepath);
    objRenderer->initialize(); 

    auto transformNoder = std::make_shared<gmGraphics::MatrixTransform>();
    //auto transformNoder = std::make_shared<gmGraphics::PoseTransform>();
    transformNoder->initialize();
    //transformNode->initialize();

    //Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    //transform.scale(scale);
    //transform.translate(pos);
    Eigen::Affine3f aa = Eigen::Affine3f::Identity();
    aa.scale(scale);
    aa.translate(pos);

    transformNoder->setMatrix(aa.matrix());
    //transformNode->setMatrix(transform.matrix());

    //gmGraphics::PoseTransform transformr = gmGraphics::PoseTra
    //transformr.setPosition(pos);
    //transformr.setScale(scale);

    transformNoder->addNode(objRenderer);
    nav_transform_node->addNode(transformNoder);

}

void MyApp::Impl::setup_sync(
    std::vector<std::shared_ptr<gmNetwork::SyncNode>> sync_nodes) {

  if (!sync_nodes.empty())
    // There should be only one SyncNode, but any excess will be
    // ignored anyways.
    sync_node = sync_nodes[0];
  else {
    // The config did not provide a SyncNode, so create one! We need
    // this so that we can use the sync_* variables even on a single
    // node without peers.
    sync_node = std::make_shared<gmNetwork::SyncNode>();
    sync_node->setLocalPeerIdx(0);
    sync_node->addPeer(""); //< Anything here since our own peer will
                            //  be ignored
    sync_node->initialize();
  }

  if (sync_node->getLocalPeerIdx() != 0) is_primary = false;

  // It is good practice to limit the use of raw pointers to the scope
  // in which you got it, however gmNetwork will keep the raw pointers
  // valid until sync_node is destroyed.
  gmNetwork::DataSync *data_sync =
      sync_node->getProtocol<gmNetwork::DataSync>();

  // Do not forget to add all containers to the synchronization queue
  data_sync->addData(sync_time);
  data_sync->addData(sync_frame_number);
  data_sync->addData(sync_analogs);
  data_sync->addData(sync_main_button);
  data_sync->addData(sync_second_button);
  data_sync->addData(sync_menu_button);
  data_sync->addData(sync_head_position);
  data_sync->addData(sync_head_orientation);
  data_sync->addData(sync_wand_position);
  data_sync->addData(sync_wand_orientation);
  
  //data_sync->addData(nav_transform_node);

}

void MyApp::Impl::setup_wand(
    std::vector<std::shared_ptr<gmTrack::Controller>> controllers) {

  if (controllers.empty())
    // With no wand we are done setting up wands
    return;

  // Only the primary node should handle wand (the replica get the
  // data via network synchronization) but we keep a reference anyways
  // just so that we know to expect wand data.
  // if (!is_primary) return;

  // We could save away more than one wand, but one is enough for now
  wand = controllers[0];
}

void MyApp::Impl::checkIntersection(const Eigen::Vector3f& wandPos,
    const Eigen::Vector3f& headPos,
    const Eigen::Quaternionf& wandRot)
{

    Eigen::Vector3f rayDir = (wandPos - headPos).normalized();
    if (HOMERmode && !grabbing) {
        rayDir = wandPos - (wandRot * Eigen::Vector3f(1, 1, 1));
        rayDir.normalize();
    }


    //shoot ray from wandPos in ray direction
    gmGraphics::IntersectionLine wandRay;
    wandRay.forwardRay(wandPos, rayDir); 
    gmGraphics::IntersectionVisitor visitor(wandRay);
    float closestDist = std::numeric_limits<float>::max();

    scenegraph_root->accept(&visitor);
    gmGraphics::ObjRenderer * closestObj = nullptr;
    gmGraphics::MatrixTransform* closestObjMat = nullptr;

    for (const auto &isec : visitor.intersections)
    {
        float dist = (wandPos - isec.position).norm();

        if (dist < closestDist) {
            closestDist = dist;
            closestObj = dynamic_cast<gmGraphics::ObjRenderer*>(isec.node_path.back());

            gmGraphics::Node* tr = isec.node_path[isec.node_path.size() - 2];
            // Task 9: take second to last obj which is a transform node for the obj
            closestObjMat = dynamic_cast<gmGraphics::MatrixTransform*>(tr);
        }
    }

    //Currently check if selectedObj is nullptr and gives us psudo-state
    if (grabbing) return;

    if (closestObj != selectedObject) {
        if (selectedObject) {
            restoreOriginalMat(*selectedObject);
        }

        if (closestObj) highlightObj(*closestObj);

        selectedObject = closestObj;
        selectedObjTransformMat = closestObjMat;
    }
}//check Intersection


void MyApp::Impl::highlightObj(gmGraphics::ObjRenderer& obj) {

    originalMaterial = obj.getMaterials().front();

    //modify mat of highlighted obj
    gmGraphics::ObjRenderer::Material newMat = originalMaterial;
    newMat.color_diffuse = Eigen::Vector3f(1.0f, 0.0f, 0.0f); 
    //color_emissive does NOT work :(((
    
    obj.setMaterials({ newMat });

    //std::cout << "highlight: " << &obj << std::endl;

}

void MyApp::Impl::restoreOriginalMat(gmGraphics::ObjRenderer& obj) {
    // Restore the original material
    obj.setMaterials({ originalMaterial });
    //std::cout << "restore: " << &obj << std::endl;
}

//task 8
void MyApp::Impl::teleport(const Eigen::Vector3f& direction, const float jumpDist) {
    //teleport forward jumpDist length and rotate 180 degrees
    constexpr float PI = 3.1415;
    Eigen::Quaternionf rott(Eigen::AngleAxisf(PI, Eigen::Vector3f(0, 1, 0)));

    Eigen::Affine3f oldM = nav_transform_node->getTransform();
    Eigen::Matrix4f M = oldM.matrix();
    Eigen::Matrix3f M3 = M.topLeftCorner(3, 3);
    Eigen::Vector3f pos = M.col(3).hnormalized();// translation();
    Eigen::Quaternionf rot(M3);

    auto newRot = rott * rot;
    auto newPos = pos - direction.normalized() * jumpDist;
    nav_transform_node->setPosition(newPos);
    nav_transform_node->setOrientationCenter(newPos);
    nav_transform_node->setOrientation(newRot);


    ////sync_head_position = *sync_head_position + direction.normalized() * jumpDist;
    //sync_head_orientation = sync_head_orientation * rot;

    //nav_transform_node->setOrientation(nav_transform_node->getTransform()*rot);

}

//task 9:
void MyApp::Impl::grabObj() {
    if (!selectedObject)return;
    Eigen::Matrix4f Mmodel = selectedObjTransformMat->getMatrix().matrix();
    Eigen::Matrix4f Mscene = nav_transform_node->getTransform().matrix();
    Eigen::Matrix4f scWand = Mscene.inverse() * wand_transform_node->getMatrix();

    //initilise upon button press
    if (!grabbing) {
        // attach to scene wand
        Mdelta = scWand.inverse() * Mmodel;
        //std::cerr << "Mdelta set" << std::endl;
        grabbing = true;


    }
    //update during button press
    else {
        //update new matrix
        Eigen::Matrix4f newM = scWand * Mdelta;
        selectedObjTransformMat->setMatrix(newM);
        //std::cerr << "selectedObjTransformMat set \n" << scWand << std::endl;
    }
}

void MyApp::Impl::update(gmCore::Updateable::clock::time_point time, size_t frame) {

  // Wait until we are connected to all peers before starting to
  // update data, animate and stuff
  if (!sync_node->isConnected())
    return;

  gmNetwork::DataSync *data_sync =
      sync_node->getProtocol<gmNetwork::DataSync>();
  gmNetwork::RunSync *run_sync =
      sync_node->getProtocol<gmNetwork::RunSync>();

  update_data(time);   // Let the primary update internal data

  run_sync->wait();    // Wait for primary to have sent its values
  data_sync->update(); // Swap old values for newly synchronized

  update_states(time); // Use the data to update scenegraph states
}

void MyApp::Impl::update_data(gmCore::Updateable::clock::time_point time) {

  if (!is_primary)
    // Only primary update internal states, the rest wait for incoming
    // data via the DataSync instance.
    return;

  // Setting data to a SyncData instance (that has been added to a
  // DataSync instance) will send this value to all other nodes and
  // end up in the corresponding instance's back buffer.

  *sync_time = gmCore::TimeTools::timePointToSeconds(time);
  *sync_frame_number = *sync_frame_number + 1;

  if (head) {
    gmTrack::PoseTracker::PoseSample pose;
    if (head->getPose(pose)) {
      *sync_head_position = pose.position;
      *sync_head_orientation = pose.orientation;
    }
  }

  if (!wand)
    // Only wand stuff below this point, so terminate early if we do
    // not have a wand
    return;

  gmTrack::AnalogsTracker::AnalogsSample analogs;
  if (wand->getAnalogs(analogs))
    *sync_analogs = analogs.analogs;

  gmTrack::ButtonsTracker::ButtonsSample buttons;
  if (wand->getButtons(buttons)) {

    typedef gmTrack::ButtonsMapper::ButtonIdx ButtonIdx;

    if (buttons.buttons.count(ButtonIdx::MAIN))
      *sync_main_button = buttons.buttons[ButtonIdx::MAIN];
    else
      *sync_main_button = false;

    if (buttons.buttons.count(ButtonIdx::SECONDARY))
      *sync_second_button = buttons.buttons[ButtonIdx::SECONDARY];
    else
      *sync_second_button = false;

    if (buttons.buttons.count(ButtonIdx::MENU))
      *sync_menu_button = buttons.buttons[ButtonIdx::MENU];
    else
      *sync_menu_button = false;
  }

  gmTrack::PoseTracker::PoseSample pose;
  if (wand->getPose(pose)) {
    *sync_wand_position = pose.position;
    *sync_wand_orientation = pose.orientation;
  }
}

void MyApp::Impl::update_states(gmCore::Updateable::clock::time_point) {

  if (!wand_transform_node)
    // Cannot update wand transform since we have no wand transform
    return;

  Eigen::Vector3f eP = *sync_wand_position;
  Eigen::Quaternionf eQ = *sync_wand_orientation;
  Eigen::Vector3f headPos = *sync_head_position;
  Eigen::Vector3f direction = (eP - headPos);
  
  //task 6
  checkIntersection(eP, headPos , eQ);


  Eigen::Affine3f M = Eigen::Translation3f(eP) * eQ;
  wand_transform_node->setMatrix(M.matrix());


  double R = 0.4, G = 0.4, B = 0.4;
  if (*sync_main_button) R = 0.8;
  if (*sync_second_button) G = 0.8;
  if (*sync_menu_button) B = 0.8;

  std::vector<gmGraphics::ObjRenderer::Material> materials =
      wand_object_node->getMaterials();
  if (!materials.empty()) {
    materials.front().color_diffuse = Eigen::Vector3f(R, G, B);
    wand_object_node->setMaterials(materials);
  }

  //right click pressed
  if (*sync_main_button) {
      if (head && wand) {

          //works but you get lost very quickly(use low value for jumpDist)
          //teleport(direction, 1.0f);
          
          Eigen::Affine3f oldM = nav_transform_node->getTransform();
          auto oldPos = oldM.translation();


          //Task 10:
          if(HOMERmode){
              float moveSpeed = 0.1f;
              if(*sync_second_button){
                Eigen::Vector3f moveDirection = direction.normalized() * moveSpeed;
                nav_transform_node->setPosition(oldPos + moveDirection);
              }
          }
          else {
              //Task7: speed based on distance from wand to head (lowering speed with arbitrary value)

            const float speedScale = 0.000001f;
            const float speed = 0.1f + speedScale * direction.norm();
            direction.normalize();

            nav_transform_node->setPosition(oldPos-direction * speed);
          }
      }
  }
  //task 9:
  //supposed to be sync_second_button
  if(*sync_main_button && head && wand) grabObj();
  if (!*sync_main_button) grabbing = false;
  if (*sync_menu_button) {
      HOMERmode = !HOMERmode;  // Toggle HOMER mode
      std::cerr << "HOMER mode: " << (HOMERmode ? "Enabled" : "Disabled") << std::endl;
  }
}

//Initilise Scene Graph
void MyApp::Impl::initSG() {


  scenegraph_root = std::make_shared<gmGraphics::Group>();
  scenegraph_root->initialize();

  //add nav transform under scene root
  nav_transform_node = std::make_shared<gmGraphics::PoseTransform>();
  nav_transform_node->initialize();
  scenegraph_root->addNode(nav_transform_node);


  if (wand) {
    // We just have to assume that if a replica should render a wand,
    // because wand data come from the primary, then also the replica
    // will have a wand defined in their config, even if its data are
    // not used. How can we otherwise at setup know if we will render
    // a wand or not?

    wand_transform_node = std::make_shared<gmGraphics::MatrixTransform>();
    wand_transform_node->initialize();
    scenegraph_root->addNode(wand_transform_node);

    wand_object_node = std::make_shared<gmGraphics::ObjRenderer>();
    wand_object_node->setFile("urn:gramods:gmGraphics/resources/wand.obj");
    wand_object_node->initialize();
    wand_transform_node->addNode(wand_object_node);

    //Loading objects

    const int numObj = 4;
    const std::string Sphere = "urn:gramods:gmGraphics/resources/sphere.obj";
    const std::string Box = "urn:gramods:gmGraphics/resources/box.obj";

    for (size_t i = 0; i < numObj; i++)
    {
        //random makes it crash
        //Eigen::Vector3f randPos = Eigen::Vector3f::Random(0.0f, 1.0f);
        //Eigen::Vector3f randRot = Eigen::Vector3f::Random(0.1f, 0.3f);

        Eigen::Vector3f randPos = Eigen::Vector3f(i, 0, 0);
        Eigen::Vector3f randRot = Eigen::Vector3f(0.3,0.3,0.3);

        //every other object is sphere
        if (i % 2 == 0)loadObj(Box, randPos, randRot);
        else loadObj(Sphere, randPos, randRot);
    }

  }
}
