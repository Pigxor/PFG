#include "Scene.h"
///SUVAT - Globals
glm::vec3 u = glm::vec3(0.75f, 0.75f, 0.0f);
glm::vec3 u2 = glm::vec3(31.0f, 30.0f, 0.0f);
int landed = 0;
///

/*! \brief Brief description.
*  Scene class is a container for loading all the game objects in your simulation or your game.
*
*/
Scene::Scene()
{
	// Set up your scene here......
	// Set a camera
	_camera = new Camera();
	_camera->SetPos(glm::vec3(5.0f, 10.0f, 5.0f));
	// Don't start simulation yet
	_simulation_start = false;

	// Position of the light, in world-space
	_lightPosition = glm::vec3(10, 10, 0);

	// Create a game object
	_physics_object = new GameObject();
  // Kinematics object
  _kinematicObj = new KinematicsObject();
  _DynamicObj = new Dynamic();
	// Create a game level object
	_level = new GameObject();

	// Create the material for the game object- level
	Material *modelMaterial = new Material();
	// Shaders are now in files
	modelMaterial->LoadShaders("assets/shaders/VertShader.txt", "assets/shaders/FragShader.txt");
	// You can set some simple material properties, these values are passed to the shader
	// This colour modulates the texture colour
	modelMaterial->SetDiffuseColour(glm::vec3(0.8, 0.8, 0.8));
	// The material currently supports one texture
	// This is multiplied by all the light components (ambient, diffuse, specular)
	// Note that the diffuse colour set with the line above will be multiplied by the texture colour
	// If you want just the texture colour, use modelMaterial->SetDiffuseColour( glm::vec3(1,1,1) );
	modelMaterial->SetTexture("assets/textures/diffuse.bmp");
	// Need to tell the material the light's position
	// If you change the light's position you need to call this again
	modelMaterial->SetLightPosition(_lightPosition);
	// Tell the level object to use this material
	_level->SetMaterial(modelMaterial);

	// The mesh is the geometry for the object
	Mesh *groundMesh = new Mesh();
	// Load from OBJ file. This must have triangulated geometry
	groundMesh->LoadOBJ("assets/models/woodfloor.obj");
	// Tell the game object to use this mesh
	_level->SetMesh(groundMesh);
	_level->SetPosition(0.0f, 0.0f, 0.0f);
	_level->SetRotation(3.141590f, 0.0f, 0.0f);


	// Create the material for the game object- level
	Material *objectMaterial = new Material();
	// Shaders are now in files
	objectMaterial->LoadShaders("assets/shaders/VertShader.txt", "assets/shaders/FragShader.txt");
	// You can set some simple material properties, these values are passed to the shader
	// This colour modulates the texture colour
	objectMaterial->SetDiffuseColour(glm::vec3(0.8, 0.1, 0.1));
	// The material currently supports one texture
	// This is multiplied by all the light components (ambient, diffuse, specular)
	// Note that the diffuse colour set with the line above will be multiplied by the texture colour
	// If you want just the texture colour, use modelMaterial->SetDiffuseColour( glm::vec3(1,1,1) );
	objectMaterial->SetTexture("assets/textures/default.bmp");
	// Need to tell the material the light's position
	// If you change the light's position you need to call this again
	objectMaterial->SetLightPosition(_lightPosition);
	// Tell the level object to use this material
  _physics_object->SetMaterial(objectMaterial);
  _kinematicObj->SetMaterial(objectMaterial);
  _DynamicObj->SetMaterial(objectMaterial);

	// Set the geometry for the object
	Mesh *modelMesh = new Mesh();
	// Load from OBJ file. This must have triangulated geometry
	modelMesh->LoadOBJ("assets/models/sphere.obj");

	
  // Tell the game object to use this mesh
	_physics_object->SetMesh(modelMesh);
	_physics_object->SetPosition(0.0f, 5.0f, 1.0f);
	_physics_object->SetScale(0.3f, 0.3f, 0.3f);

  _kinematicObj->SetMesh(modelMesh);
  _kinematicObj->SetPosition(0.0f, 5.0f, -1.0f);
  _kinematicObj->SetScale(0.3f, 0.3f, 0.3f);

  _DynamicObj->SetMesh(modelMesh);
  _DynamicObj->SetPosition(0.0f, 15.0f, 0.0f);
  _DynamicObj->SetScale(0.3f, 0.3f, 0.3f);
  _DynamicObj->SetMass(1.0f);


  _kinematicObj->SetVelocity(u);


	
}

Scene::~Scene()
{
	// You should neatly clean everything up here
	delete _physics_object;
  delete _kinematicObj;
  delete _DynamicObj;
	delete _level;
	delete _camera;

}


void Scene::Update(float deltaTs, Input* input)
{
	if (input->cmd_x)
	{
		_simulation_start = true;
    _kinematicObj->StartSimulation(true);
    _DynamicObj->StartSimulation(true);
	}
	//if (_simulation_start == true)
	//{
	//	glm::vec3 pos = _physics_object->GetPosition();
	//	glm::vec3 a = glm::vec3(0.0, -9.81, 0.0);
	//	glm::vec3 v = u + (a * (deltaTs));
	//	u = v;
	//	if(landed == 0) //Has it hit the floor?
	//	{
	//		pos += v; //If in the air, keep moving - stops once it hits the floor
	//	}
	//	_physics_object->SetPosition(pos);
	//	//General Purpose Hit Detection
	//	glm::vec3 ball = _physics_object->GetPosition();
	//	glm::vec3 floor = _level->GetPosition();
	//	glm::vec3 ballScale = _physics_object->GetScale();
	//	float floorHit = floor.y + ballScale.y +(ballScale.y / 3);
	//	if (ball.y <= floorHit)
	//	{
	//		landed++;
	//		glm::vec3 ballFinal = _physics_object->GetPosition();
	//		_physics_object->SetPosition(ballFinal.x, floorHit, ballFinal.z);
	//	}
	//}
	_physics_object->Update(deltaTs);
  _kinematicObj->Update(deltaTs, _level);
  _DynamicObj->Update(deltaTs);
	_level->Update(deltaTs);
	_camera->Update(input);
	_viewMatrix = _camera->GetView();
	_projMatrix = _camera->GetProj();

														
}

void Scene::Draw()
{
	// Draw objects, giving the camera's position and projection
	//_physics_object->Draw(_viewMatrix, _projMatrix);
  _kinematicObj->Draw(_viewMatrix, _projMatrix);
  _DynamicObj->Draw(_viewMatrix, _projMatrix);
	_level->Draw(_viewMatrix, _projMatrix);

}


