#include <GLM/gtc/type_ptr.hpp>
#include <GLM/gtc/matrix_transform.hpp>
#include "KinematicsObject.h"

KinematicsObject::KinematicsObject()
{
}

KinematicsObject::~KinematicsObject()
{
}

void KinematicsObject::Update(float deltaTs, GameObject* level)
{
  if (_start == true)
  {
    glm::vec3 pos = GetPosition();
    glm::vec3 a = glm::vec3(0.0, -9.81, 0.0);
    glm::vec3 v = _velocity + (a * (deltaTs));
    SetVelocity(v);
    if (GetLanded() == 0) //Has it hit the floor?
    {
      SetPosition(pos += v); //If in the air, keep moving - stops once it hits the floor
    }
    
    //General Purpose Hit Detection
    glm::vec3 ball = GetPosition();
    glm::vec3 floor = level->GetPosition();
    glm::vec3 ballScale = GetScale();
    float floorHit = floor.y + ballScale.y + (ballScale.y / 3);
    if (ball.y <= floorHit)
    {
      SetLanded(1);
      glm::vec3 ballFinal = GetPosition();
      SetPosition(ballFinal.x, floorHit, ballFinal.z);
    }
    //

  }
  UpdateModelMatrix();
}

void KinematicsObject::UpdateModelMatrix()
{
  _modelMatrix = glm::translate(glm::mat4(1.0f), _position);
  _modelMatrix = glm::scale(_modelMatrix, _scale);
}
