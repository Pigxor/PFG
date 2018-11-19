#include <GLM/gtc/type_ptr.hpp>
#include <GLM/gtc/matrix_transform.hpp>
#include "Dynamic.h"


Dynamic::Dynamic()
{
  _velocity = glm::vec3(1.0, 5.0, 0);
  _torque = glm::vec3(0.0, 0.0, 0.0);
  _angularVel = glm::vec3(0.0, 0.0, 0.0);
  _angularMom = glm::vec3(0.0, 0.0, 0.0);
  _R = glm::mat3(1.0f);
  ComputeInertiaTensor();
  _angularMom = glm::vec3(0.0f, 0.0f, 0.0f);
  _angularVel = _inertiaTensorInv * _angularMom;
  bodyTensor = glm::mat3
  {
    (2 / 5)*mass*(pow(_scale.y,2)),0,0,
    0,(2 / 5)*mass*(pow(_scale.y,2)),0,
    0,0,(2 / 5)*mass*(pow(_scale.y,2))
  };
}

Dynamic::~Dynamic()
{
}

void Dynamic::Update(float deltaTs)
{
  if (_start == true)
  {
    glm::vec3 pos = GetPosition();
    prevPos = GetPosition() - (_velocity*deltaTs);
    ///Clear
    ClearForces();
    ///Compute
    glm::vec3 g = glm::vec3(0.0, -9.81, 0.0);
    glm::vec3 gravF = mass * g;
    AddForce(gravF);
    glm::vec3 tau = glm::cross(_force, pos - prevPos);
    AddTorque(tau);
    ///Collision
    //General Purpose Hit Detection
    CollisionResponse(deltaTs);
    if (_landed == 0)
    {
    ///timeIntegration
    //RungeKutta4(deltaTs);
    //RungeKutta2(deltaTs);
    Euler(deltaTs);
    }
  }
  SetScale(0.3f, 0.3f, 0.3f);
  UpdateModelMatrix();
}

void Dynamic::AddForce(glm::vec3 newForce)
{
  _force += newForce;
}

void Dynamic::ComputeInertiaTensor()
{

  //_inertiaTensor = _R * bodyTensor * glm::transpose(_R);
  _bodyInertiaTensorInv = _R * glm::inverse(bodyTensor) * glm::transpose(_R);

}

void Dynamic::Euler(float deltaTs)
{
  _velocity += (_force/mass)*(deltaTs);
  _position += _velocity * (deltaTs);
  _angularMom += _torque*deltaTs;
  ComputeInertiaTensor();
  _angularVel += _inertiaTensorInv * _angularMom;
  _SkewAngVel = glm::mat3
  {
    0,(-_angularVel.z),(_angularVel.y),
    (_angularVel.z),0,(-_angularVel.x),
    (-_angularVel.y),(_angularVel.x),0
  };
  _R += (deltaTs* _SkewAngVel *_R);
}

void Dynamic::UpdateModelMatrix()
{
  glm::mat4 model_rotation = glm::mat4(_R);
  _modelMatrix = glm::translate(glm::mat4(1.0f), _position);
  _modelMatrix = _modelMatrix*model_rotation;
  _modelMatrix = glm::scale(_modelMatrix, _scale);

  _invModelMatrix = glm::inverse(_modelMatrix);

}

void Dynamic::CollisionResponse(float deltaTs)
{
  glm::vec3 position = GetPosition();

  glm::vec3 n(0.0f, 1.0f, 0.0f);
  glm::vec3 q(0.0f, 0.0f, 0.0f);
  float d = PFG::DistToPlane(n, position, q);
  float bRadius = GetScale().y;

  glm::vec3 Vel = GetVelocity();
  glm::vec3 ContactPos;
  glm::vec3 c1 = position + Vel * deltaTs;
  glm::vec3 c0 = position;
  bool collision = PFG::MovSpherePlaneColl2(n, c0, c1, q, bRadius, ContactPos);

  if (collision)
  {
    float TopDiv = -(1.0f + 0.8f) * glm::dot((GetVelocity() - glm::vec3(0.0f, 0.0f, 0.0f)), n);
    float massInv1 = 1 / GetMass();
    float massFlInv = 0.0f;
    float ImpulseJ = TopDiv / (massInv1+massFlInv);
    glm::vec3 ImpForce = ImpulseJ * n / deltaTs;
    glm::vec3 contactForce(0.0f, 9.81f*GetMass(), 0.0f);
    glm::vec3 responseForce = ImpForce + contactForce;
    AddForce(responseForce);

    if (responseForce.y < 0.1)
    {
      glm::vec3 ballScale = GetScale();
      float floorHit = q.y + ballScale.y;
      glm::vec3 ballFinal = GetPosition();
      SetPosition(ballFinal.x, floorHit, ballFinal.z);
      SetLanded(1);
    }
  }

}

void Dynamic::RungeKutta2(float deltaTs)
{
  glm::vec3 force;
  glm::vec3 accel;
  glm::vec3 k0;
  glm::vec3 k1;

  force = _force;
  accel = force / mass;
  k0 = deltaTs * accel;

  force = _force + k0 / 2.0f;
  accel = force / mass;
  k1 = deltaTs * accel;

  _velocity += k1;
  _position += _velocity * deltaTs;
}

void Dynamic::RungeKutta4(float deltaTs)
{
  glm::vec3 force;
  glm::vec3 accel;
  glm::vec3 k0;
  glm::vec3 k1;
  glm::vec3 k2;
  glm::vec3 k3;

  force = _force;
  accel = force / mass;
  k0 = deltaTs * accel;

  force = _force + k0 / 2.0f;
  accel = force / mass;
  k1 = deltaTs * accel;

  force = _force + k1 / 2.0f;
  accel = force / mass;
  k2 = deltaTs * accel;

  force = _force + k2;
  accel = force / mass;
  k3 = deltaTs * accel;

  _velocity += (k0 + 2.0f * k1 + 2.0f * k2 + k3) / 6.0f;
  _position += _velocity * deltaTs;
  _angularMom += _torque*deltaTs;
  ComputeInertiaTensor();
  _angularVel += _inertiaTensorInv * _angularMom;
  _SkewAngVel = glm::mat3
  {
    0,(-_angularVel.z),(_angularVel.y),
    (_angularVel.z),0,(-_angularVel.x),
    (-_angularVel.y),(_angularVel.x),0
  };
  _R += (deltaTs* _SkewAngVel *_R);

}
